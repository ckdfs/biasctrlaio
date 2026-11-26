/* SPDX-License-Identifier: Apache-2.0 */
#include "dac8568.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/atomic.h>
#include <stm32_ll_dma.h>
#include <stm32_ll_spi.h>

#define DAC DT_NODELABEL(bias_dac)
#define BUS DT_BUS(DAC)
#define DMA_NODE DT_DMAS_CTLR_BY_NAME(BUS, tx)
#define CHANNEL DT_DMAS_CELL_BY_NAME(BUS, tx, channel)
#define DMA_ERRORS               (DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF | DMA_CSR_TOF)
#define DMA_CLEAR_FLAGS          (DMA_ERRORS | DMA_CSR_TCF | DMA_CSR_HTF | DMA_CSR_SUSPF)
#define DAC8568_CMD_WRITE_UPDATE 0x03U
#define DAC8568_CMD_RESET        0x07U
#define DAC8568_CH_ALL           0x0FU
#define DAC8568_MIDSCALE         32768U
#define RESET_CMD                ((uint32_t)DAC8568_CMD_RESET << 24)
#define REF_ENABLE               0x08000001U

/* The fixed-address TX word and the peripheral belong to this driver. */
BUILD_ASSERT(DT_CHILD_NUM_STATUS_OKAY(BUS) == 1, "DAC needs a dedicated SPI bus");
BUILD_ASSERT(DT_PROP(DAC, spi_cpha) && !DT_PROP(DAC, spi_cpol));
static SPI_TypeDef *const spi = (SPI_TypeDef *)DT_REG_ADDR(BUS);
static DMA_Channel_TypeDef *const dma_ch =
	LL_DMA_GET_CHANNEL_INSTANCE(DT_REG_ADDR(DMA_NODE), CHANNEL);
static const struct device *const dma = DEVICE_DT_GET(DMA_NODE);
static const struct gpio_dt_spec sync_pin = SPI_CS_GPIOS_DT_SPEC_GET(DAC);
static const struct gpio_dt_spec ldac = GPIO_DT_SPEC_GET(DAC, ldac_gpios);
static const struct gpio_dt_spec clr = GPIO_DT_SPEC_GET(DAC, clr_gpios);
static uint32_t tx_word;

/* Negative states latch faults until reboot. Only the waiting caller releases
 * SYNC_DONE, so another submitter cannot consume its completion.
 */
enum tx_state { IDLE, SYNC_PENDING, SYNC_DONE, PILOT_PENDING };
static int state = -EAGAIN;
static atomic_t boot_result = ATOMIC_INIT(-EAGAIN);

static int fault_locked(int err)
{
	LL_SPI_Disable(spi);
	if (dma_ch->CCR & DMA_CCR_EN) {
		dma_ch->CCR |= DMA_CCR_RESET;
	}
	dma_ch->CFCR = DMA_CLEAR_FLAGS;
	gpio_pin_set_dt(&sync_pin, 0);
	state = err;
	return err;
}

static void finish_if_eot_locked(void)
{
	if (state != SYNC_PENDING && state != PILOT_PENDING) {
		return;
	}
	uint32_t csr = dma_ch->CSR;
	uint32_t sr = spi->SR;

	if ((csr & DMA_ERRORS) || (sr & (SPI_SR_MODF | SPI_SR_TIFRE | SPI_SR_UDR))) {
		fault_locked(-EIO);
		return;
	}
	if ((csr & DMA_CSR_TCF) && (sr & SPI_SR_EOT)) {
		int owner = state;

		LL_SPI_Disable(spi);
		dma_ch->CFCR = DMA_CLEAR_FLAGS;
		int rc = gpio_pin_set_dt(&sync_pin, 0);

		if (rc == 0) {
			rc = gpio_pin_set_dt(&ldac, 1);
		}
		if (rc == 0) {
			/* > 128 ns + GPIO overhead for low-speed FPC2 PB2 LDAC# edge. */
			for (unsigned int i = 0; i < 32; ++i) {
				__NOP();
			}
			rc = gpio_pin_set_dt(&ldac, 0);
		}
		state = rc ? rc : (owner == SYNC_PENDING ? SYNC_DONE : IDLE);
	}
}

static int submit(uint32_t word, enum tx_state owner)
{
	unsigned int key = irq_lock();

	finish_if_eot_locked();
	if (state != IDLE) {
		int rc = state < 0 ? state : -EBUSY;

		irq_unlock(key);
		return rc;
	}
	if (dma_ch->CCR & DMA_CCR_EN) {
		int rc = fault_locked(-EIO);

		irq_unlock(key);
		return rc;
	}
	tx_word = word;
	__DMB();
	dma_ch->CFCR = DMA_CLEAR_FLAGS;
	dma_ch->CBR1 = sizeof(tx_word);
	LL_SPI_ClearFlag_EOT(spi);
	LL_SPI_ClearFlag_TXTF(spi);
	LL_SPI_SetTransferSize(spi, 1);
	int rc = gpio_pin_set_dt(&sync_pin, 1);

	if (rc < 0) {
		rc = fault_locked(rc);
		irq_unlock(key);
		return rc;
	}
	dma_ch->CCR |= DMA_CCR_EN;
	LL_SPI_Enable(spi);
	LL_SPI_StartMasterTransfer(spi);
	state = owner;
	irq_unlock(key);
	return 0;
}

static int write_sync(uint32_t word)
{
	int rc = submit(word, SYNC_PENDING);

	if (rc < 0) {
		return rc;
	}
	uint32_t start = k_cycle_get_32();

	for (;;) {
		unsigned int key = irq_lock();

		finish_if_eot_locked();
		int current = state;

		if (current == SYNC_DONE) {
			state = IDLE;
			irq_unlock(key);
			return 0;
		}
		if (current != SYNC_PENDING) {
			irq_unlock(key);
			return current < 0 ? current : -EIO;
		}
		if (k_cycle_get_32() - start >= k_us_to_cyc_ceil32(1000)) {
			rc = fault_locked(-ETIMEDOUT);
			irq_unlock(key);
			return rc;
		}
		irq_unlock(key);
	}
}

static uint32_t write_update_frame(uint8_t channel, uint16_t code)
{
	/* Command 3 writes the input register and immediately updates the DAC
	 * output on SYNC# rising edge (matches biascontrol_h523).
	 */
	return ((uint32_t)DAC8568_CMD_WRITE_UPDATE << 24) |
	       ((uint32_t)(channel & 0x0FU) << 20) |
	       ((uint32_t)code << 4);
}

int dac8568_init(void)
{
	return atomic_get(&boot_result);
}

int dac8568_status(void)
{
	int rc = dac8568_init();

	if (rc == 0) {
		unsigned int key = irq_lock();

		finish_if_eot_locked();
		rc = state;
		irq_unlock(key);
	}
	return rc > 0 ? -EBUSY : rc;
}

int dac8568_set_pilot_code(uint8_t pilot, uint16_t code)
{
	if (pilot >= 2) {
		return -EINVAL;
	}
	int rc = dac8568_init();

	return rc ? rc : submit(write_update_frame(6 + pilot, code), PILOT_PENDING);
}

int dac8568_set_dc_code(uint8_t dc, uint16_t code)
{
	if (dc >= 6 || k_is_in_isr()) {
		return dc >= 6 ? -EINVAL : -EWOULDBLOCK;
	}
	int rc = dac8568_init();

	return rc ? rc : write_sync(write_update_frame(dc, code));
}

int dac8568_set_dc_code_async(uint8_t dc, uint16_t code)
{
	if (dc >= 6) {
		return -EINVAL;
	}
	int rc = dac8568_init();

	return rc ? rc : submit(write_update_frame(dc, code), PILOT_PENDING);
}

static int configure(void)
{
	const struct device *bus = DEVICE_DT_GET(BUS);
	const struct stm32_pclken clocks[] = STM32_DT_CLOCKS(BUS);
	uint32_t hz;

	if (!device_is_ready(bus) || !device_is_ready(dma) ||
	    !gpio_is_ready_dt(&sync_pin) || !gpio_is_ready_dt(&clr) ||
	    !gpio_is_ready_dt(&ldac)) {
		return -ENODEV;
	}
	int rc = pm_device_runtime_get(bus);

	if (rc < 0) {
		return rc;
	}
	rc = clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				    (clock_control_subsys_t)&clocks[1], &hz);
	if (rc < 0) {
		return rc;
	}
	unsigned int br = 0;

	while (br < 8 && DIV_ROUND_UP(hz, 2U << br) > DT_PROP(DAC, spi_max_frequency)) {
		++br;
	}
	if (br == 8 || hz == 0) {
		return -ERANGE;
	}
	const struct gpio_dt_spec *pins[] = {&sync_pin, &ldac, &clr};

	for (size_t i = 0; i < ARRAY_SIZE(pins); ++i) {
		rc = gpio_pin_configure_dt(pins[i], GPIO_OUTPUT_INACTIVE);
		if (rc < 0) {
			return rc;
		}
	}

	if (dma_ch->CCR & DMA_CCR_EN) {
		return -EBUSY;
	}
	dma_ch->CCR = 0;
	dma_ch->CFCR = DMA_CLEAR_FLAGS;
	dma_ch->CTR1 = LL_DMA_SRC_DATAWIDTH_WORD | LL_DMA_DEST_DATAWIDTH_WORD;
	dma_ch->CTR2 = DT_DMAS_CELL_BY_NAME(BUS, tx, slot) | DMA_CTR2_DREQ;
	dma_ch->CLLR = 0;
	dma_ch->CSAR = (uintptr_t)&tx_word;
	dma_ch->CDAR = (uintptr_t)&spi->TXDR;

	LL_SPI_Disable(spi);
	LL_SPI_SetNSSMode(spi, LL_SPI_NSS_SOFT);
	LL_SPI_SetInternalSSLevel(spi, LL_SPI_SS_LEVEL_HIGH);
	LL_SPI_ClearFlag_MODF(spi);
	LL_SPI_ClearFlag_OVR(spi);
	LL_SPI_ClearFlag_UDR(spi);
	LL_SPI_ClearFlag_FRE(spi);
	LL_SPI_ClearFlag_EOT(spi);
	LL_SPI_ClearFlag_TXTF(spi);
	LL_SPI_SetMode(spi, LL_SPI_MODE_MASTER);
	LL_SPI_SetTransferDirection(spi, LL_SPI_SIMPLEX_TX);
	LL_SPI_SetTransferBitOrder(spi, LL_SPI_MSB_FIRST);
	LL_SPI_SetClockPolarity(spi, LL_SPI_POLARITY_LOW);
	LL_SPI_SetClockPhase(spi, LL_SPI_PHASE_2EDGE);
	LL_SPI_SetBaudRatePrescaler(spi, br << SPI_CFG1_MBR_Pos);
	LL_SPI_SetDataWidth(spi, LL_SPI_DATAWIDTH_32BIT);
	LL_SPI_SetFIFOThreshold(spi, LL_SPI_FIFO_TH_01DATA);
	LL_SPI_EnableGPIOControl(spi);
	LL_SPI_EnableDMAReq_TX(spi);
	return 0;
}

static int startup(const struct device *dev)
{
	ARG_UNUSED(dev);
	int rc = configure();

	if (rc == 0) {
		unsigned int key = irq_lock();

		state = IDLE;
		irq_unlock(key);
		k_busy_wait(1000);
		rc = write_sync(RESET_CMD);
	}
	if (rc == 0) {
		k_busy_wait(2000);
		rc = write_sync(REF_ENABLE);
	}
	if (rc == 0) {
		k_busy_wait(5000);
		rc = write_sync(write_update_frame(DAC8568_CH_ALL, DAC8568_MIDSCALE));
	}
	for (unsigned int i = 0; rc == 0 && i < 100; ++i) {
		rc = write_sync(write_update_frame(i % 8, DAC8568_MIDSCALE));
		if (rc < 0) {
			printk("DAC8568 smoke write %u/100 failed (%d)\n", i + 1, rc);
		}
	}
	if (rc < 0) {
		printk("DAC8568 startup failed (%d)\n", rc);
	}
	atomic_set(&boot_result, rc);
	return rc;
}

DEVICE_DT_DEFINE(DAC, startup, NULL, NULL, NULL, POST_KERNEL, 90, NULL);
