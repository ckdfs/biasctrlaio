/* SPDX-License-Identifier: Apache-2.0 */
/* STM32H5 backend. The caller holds the SPI bus throughout acquisition.
 * DMA completion is consumed at the next DRDY edge, avoiding two additional
 * DMA interrupts per sample. The channels are dedicated to this SPI bus in DT.
 */
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/kernel.h>
#include <stm32_ll_dma.h>
#include <stm32_ll_spi.h>
#include "ads131m02_dma.h"

#define BUS DT_BUS(DT_NODELABEL(bias_adc))
#define DMA_NODE DT_DMAS_CTLR_BY_NAME(BUS, rx)
#define CELL(dir, field) DT_DMAS_CELL_BY_NAME(BUS, dir, field)
#define CHANNEL(dir) LL_DMA_GET_CHANNEL_INSTANCE(DT_REG_ADDR(DMA_NODE), CELL(dir, channel))
#define PRIORITY(dir) (STM32_DMA_CONFIG_PRIORITY(CELL(dir, channel_config)) << DMA_CCR_PRIO_Pos)
#define DMA_ERRORS (DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF | DMA_CSR_TOF)
#define CLEAR_FLAGS (DMA_ERRORS | DMA_CSR_TCF | DMA_CSR_HTF | DMA_CSR_SUSPF)

BUILD_ASSERT(DT_SAME_NODE(DMA_NODE, DT_DMAS_CTLR_BY_NAME(BUS, tx)));
BUILD_ASSERT(CELL(rx, channel) != CELL(tx, channel));
static SPI_TypeDef *const spi = (SPI_TypeDef *)DT_REG_ADDR(BUS);

int ads_dma_prepare(void)
{
	DMA_Channel_TypeDef *rx = CHANNEL(rx);
	DMA_Channel_TypeDef *tx = CHANNEL(tx);

	if (!device_is_ready(DEVICE_DT_GET(DMA_NODE))) {
		return -ENODEV;
	}
	if (((rx->CCR | tx->CCR) & DMA_CCR_EN) || (spi->CR1 & SPI_CR1_SPE)) {
		return -EBUSY;
	}

	/* Byte-wide, single-beat, one block. Peripheral address stays fixed. */
	rx->CCR = PRIORITY(rx);
	tx->CCR = PRIORITY(tx);
	rx->CTR1 = DMA_CTR1_DINC;
	tx->CTR1 = DMA_CTR1_SINC;
	rx->CTR2 = CELL(rx, slot);
	tx->CTR2 = CELL(tx, slot) | DMA_CTR2_DREQ;
	rx->CLLR = 0;
	tx->CLLR = 0;
	rx->CSAR = (uintptr_t)&spi->RXDR;
	tx->CDAR = (uintptr_t)&spi->TXDR;
	LL_SPI_SetFIFOThreshold(spi, LL_SPI_FIFO_TH_01DATA);
	LL_SPI_SetTransferSize(spi, ADS_FRAME_BYTES);
	LL_SPI_EnableDMAReq_RX(spi);
	LL_SPI_EnableDMAReq_TX(spi);
	return 0;
}

int ads_dma_complete(void)
{
	DMA_Channel_TypeDef *rx_ch = CHANNEL(rx);
	DMA_Channel_TypeDef *tx_ch = CHANNEL(tx);

	(void)WAIT_FOR(((rx_ch->CSR & tx_ch->CSR & DMA_CSR_TCF) && (spi->SR & SPI_SR_EOT)) ||
			       ((rx_ch->CSR | tx_ch->CSR) & DMA_ERRORS) ||
			       (spi->SR & (SPI_SR_OVR | SPI_SR_UDR | SPI_SR_MODF)),
		       2, (void)0);

	uint32_t rx = rx_ch->CSR;
	uint32_t tx = tx_ch->CSR;
	uint32_t sr = spi->SR;

	if (((rx | tx) & DMA_ERRORS) || (sr & (SPI_SR_OVR | SPI_SR_UDR | SPI_SR_MODF))) {
		return -EIO;
	}
	if (!(rx & tx & DMA_CSR_TCF) || !(sr & SPI_SR_EOT)) {
		return -EAGAIN;
	}
	__DMB();
	return 0;
}

void ads_dma_launch(const uint8_t *txbuf, uint8_t *rxbuf)
{
	DMA_Channel_TypeDef *rx = CHANNEL(rx);
	DMA_Channel_TypeDef *tx = CHANNEL(tx);

	LL_SPI_Disable(spi);
	spi->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC;
	rx->CFCR = CLEAR_FLAGS;
	tx->CFCR = CLEAR_FLAGS;
	rx->CDAR = (uintptr_t)rxbuf;
	tx->CSAR = (uintptr_t)txbuf;
	rx->CBR1 = ADS_FRAME_BYTES;
	tx->CBR1 = ADS_FRAME_BYTES;
	__DMB();
	rx->CCR |= DMA_CCR_EN;
	tx->CCR |= DMA_CCR_EN;
	LL_SPI_Enable(spi);
	LL_SPI_StartMasterTransfer(spi);
}

static int quiesce(DMA_Channel_TypeDef *channel)
{
	if (channel->CCR & DMA_CCR_EN) {
		channel->CCR |= DMA_CCR_SUSP;
		if (!WAIT_FOR((channel->CSR & (DMA_CSR_SUSPF | DMA_CSR_IDLEF)), 100, (void)0)) {
			return -ETIMEDOUT;
		}
		channel->CCR |= DMA_CCR_RESET;
		if (!WAIT_FOR(!(channel->CCR & DMA_CCR_EN), 100, (void)0)) {
			return -ETIMEDOUT;
		}
	}
	channel->CFCR = CLEAR_FLAGS;
	return 0;
}

int ads_dma_shutdown(void)
{
	/* Thread context. Give the final wire transfer time to finish. */
	if (spi->CR1 & SPI_CR1_CSTART) {
		k_busy_wait(25);
	}
	int rx = quiesce(CHANNEL(rx));
	int tx = quiesce(CHANNEL(tx));

	LL_SPI_Disable(spi);
	LL_SPI_DisableDMAReq_RX(spi);
	LL_SPI_DisableDMAReq_TX(spi);
	LL_SPI_SetTransferSize(spi, 0);
	spi->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_OVRC | SPI_IFCR_UDRC |
		    SPI_IFCR_MODFC;
	return rx ? rx : tx;
}
