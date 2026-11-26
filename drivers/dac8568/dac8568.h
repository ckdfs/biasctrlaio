/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BIAS_DAC8568_H
#define BIAS_DAC8568_H

#include <stdint.h>

/* Boot initializes the device and tests 100 DMA writes. Returns zero when
 * ready, -EAGAIN before initialization, or the latched hardware error.
 */
int dac8568_init(void);

/* Thread context. Returns after LDAC# rises. DC 0..5 select VA..VF.
 * Both write APIs return -EBUSY if another transfer owns the DAC.
 */
int dac8568_set_dc_code(uint8_t dc, uint16_t code);

/* ISR or thread. Submits DC channel 0..5 asynchronously (commits LDAC# on next
 * submission or dac8568_status() poll after EOT). Returns -EBUSY if busy.
 */
int dac8568_set_dc_code_async(uint8_t dc, uint16_t code);

/* ISR or thread. Pilot 0/1 select DITHER1/2. Zero means submitted.
 * Poll status for completion or inspect the next submission's result.
 */
int dac8568_set_pilot_code(uint8_t pilot, uint16_t code);

/* Zero when idle, -EBUSY during a write, or a latched negative errno. */
int dac8568_status(void);

#endif
