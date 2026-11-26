/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BIAS_ADS131M02_DMA_H
#define BIAS_ADS131M02_DMA_H

#include <stdint.h>

#define ADS_FRAME_BYTES 12
int ads_dma_prepare(void);
int ads_dma_complete(void);
void ads_dma_launch(const uint8_t *tx, uint8_t *rx);
int ads_dma_shutdown(void);

#endif
