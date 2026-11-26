/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <bias/eeprom_schema.h>

/* Thread context. Load publishes the descriptor only after validation.
 * Store validates the header and calculates the wire CRC, ignoring input crc32.
 * A failed write may leave an incomplete descriptor. Retry the entire store.
 */
int eeprom_board_load(struct bias_eeprom_descriptor_v1 *descriptor);
int eeprom_board_store(const struct bias_eeprom_descriptor_v1 *descriptor);
