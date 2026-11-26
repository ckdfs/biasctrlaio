/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <bias/analog_board.h>
#include <ctrl_observer.h>

/* Optional record transport. Descriptor I/O remains in analog_board_ops_t.
 * write_page receives one aligned page (or a final partial page) and must
 * complete its write cycle before returning. */
typedef struct {
	int (*read)(uint8_t offset, void *data, size_t len);
	int (*write_page)(uint8_t offset, const void *data, size_t len);
} cal_storage_ops_t;

int cal_service_voltage_bounds(const struct bias_eeprom_descriptor_v1 *desc,
			       float *min_v, float *max_v);
int cal_service_voltage_to_dac_code(const struct bias_eeprom_descriptor_v1 *desc,
				    float voltage, uint16_t *code);
int cal_service_load_descriptor(const analog_board_ops_t *board, struct bias_eeprom_descriptor_v1 *desc);
/* NULL storage uses U21. Outputs are published only after validation. */
int cal_service_load(const analog_board_ops_t *board, const cal_storage_ops_t *storage,
		     struct bias_eeprom_descriptor_v1 *desc, ctrl_calibration_t *cal);
int cal_service_save(const analog_board_ops_t *board, const cal_storage_ops_t *storage,
		     struct bias_eeprom_descriptor_v1 *desc, const ctrl_calibration_t *cal);
