/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZTEST_MOCKS_H
#define ZTEST_MOCKS_H

#include "cal_service.h"
#include <ads131m02.h>
#include <analog_board_mzm.h>
#include <bias/analog_board.h>
#include <stdbool.h>
#include <stdint.h>

struct mock_board_state {
	uint8_t eeprom[BIAS_EEPROM_CAPACITY_BYTES];
	uint16_t dc_code[6];
	uint16_t pilot_code[2];
	bool route[2][6];
	uint32_t disconnect_count;
	uint32_t store_cal_count;
	uint32_t load_cal_count;
	uint32_t write_page_count;
	uint32_t read_count;
	int set_dc_fail_rc;
	int set_pilot_fail_rc;
	int set_route_fail_rc;
	int disconnect_fail_rc;
	int load_cal_fail_rc;
	int store_cal_fail_rc;
	int storage_read_fail_rc;
	int storage_write_fail_rc;
};

extern struct mock_board_state mock_state;
extern const analog_board_ops_t mock_board_ops;
extern const cal_storage_ops_t mock_storage_ops;

void mock_reset_all(void);
void mock_install_mzm_descriptor(int32_t offset_uv, int32_t gain_ppm, uint32_t vpi_uv);

#endif /* ZTEST_MOCKS_H */
