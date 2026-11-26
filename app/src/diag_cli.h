/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

struct diag_boot_status {
	int board_rc;
	int oled_rc;
	int keys_rc;
	int eeprom_rc;
	int adc_rc;
};

int diag_cli_init(const struct diag_boot_status *boot);
void diag_cli_poll(void);
