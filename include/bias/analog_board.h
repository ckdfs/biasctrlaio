/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BIAS_ANALOG_BOARD_H
#define BIAS_ANALOG_BOARD_H

#include <stdbool.h>
#include <stdint.h>
#include <bias/eeprom_schema.h>

/* Indices are logical and zero-based. For 20261110-MZM the DC indices
 * denote VA..VF and pilot indices denote DITHER1..DITHER2.
 */
struct analog_pilot_route {
	uint8_t pilot;
	uint8_t dc;
	uint8_t switch_index; /* Index in the controller's pilot-sw-gpios array. */
};

struct analog_board_caps {
	const char *board_id;
	uint8_t dc_count;
	uint8_t pilot_count;
	uint8_t route_count;
	const uint8_t *dc_dac_channels;    /* dc_count entries */
	const uint8_t *pilot_dac_channels; /* pilot_count entries */
	const struct analog_pilot_route *routes; /* route_count entries */
};

/* The backend owns these tables for its lifetime. Operations return zero
 * on success or negative errno. Init disconnects every pilot route before
 * loading calibration. On a fault the backend disconnects all routes.
 * Calibration access and initialization run in thread context.
 */
typedef struct analog_board_ops {
	const struct analog_board_caps *caps;
	int (*init)(void);
	int (*set_dc_code)(uint8_t dc, uint16_t code);
	int (*set_pilot_code)(uint8_t pilot, uint16_t code);
	int (*set_route)(uint8_t pilot, uint8_t dc, bool enabled);
	int (*disconnect_all)(void);
	/* Load validates magic, version, length, board ID and CRC before use.
	 * Store computes the CRC of the serialized descriptor.
	 */
	int (*load_calibration)(struct bias_eeprom_descriptor_v1 *descriptor);
	int (*store_calibration)(const struct bias_eeprom_descriptor_v1 *descriptor);
} analog_board_ops_t;

#ifdef __cplusplus
extern "C" {
#endif
/* Implemented by the selected daughterboard backend. */
const analog_board_ops_t *analog_board_get_ops(void);
#ifdef __cplusplus
}
#endif

#endif /* BIAS_ANALOG_BOARD_H */
