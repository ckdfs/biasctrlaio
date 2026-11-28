#ifndef CTRL_MODULATOR_H
#define CTRL_MODULATOR_H

#include "dsp_types.h"
#include <stdbool.h>

enum { CTRL_DC_CHANNELS = 6, CTRL_PILOTS = 2 };
typedef enum { MOD_TYPE_MZM, MOD_TYPE_DPMZM, MOD_TYPE_DPQPSK } modulator_type_t;
typedef enum { BIAS_POINT_QUAD, BIAS_POINT_MAX, BIAS_POINT_MIN, BIAS_POINT_CUSTOM } bias_point_t;

/* Each mask bit selects a logical DC channel (A..F). A zero pilot mask
 * disconnects that tone. Board-specific routing is resolved by the caller. */
typedef struct {
    uint8_t dc_mask;
    uint8_t pilot_mask[CTRL_PILOTS];
} ctrl_routes_t;

typedef struct {
    ctrl_routes_t routes;
    float bias_v[CTRL_DC_CHANNELS];
    float rho_ema;
    bool locked, hold_assist_active, frozen;
} ctrl_command_t;

/* State belongs to the caller. start produces the initial sweep-seeded
 * command. update consumes the harmonic windows for the previous command,
 * indexed by pilot, and produces the next absolute bias and routing. */
typedef struct {
    modulator_type_t type;
    dsp_status_t (*start)(void *ctx, bias_point_t point, float custom_phase_rad,
                          const float sweep_bias_v[CTRL_DC_CHANNELS], ctrl_command_t *cmd);
    dsp_status_t (*update)(void *ctx, const harmonic_data_t pilot[CTRL_PILOTS],
                           ctrl_command_t *cmd);
} modulator_strategy_t;

bool ctrl_routes_valid(const ctrl_routes_t *routes);

#endif
