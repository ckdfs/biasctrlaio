#include "check.h"
#include "ctrl_modulator.h"

static dsp_status_t start(void *ctx, bias_point_t target, float custom,
                          const float seed[CTRL_DC_CHANNELS], ctrl_command_t *cmd)
{
    (void)ctx;
    CHECK(target == BIAS_POINT_CUSTOM && custom == 1);
    cmd->routes = (ctrl_routes_t){0x3f, {0x1b, 0x36}};
    for (unsigned i = 0; i < CTRL_DC_CHANNELS; ++i)
        cmd->bias_v[i] = seed[i];
    return DSP_OK;
}

static dsp_status_t update(void *ctx, const harmonic_data_t pilot[CTRL_PILOTS], ctrl_command_t *cmd)
{
    (void)ctx;
    cmd->bias_v[5] += pilot[1].h1_magnitude - pilot[0].h1_magnitude;
    return DSP_OK;
}

int main(void)
{
    const modulator_strategy_t strategy = {MOD_TYPE_DPQPSK, start, update};
    ctrl_command_t cmd = {0};
    float seed[CTRL_DC_CHANNELS] = {1,2,3,4,5,6};
    harmonic_data_t pilot[CTRL_PILOTS] = {{.h1_magnitude = 2}, {.h1_magnitude = 5}};
    CHECK(strategy.start(NULL, BIAS_POINT_CUSTOM, 1, seed, &cmd) == DSP_OK);
    CHECK(ctrl_routes_valid(&cmd.routes));
    CHECK(strategy.update(NULL, pilot, &cmd) == DSP_OK && cmd.bias_v[5] == 9);
    cmd.routes.dc_mask = 0x1f;
    CHECK(!ctrl_routes_valid(&cmd.routes));
    cmd.routes.dc_mask = 0xff;
    CHECK(!ctrl_routes_valid(&cmd.routes));
    return 0;
}
