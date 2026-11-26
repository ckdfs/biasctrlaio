#include "ctrl_modulator.h"

bool ctrl_routes_valid(const ctrl_routes_t *routes)
{
    const unsigned dc_bits = (1u << CTRL_DC_CHANNELS) - 1u;
    if (!routes || (routes->dc_mask & ~dc_bits))
        return false;
    for (unsigned i = 0; i < CTRL_PILOTS; ++i)
        if (routes->pilot_mask[i] & ~routes->dc_mask)
            return false;
    return true;
}
