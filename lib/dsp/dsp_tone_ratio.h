#ifndef DSP_TONE_RATIO_H
#define DSP_TONE_RATIO_H

#include "dsp_types.h"
#include <math.h>

/* Euclid's algorithm on the exact binary inputs, used only at setup.
 * Reducing first avoids rounding nearly coherent tones onto a DFT bin. */
static inline dsp_status_t dsp_tone_ratio(float freq, float rate, uint32_t *cycles,
                                         uint32_t *samples)
{
    if (!isfinite(freq) || !isfinite(rate) || freq <= 0 || 2.0 * freq >= rate)
        return DSP_INVALID_ARGUMENT;
    double a = rate, b = freq;
    while (b != 0) {
        double rem = fmod(a, b);
        a = b;
        b = rem;
    }
    double period = rate / a;
    if (period > UINT32_MAX) return DSP_INCOHERENT;
    *samples = (uint32_t)period;
    *cycles = (uint32_t)(freq / a);
    return DSP_OK;
}
#endif
