#ifndef DSP_TYPES_H
#define DSP_TYPES_H

#include <stdint.h>

typedef enum {
    DSP_OK,
    DSP_INVALID_ARGUMENT,
    DSP_INCOHERENT,
    DSP_BLOCK_FULL,
    DSP_INCOMPLETE,
    DSP_SINGULAR
} dsp_status_t;

#define DSP_TAU 6.2831853071795864769
#define DSP_TDM_PILOT1_HZ 962.5f
#define DSP_TDM_PILOT2_HZ 1762.5f
#define DSP_TDM_BLOCK_SIZE 5120u

/* ========================================================================= */
/*  DSP system parameters                                                    */
/* ========================================================================= */

/** ADC sample rate in Hz (ADS131M02, OSR=128, HR mode, 8.192 MHz CLKIN) */
#define DSP_SAMPLE_RATE_HZ      64000

/** Pilot tone frequency in Hz */
#define DSP_PILOT_FREQ_HZ            1000

/** Samples per 1 kHz pilot cycle at 64 kSPS. Must stay integer for coherent detection. */
#define DSP_PILOT_PERIOD_SAMPLES     64

/** Number of full pilot cycles integrated by each Goertzel block.
 *  Using multiple cycles improves SNR for weak harmonic extraction while
 *  preserving integer-cycle coherence (no spectral leakage). */
#define DSP_GOERTZEL_BLOCK_CYCLES    20

/** Goertzel block size in samples.
 *  N = 64 samples/cycle * 20 cycles = 1280 samples = 20 ms @ 64 kSPS. */
#define DSP_GOERTZEL_BLOCK_SIZE      (DSP_PILOT_PERIOD_SAMPLES * DSP_GOERTZEL_BLOCK_CYCLES)

/** Number of Goertzel blocks per control update.
 *  With N=1280 and decimation=10, control rate is 64k / (1280 * 10) = 5 Hz. */
#define DSP_CONTROL_DECIMATION       10

/* ========================================================================= */
/*  Harmonic analysis result — shared between DSP and Control layers         */
/* ========================================================================= */

typedef struct {
    float h1_magnitude;  /**< 1st harmonic magnitude (at pilot freq f0) */
    float h2_magnitude;  /**< 2nd harmonic magnitude (at 2*f0) */
    float h1_phase;      /**< 1st harmonic phase relative to sample-zero cosine (radians) */
    float h2_phase;      /**< 2nd harmonic phase (radians) */
    float dc_power;      /**< DC component from CH1 (mean of block samples) */
} harmonic_data_t;

#endif /* DSP_TYPES_H */
