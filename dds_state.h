#ifndef DDS_STATE_H
#define DDS_STATE_H

/* dds_state.h — shared DDS parameters, globals, and safe update function
 *
 * Include this header in both main.c and i2c_slave.c.
 * Single source of truth for:
 *   - SAMPLE_RATE_HZ, frequency/amplitude limits, ramp rate
 *   - extern declarations for the shared volatile globals
 *   - calc_phase_inc(), calc_amp_scale(), dds_update()              */

#include <stdint.h>
#include <stdbool.h>
#include "dds_tables.h"

/* ── Timing and frequency limits ─────────────────────────────────────────────
 * SAMPLE_RATE_HZ was previously duplicated as the magic number 327973UL in
 * both main.c and i2c_slave.c. Now both include this header and share one
 * definition — changing it here updates everything automatically.            */
#define SAMPLE_RATE_HZ  100000UL    /* Hz  — must divide evenly into 1,000,000 */
#define FREQ_MIN_HZ     500UL       /* Hz  — minimum commanded frequency        */
#define FREQ_MAX_HZ     10000UL     /* Hz  — maximum commanded frequency        */
#define DAC_IOUT_MAX_UA 2000U       /* µA  — full-scale DAC output current      */

/* ── Amplitude ramp rate enforcement ────────────────────────────────────────
 * Hardware / safety requirement: current must ramp at ≤ 0.1 mA/sec,
 * both when starting and stopping stimulation.
 *
 * The ramp is implemented in the Core 1 timer ISR by stepping the applied
 * amplitude scale by 1 DAC count every AMP_RAMP_INTERVAL samples.
 *
 * Derivation:
 *   Full scale   = DAC_IOUT_MAX_UA = 2000 µA  →  scale 65536
 *   Max ramp     = 100 µA/sec
 *   µA per step  = DAC_IOUT_MAX_UA / 65536 = 0.03052 µA/count
 *   Steps/sec    = 100 µA/sec ÷ 0.03052 µA/count = 3276.8 steps/sec
 *   Interval     = SAMPLE_RATE_HZ / 3276.8 = 30.52 samples/step
 *   → step 1 count every 31 samples = 96.8 µA/sec  ✓ (within limit)        */
#define AMP_RAMP_INTERVAL  31U      /* samples between 1-count amplitude steps */

/* ── Shared volatile globals — defined in main.c ────────────────────────────
 * g_phase_inc      and g_wave_type are written atomically (single 32-bit
 *                  write on Cortex-M0+) so they take effect within one
 *                  sample with no risk of a torn read.
 *
 * g_amplitude_target  is the REQUESTED scale. The ISR ramps the APPLIED
 *                  scale toward this value at AMP_RAMP_INTERVAL to enforce
 *                  the ≤ 0.1 mA/sec limit. Core 0 never writes the applied
 *                  scale directly — only the target.
 *
 * g_output_enable  is no longer used to gate output. Enabling sets the
 *                  amplitude target to the requested value; disabling sets
 *                  the target to 0 so the ISR ramps down naturally.
 *                  The output is always running — it just ramps to silence. */
extern volatile uint32_t    g_phase_inc;
extern volatile wave_type_t g_wave_type;
extern volatile uint32_t    g_amplitude_target;  /* requested scale — ISR ramps to this */

/* ── Phase increment ─────────────────────────────────────────────────────────
 * phase_inc = freq_hz × 2^32 / SAMPLE_RATE_HZ
 * 64-bit intermediate prevents overflow up to ~1 MHz.                        */
static inline uint32_t calc_phase_inc(uint32_t freq_hz)
{
    return (uint32_t)(((uint64_t)freq_hz << 32) / SAMPLE_RATE_HZ);
}

/* ── Amplitude scale ─────────────────────────────────────────────────────────
 * Converts µA to a Q16 multiplier used by the ISR:
 *   output_sample = (lut[idx] * amplitude_scale) >> 16                       */
static inline uint32_t calc_amp_scale(uint16_t amp_uA)
{
    if (amp_uA >= DAC_IOUT_MAX_UA) return 65536U;
    return (uint32_t)((amp_uA * 65536ULL) / DAC_IOUT_MAX_UA);
}

/* ── DDS parameter update ────────────────────────────────────────────────────
 * Call from Core 0 (i2c_slave.c) whenever a new command arrives.
 *
 * Frequency and waveform are written directly — each is a single atomic
 * 32-bit write on the Cortex-M0+, so the ISR can never read a torn value.
 * A one-sample glitch at a mid-output waveform change is imperceptible.
 *
 * Amplitude is never written directly. Instead g_amplitude_target is updated
 * and the ISR ramps the applied scale toward it at ≤ 0.1 mA/sec. This
 * enforces the hardware safety requirement automatically:
 *   - enable=true  → target set to requested scale → ISR ramps UP
 *   - enable=false → target set to 0              → ISR ramps DOWN to silence
 *
 * No disable/barrier/re-enable sequence needed — continuous output is
 * maintained throughout all parameter changes.                               */
static inline void dds_update(uint32_t freq_hz, wave_type_t wave,
                               uint32_t amp_scale, bool enable)
{
    g_phase_inc        = calc_phase_inc(freq_hz);
    g_wave_type        = wave;
    g_amplitude_target = enable ? amp_scale : 0U;
}

#endif /* DDS_STATE_H */