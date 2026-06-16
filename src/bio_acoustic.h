/*
 * bio_acoustic -- tap-capture / FFT / spectral-feature subsystem
 *
 * Owns the chip FIFO capture pipeline, the CMSIS-DSP FFT, band-energy
 * feature extraction, and the surface-classification heuristic.  Runs a
 * background worker thread; exposes a small init/on_tap/query interface.
 *
 * Architecture reference: gesture-architecture.md §3 Bio-acoustic sensing
 * path (Stage E).  The gesture FSM (gesture_mode.cpp) drives on_tap from the
 * INT1 dispatcher and queries last_was_hard_surface for diagnostic logging.
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Initialise the bio-acoustic pipeline.  Configures the CMSIS-DSP RFFT
 * instance and enables the LSM6DSL FIFO in continuous mode at 1.66 kHz.
 * Call once from main() at boot, AFTER the tap engine is enabled so the
 * FIFO ODR override supersedes the tap engine's 416 Hz default.
 */
void bio_acoustic_init(void);

/*
 * Notify the bio-acoustic pipeline that a chip tap event fired.  Reads
 * the FIFO into the static capture buffer and signals the worker thread
 * to extract features and classify.  If the worker is still processing
 * a previous tap the new one is dropped with a log line.
 *
 * Called from the INT1 dispatcher in main.cpp on every chip tap event.
 */
void bio_acoustic_on_tap(void);

/*
 * Query whether the most recently processed tap shows hard-surface
 * spectral content (mid_band_energy >= SURFACE_RESONANCE_MID_BAND_THRESH).
 * Used by gesture_mode.cpp for diagnostic logging of the SURFACE pose path.
 * Returns false until at least one tap has been processed.
 */
bool bio_acoustic_last_was_hard_surface(void);

#ifdef __cplusplus
}
#endif
