"""Host-side downlink clock recovery (PI controller).

Ported verbatim (math-identical) from tools/audio_tx.py, where it was proven on
hardware to hold the device's playout jitter buffer at a setpoint drift-free over
60 s+ (see [[project_ble_downlink_2026_06_21]]). Pure + side-effect-free: the
integral is threaded through the step, not stored globally, so a barge-in just
resets it to 0.0.

The device reports (used, capacity, flags) over the status notify; this maps that
to a `pace_scale` that multiplies the downlink inter-block send interval:
  pace_scale > 1  -> send SLOWER (buffer too full, ring draining slow / overflow)
  pace_scale < 1  -> send FASTER (buffer too empty, underrunning)
ble_link applies pace_scale to its downlink pacing so we feed the device at its
TRUE consumption rate (the speaker's actual 15873 Hz), not our nominal 16 kHz.

[STRUCTURAL] Gains/setpoint re-tune if the firmware jitter-buffer sizing changes.
"""

# --- Setpoint: target buffered audio on the device (latency vs jitter cushion) ---
SETPOINT_MS = 140                       # HW-tuned 2026-06-21 (120 jitter-marginal, 160 robust)
SR_HZ = 16000                           # nominal host sample rate (for the byte setpoint)


def setpoint_bytes(setpoint_ms=SETPOINT_MS):
    """Target device-buffer fill in bytes (16 kHz mono 16-bit)."""
    return setpoint_ms * SR_HZ * 2 // 1000


# --- Control-law gains (slow loop -> no oscillation). Clamp must EXCEED worst-case
# I2S drift (~0.8%), so +/-1.5%. ---
CR_KP = 0.5             # proportional, on normalized error (used-setpoint)/capacity
CR_KI = 0.02            # integral (slow)
CR_CLAMP = 0.015        # +/-1.5% correction authority
CR_INTEG_MAX = CR_CLAMP / CR_KI         # anti-windup: integral alone can't exceed clamp
CR_FLAG_KICK = 0.10     # integral nudge on a hard over/underrun event

# Status flags byte bits (mirror firmware ble_audio.h BLE_AUDIO_STATUS_FL_*).
STATUS_FL_OVERFLOW = 0x02
STATUS_FL_UNDERRUN = 0x04


def reset():
    """Fresh controller integral (0.0). Call on barge-in flush / session restart."""
    return 0.0


def step(integ, used, capacity, setpoint, ev_overflow, ev_underrun):
    """Pure PI step. Returns (new_integ, pace_scale).

    Args:
        integ: prior integral (thread it back in next call).
        used: device-buffer bytes currently buffered (from status notify).
        capacity: device-buffer total bytes (from status notify; >=1).
        setpoint: target used-bytes (see setpoint_bytes()).
        ev_overflow / ev_underrun: 1 if the status flags reported the event.
    """
    err = (used - setpoint) / float(capacity)       # >0 = too full -> slow down
    integ += err
    if ev_overflow:
        integ += CR_FLAG_KICK       # too full -> bias slower
    if ev_underrun:
        integ -= CR_FLAG_KICK       # too empty -> bias faster
    integ = max(-CR_INTEG_MAX, min(CR_INTEG_MAX, integ))    # anti-windup
    corr = CR_KP * err + CR_KI * integ
    corr = max(-CR_CLAMP, min(CR_CLAMP, corr))
    return integ, 1.0 + corr
