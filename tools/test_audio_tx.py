#!/usr/bin/env python3
"""Unit tests for the audio_tx clock-recovery control law (pure, no BLE)."""
import audio_tx as tx


def test_clamps_to_pm_1_5_percent():
    # Huge positive error (ring nearly full) -> clamp at +1.5% (send slower).
    integ, scale = tx.clock_recovery_step(0.0, used=16000, capacity=16384,
                                          setpoint_bytes=3840, ev_overflow=0, ev_underrun=0)
    assert scale <= 1.015 + 1e-9
    # Huge negative error (ring empty) -> clamp at -1.5% (send faster).
    integ, scale = tx.clock_recovery_step(0.0, used=0, capacity=16384,
                                          setpoint_bytes=3840, ev_overflow=0, ev_underrun=0)
    assert scale >= 0.985 - 1e-9


def test_cancels_0_8pct_drift_without_overflow():
    # Simulate: the I2S consumer drains 0.8% SLOWER than the sender's nominal rate
    # (the DAC reads slower than we push -> the ring fills). The loop must converge to
    # sending ~0.8% slower so the ring level stabilises near setpoint.
    cap, setp = 16384, 3840
    used = float(setp)
    integ = 0.0
    rate = 16000 * 2  # nominal bytes/s the sender pushes at scale=1.0
    drain = rate * (1.0 - 0.008)  # DAC drains 0.8% slower than send rate -> ring grows
    dt = 0.1
    max_used = used
    for _ in range(3000):  # 300 s simulated
        integ, scale = tx.clock_recovery_step(integ, int(used), cap, setp, 0, 0)
        produced = rate / scale * dt   # slower pace (scale>1) -> fewer bytes pushed
        used += produced - drain * dt
        used = max(0.0, min(used, cap))
        max_used = max(max_used, used)
    assert max_used < cap - 256, f"ring hit the rail: max_used={max_used}"
    assert abs(used - setp) < cap * 0.05, f"did not settle near setpoint: used={used}"


def test_flags_bias_direction():
    _, scale_of = tx.clock_recovery_step(0.0, used=3840, capacity=16384,
                                         setpoint_bytes=3840, ev_overflow=1, ev_underrun=0)
    assert scale_of > 1.0   # overflow -> send slower
    _, scale_uf = tx.clock_recovery_step(0.0, used=3840, capacity=16384,
                                         setpoint_bytes=3840, ev_overflow=0, ev_underrun=1)
    assert scale_uf < 1.0   # underrun -> send faster


def test_reset_zeroes_integral():
    assert tx.clock_recovery_reset() == 0.0


def test_reset_then_step_starts_clean():
    # The contract: on barge-in flush / restart the host resets the controller. A
    # wound-up integral after reset must behave exactly like a fresh controller.
    integ_fresh, scale_fresh = tx.clock_recovery_step(0.0, 5000, 16384, 3840, 0, 0)
    integ_reset, scale_reset = tx.clock_recovery_step(
        tx.clock_recovery_reset(), 5000, 16384, 3840, 0, 0)
    assert integ_reset == integ_fresh
    assert scale_reset == scale_fresh


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS {name}")
    print("all passed")
