"""Clock-recovery PI controller behavior (pure, no hardware)."""
from voiceio import clock_recovery as cr


def test_at_setpoint_no_correction():
    sp = cr.setpoint_bytes(140)
    integ, scale = cr.step(0.0, sp, 8192, sp, 0, 0)
    assert scale == 1.0          # used == setpoint, no integral history -> neutral pace
    assert integ == 0.0


def test_too_full_sends_slower():
    sp = cr.setpoint_bytes(140)
    integ, scale = cr.step(0.0, sp + 3000, 8192, sp, 0, 0)
    assert scale > 1.0           # buffer over setpoint -> pace_scale > 1 (slow down)


def test_too_empty_sends_faster():
    sp = cr.setpoint_bytes(140)
    integ, scale = cr.step(0.0, sp - 3000, 8192, sp, 0, 0)
    assert scale < 1.0           # buffer under setpoint -> pace_scale < 1 (speed up)


def test_correction_is_clamped_to_authority():
    sp = cr.setpoint_bytes(140)
    # Wildly full buffer + repeated overflow kicks must not exceed +/-1.5%.
    integ = 0.0
    scale = 1.0
    for _ in range(500):
        integ, scale = cr.step(integ, 8192, 8192, sp, 1, 0)
    assert scale <= 1.0 + cr.CR_CLAMP + 1e-9
    assert integ <= cr.CR_INTEG_MAX + 1e-9    # anti-windup holds the integral bounded


def test_reset_returns_zero_integral():
    assert cr.reset() == 0.0
