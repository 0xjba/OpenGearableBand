#include "cursor_track.h"
#include "gesture_thresholds.h"
#include <cstdio>
#include <cmath>

static int failures = 0;
#define CHECK(cond) do { if(!(cond)){ printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while(0)

static int drain_slam(float vert, float roll)
{
    float dx, dy;
    int n = 0;
    for (; n < CURSOR_SLAM_MAX_REPORTS + 2; n++) {
        if (!cursor_track_is_slamming()) break;
        cursor_track_update(vert, roll, false, 0.0f, &dx, &dy);
    }
    return n;
}

int main(void)
{
    float dx, dy;
    const float V = CURSOR_ROLL_SHADOW_REVALIDATE + 1.0f;
    /* Fixed anchor: vert_top == CURSOR_VERT_TOP_DEG (12).  vert_bottom =
     * min(12+70, 85) = 82, so max_counts = GAIN_Y * 70 (no BOTTOM_MAX clamp). */
    const float TOP  = CURSOR_VERT_TOP_DEG;
    const float MAXC = CURSOR_GAIN_Y * CURSOR_VERT_SPAN_DEG;

    /* M1: after entry slam drains, cur_y == 0; holding vert==top yields dy 0. */
    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    CHECK(fabsf(cursor_track_cur_y()) < 1e-3f);
    cursor_track_update(TOP, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);

    /* M2: servo toward GAIN_Y*(vert-top); vert=top+10 -> 300 = 127+127+46. */
    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    cursor_track_update(TOP + 10.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy - 127.0f) < 1e-3f);
    cursor_track_update(TOP + 10.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy - 127.0f) < 1e-3f);
    cursor_track_update(TOP + 10.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy - 46.0f) < 1e-3f);
    cursor_track_update(TOP + 10.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);
    CHECK(fabsf(cursor_track_cur_y() - 300.0f) < 1e-3f);

    /* M3: target clamps to max_counts at the bottom (vert 90 >> 82). */
    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    for (int i = 0; i < 30; i++) cursor_track_update(90.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(cursor_track_cur_y() - MAXC) < 1e-3f);

    /* M4: post-stop update is a no-op. */
    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    cursor_track_stop();
    dx = 1.0f; dy = 1.0f;
    cursor_track_update(40.0f, 80.0f, false, V, &dx, &dy);
    CHECK(dx == 0.0f && dy == 0.0f);

    /* M5: once clamped at the bottom, further-down vert holds (dy 0). */
    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    for (int i = 0; i < 30; i++) cursor_track_update(90.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(cursor_track_cur_y() - MAXC) < 1e-3f);
    cursor_track_update(95.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);

    /* --- Slam sizing: floor untuned-safe, cap respected. --- */
    cursor_track_set_gain(8.0f, 1.0f);
    cursor_track_start(TOP, 50.0f);
    CHECK(cursor_track_is_slamming());
    int floor_reports = (int)ceilf(CURSOR_SLAM_FLOOR_COUNTS / 127.0f);
    int used = drain_slam(TOP, 50.0f);
    CHECK(used >= floor_reports);
    CHECK(used <= CURSOR_SLAM_MAX_REPORTS);
    cursor_track_set_gain(8.0f, 200.0f);
    cursor_track_start(TOP, 50.0f);
    int used3 = drain_slam(TOP, 50.0f);
    CHECK(used3 == CURSOR_SLAM_MAX_REPORTS);
    cursor_track_set_gain(CURSOR_GAIN_X, CURSOR_GAIN_Y);

    /* --- Fixed anchor: entry vert NEVER moves vert_top (replaces calibration). */
    cursor_track_start(40.0f, 50.0f);
    CHECK(fabsf(cursor_track_vert_top() - CURSOR_VERT_TOP_DEG) < 1e-3f);
    cursor_track_start(14.0f, 50.0f);
    CHECK(fabsf(cursor_track_vert_top() - CURSOR_VERT_TOP_DEG) < 1e-3f);

    /* --- Top re-pin hysteresis (anchored at the fixed top). --- */
    cursor_track_set_gain(CURSOR_GAIN_X, CURSOR_GAIN_Y);
    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    CHECK(fabsf(cursor_track_vert_top() - CURSOR_VERT_TOP_DEG) < 1e-3f);
    for (int i = 0; i < 25; i++) cursor_track_update(40.0f, 50.0f, false, V, &dx, &dy);
    CHECK(cursor_track_cur_y() > 100.0f);
    cursor_track_update(TOP, 50.0f, false, V, &dx, &dy);
    CHECK(cursor_track_is_slamming());
    drain_slam(TOP, 50.0f);
    CHECK(fabsf(cursor_track_cur_y()) < 1e-3f);

    /* R2: no chatter while hovering in [ENTER,LEAVE]. */
    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    cursor_track_update(TOP, 50.0f, false, V, &dx, &dy);
    cursor_track_update(TOP + (CURSOR_PIN_ENTER_DEG + CURSOR_PIN_LEAVE_DEG) * 0.5f,
                        50.0f, false, V, &dx, &dy);
    CHECK(!cursor_track_is_slamming());

    /* R3: leave past LEAVE then return re-arms exactly once; R3b no double-fire. */
    cursor_track_update(TOP + CURSOR_PIN_LEAVE_DEG + 5.0f, 50.0f, false, V, &dx, &dy);
    CHECK(!cursor_track_is_slamming());
    cursor_track_update(TOP, 50.0f, false, V, &dx, &dy);
    CHECK(cursor_track_is_slamming());
    drain_slam(TOP, 50.0f);
    cursor_track_update(TOP, 50.0f, false, V, &dx, &dy);
    CHECK(!cursor_track_is_slamming());

    /* --- X = linear yaw delta (was roll); Y stillness via servo. --- */
    cursor_track_set_gain(CURSOR_GAIN_X, CURSOR_GAIN_Y);
    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    cursor_track_update(TOP, 50.0f, false, V, &dx, &dy);
    cursor_track_update(TOP, 53.0f, false, V, &dx, &dy);
    CHECK(fabsf(dx - CURSOR_GAIN_X * 3.0f) < 1e-3f);
    cursor_track_update(TOP, 56.0f, false, CURSOR_ROLL_SHADOW_INVALIDATE - 1.0f, &dx, &dy);
    CHECK(dx == 0.0f);

    cursor_track_start(TOP, 50.0f);
    drain_slam(TOP, 50.0f);
    for (int i = 0; i < 20; i++) cursor_track_update(TOP + 10.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);
    cursor_track_update(TOP + 10.0f, 50.0f, true, V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);
    cursor_track_update(TOP + 15.0f, 50.0f, true, V, &dx, &dy);
    CHECK(dy > 0.0f);

    /* --- Runtime anchors: set_anchors changes top + bottom; span derives. --- */
    cursor_track_set_gain(CURSOR_GAIN_X, CURSOR_GAIN_Y);
    cursor_track_set_anchors(20.0f, 80.0f);
    CHECK(fabsf(cursor_track_vert_top() - 20.0f) < 1e-3f);
    CHECK(fabsf(cursor_track_vert_bottom() - 80.0f) < 1e-3f);
    /* New top=20: holding vert=20 after slam yields dy 0; vert=30 servos to GAIN_Y*10. */
    cursor_track_start(20.0f, 50.0f);
    drain_slam(20.0f, 50.0f);
    cursor_track_update(20.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);
    for (int i = 0; i < 20; i++) cursor_track_update(30.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(cursor_track_cur_y() - CURSOR_GAIN_Y * 10.0f) < 1e-3f);

    /* --- Bottom still clamps to CURSOR_VERT_BOTTOM_MAX when set beyond it. --- */
    cursor_track_set_anchors(20.0f, 200.0f);
    CHECK(fabsf(cursor_track_vert_bottom() - CURSOR_VERT_BOTTOM_MAX) < 1e-3f);

    /* Restore defaults for hygiene (nothing runs after, but be explicit). */
    cursor_track_set_anchors(CURSOR_VERT_TOP_DEG, CURSOR_VERT_TOP_DEG + CURSOR_VERT_SPAN_DEG);

    /* ===== X-axis (yaw) contract ===== */
    cursor_track_set_gain(CURSOR_GAIN_X, CURSOR_GAIN_Y);

    /* A: arc-linearity + speed-independence -- a 20deg sweep moves GAIN_X*20 px
     * whether fed as one big step or many small ones. */
    cursor_track_start(TOP, 0.0f);
    drain_slam(TOP, 0.0f);
    cursor_track_update(TOP, 0.0f, false, V, &dx, &dy);   /* sync prev at yaw 0 */
    float x_big = 0.0f;
    cursor_track_update(TOP, 20.0f, false, V, &dx, &dy);
    x_big += dx;
    cursor_track_start(TOP, 0.0f);
    drain_slam(TOP, 0.0f);
    cursor_track_update(TOP, 0.0f, false, V, &dx, &dy);
    float x_small = 0.0f;
    for (int i = 1; i <= 40; i++) {
        cursor_track_update(TOP, i * 0.5f, false, V, &dx, &dy);
        x_small += dx;
    }
    CHECK(fabsf(x_big   - CURSOR_GAIN_X * 20.0f) < 1e-3f);
    CHECK(fabsf(x_small - CURSOR_GAIN_X * 20.0f) < 1e-3f);
    CHECK(fabsf(x_big - x_small) < 1e-3f);

    /* B: at-rest yaw re-zero (a big discontinuity) is absorbed -> dx 0, and the
     * next real move is uncontaminated (prev resynced across the re-zero). */
    cursor_track_start(TOP, 100.0f);
    drain_slam(TOP, 100.0f);
    cursor_track_update(TOP, 100.0f, false, V, &dx, &dy);  /* sync prev at 100 */
    cursor_track_update(TOP, 0.0f, true, V, &dx, &dy);      /* re-zero: -100 jump */
    CHECK(dx == 0.0f);
    cursor_track_update(TOP, 2.0f, false, V, &dx, &dy);     /* 0->2, not 100->2 */
    CHECK(fabsf(dx - CURSOR_GAIN_X * 2.0f) < 1e-3f);

    /* C: freeze at rest for sub-release motion; releases for a real move. */
    cursor_track_start(TOP, 0.0f);
    drain_slam(TOP, 0.0f);
    cursor_track_update(TOP, 0.0f, false, V, &dx, &dy);
    cursor_track_update(TOP, 0.02f, true, V, &dx, &dy);     /* 0.02 < FREEZE_RELEASE */
    CHECK(dx == 0.0f);
    cursor_track_update(TOP, 1.02f, true, V, &dx, &dy);      /* +1.0 > FREEZE_RELEASE */
    CHECK(fabsf(dx - CURSOR_GAIN_X * 1.0f) < 1e-3f);

    /* D: yaw wrap across +/-180 -> small real delta, no ~360 spike. */
    cursor_track_start(TOP, 179.0f);
    drain_slam(TOP, 179.0f);
    cursor_track_update(TOP, 179.0f, false, V, &dx, &dy);
    cursor_track_update(TOP, -179.0f, false, V, &dx, &dy);  /* wraps to +2 deg */
    CHECK(fabsf(dx - CURSOR_GAIN_X * 2.0f) < 1e-3f);

    /* E: X suppressed during a Y slam. */
    cursor_track_start(TOP, 0.0f);
    CHECK(cursor_track_is_slamming());
    cursor_track_update(TOP, 10.0f, false, V, &dx, &dy);
    CHECK(dx == 0.0f);
    drain_slam(TOP, 10.0f);

    /* F: cone-gate hysteresis -- off below INVALIDATE, holds off in the band,
     * back on above REVALIDATE. */
    cursor_track_start(TOP, 0.0f);
    drain_slam(TOP, 0.0f);
    cursor_track_update(TOP, 0.0f, false, V, &dx, &dy);
    cursor_track_update(TOP, 2.0f, false, CURSOR_ROLL_SHADOW_INVALIDATE - 0.5f, &dx, &dy);
    CHECK(dx == 0.0f);
    cursor_track_update(TOP, 4.0f, false,
        (CURSOR_ROLL_SHADOW_INVALIDATE + CURSOR_ROLL_SHADOW_REVALIDATE) * 0.5f, &dx, &dy);
    CHECK(dx == 0.0f);
    cursor_track_update(TOP, 6.0f, false, CURSOR_ROLL_SHADOW_REVALIDATE + 0.5f, &dx, &dy);
    CHECK(dx != 0.0f);
    /* F2 (hold-high leg): from VALID, the mid-band keeps X ON; only dropping
     * below INVALIDATE turns it off. Covers the asymmetric else-if branch. */
    cursor_track_update(TOP, 8.0f, false,
        (CURSOR_ROLL_SHADOW_INVALIDATE + CURSOR_ROLL_SHADOW_REVALIDATE) * 0.5f, &dx, &dy);
    CHECK(dx != 0.0f);
    cursor_track_update(TOP, 10.0f, false, CURSOR_ROLL_SHADOW_INVALIDATE - 0.5f, &dx, &dy);
    CHECK(dx == 0.0f);

    /* ===== Sweep-coupling compensation (X-arc fix) =====
     * Helper inline: sweep to yaw=D (sweep_deg == D) while holding a fixed vert,
     * then settle the Y servo, and read the servo target. */
    cursor_track_set_gain(CURSOR_GAIN_X, CURSOR_GAIN_Y);

    /* SC1: k=0 is identity -- a swept cursor still targets GAIN_Y*(vert-top). */
    cursor_track_set_swing_comp(0.0f);
    cursor_track_start(TOP, 0.0f);
    drain_slam(TOP, 0.0f);
    cursor_track_update(TOP + 20.0f, 0.0f, false, V, &dx, &dy);          /* sync, sweep 0 */
    for (int i = 1; i <= 30; i++) cursor_track_update(TOP + 20.0f, (float)i, false, V, &dx, &dy);
    for (int i = 0; i < 30; i++)  cursor_track_update(TOP + 20.0f, 30.0f, false, V, &dx, &dy);
    CHECK(fabsf(cursor_track_target_counts() - CURSOR_GAIN_Y * 20.0f) < 1e-3f);

    /* SC2: parabola subtraction -- k=0.01, sweep_deg=30 -> correction 0.01*900=9deg,
     * so target = GAIN_Y*(20-9) = GAIN_Y*11. */
    cursor_track_set_swing_comp(0.01f);
    cursor_track_start(TOP, 0.0f);
    drain_slam(TOP, 0.0f);
    cursor_track_update(TOP + 20.0f, 0.0f, false, V, &dx, &dy);
    for (int i = 1; i <= 30; i++) cursor_track_update(TOP + 20.0f, (float)i, false, V, &dx, &dy);
    for (int i = 0; i < 30; i++)  cursor_track_update(TOP + 20.0f, 30.0f, false, V, &dx, &dy);
    CHECK(fabsf(cursor_track_target_counts() - CURSOR_GAIN_Y * 11.0f) < 1e-3f);

    /* SC3: center is unaffected -- at sweep 0, k>0 still applies no correction. */
    cursor_track_set_swing_comp(0.01f);
    cursor_track_start(TOP, 0.0f);
    drain_slam(TOP, 0.0f);
    for (int i = 0; i < 30; i++) cursor_track_update(TOP + 20.0f, 0.0f, false, V, &dx, &dy);
    CHECK(fabsf(cursor_track_target_counts() - CURSOR_GAIN_Y * 20.0f) < 1e-3f);

    /* SC4: deliberate vertical survives -- at a fixed nonzero sweep, raising vert
     * beyond the parabola still raises the Y target (intent preserved). */
    cursor_track_set_swing_comp(0.01f);
    cursor_track_start(TOP, 0.0f);
    drain_slam(TOP, 0.0f);
    cursor_track_update(TOP + 20.0f, 0.0f, false, V, &dx, &dy);
    for (int i = 1; i <= 30; i++) cursor_track_update(TOP + 20.0f, (float)i, false, V, &dx, &dy);
    for (int i = 0; i < 30; i++)  cursor_track_update(TOP + 20.0f, 30.0f, false, V, &dx, &dy);
    float t_low = cursor_track_target_counts();
    for (int i = 0; i < 30; i++)  cursor_track_update(TOP + 40.0f, 30.0f, false, V, &dx, &dy);
    float t_high = cursor_track_target_counts();
    CHECK(t_high > t_low + 100.0f);

    cursor_track_set_swing_comp(CURSOR_SWING_COMP_K);   /* restore default for hygiene */

    printf(failures ? "FAILURES: %d\n" : "ALL PASS\n", failures);
    return failures ? 1 : 0;
}
