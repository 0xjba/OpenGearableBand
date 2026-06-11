#include "cursor_track.h"
#include "gesture_thresholds.h"
#include <cstdio>
#include <cmath>

static int failures = 0;
#define CHECK(cond) do { if(!(cond)){ printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while(0)

/* Drive past any entry slam: call update until the slam burst is exhausted,
 * holding vert/roll fixed so only the slam contributes.  Returns ticks used. */
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
    const float V = CURSOR_ROLL_SHADOW_REVALIDATE + 1.0f; /* shadow: X valid */

    /* --- Map + servo (Task 2) --- */

    /* M1: after the entry slam drains, the cursor estimate is pinned to top
     * (cur_y == 0) and holding vert==vert_top yields no Y motion. */
    cursor_track_start(15.0f, 50.0f);   /* vert_top = 15 */
    drain_slam(15.0f, 50.0f);
    CHECK(fabsf(cursor_track_cur_y() - 0.0f) < 1e-3f);
    cursor_track_update(15.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);                       /* target 0, cur_y 0 -> err 0 */

    /* M2: servo drives toward target = GAIN_Y*(vert-vert_top), int8-clamped. */
    cursor_track_start(15.0f, 50.0f);
    CHECK(cursor_track_is_slamming());
    drain_slam(15.0f, 50.0f);
    /* vert 25 -> target = 30*(25-15)=300 counts; first tick clamps to +127. */
    cursor_track_update(25.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy - 127.0f) < 1e-3f);
    /* keep holding vert=25: converges (300 = 127+127+46). */
    cursor_track_update(25.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy - 127.0f) < 1e-3f);
    cursor_track_update(25.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy - 46.0f) < 1e-3f);
    cursor_track_update(25.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(dy) < 1e-3f);                       /* converged */
    CHECK(fabsf(cursor_track_cur_y() - 300.0f) < 1e-3f);

    /* M3: target clamps to max_counts at the comfort bottom.
     * vert_top=15, span=40 -> vert_bottom=min(55,60)=55; max=30*40=1200. */
    cursor_track_start(15.0f, 50.0f);
    CHECK(cursor_track_is_slamming());
    drain_slam(15.0f, 50.0f);
    for (int i = 0; i < 20; i++) cursor_track_update(90.0f, 50.0f, false, V, &dx, &dy);
    CHECK(fabsf(cursor_track_cur_y() - 1200.0f) < 1e-3f); /* clamped, not 30*(90-15) */

    /* M4: post-stop update is a no-op. */
    cursor_track_start(15.0f, 50.0f);
    drain_slam(15.0f, 50.0f);
    cursor_track_stop();
    dx = 1.0f; dy = 1.0f;
    cursor_track_update(40.0f, 80.0f, false, V, &dx, &dy);
    CHECK(dx == 0.0f && dy == 0.0f);

    /* --- Slam sizing (Task 3) --- */

    /* S1: entry arms a slam, and it lasts >= the floor's worth of reports
     * regardless of a tiny GAIN_Y (undershoot would poison registration). */
    cursor_track_set_gain(8.0f, 1.0f);          /* gain_y tiny -> max_counts small */
    cursor_track_start(15.0f, 50.0f);
    CHECK(cursor_track_is_slamming());
    int floor_reports = (int)ceilf(CURSOR_SLAM_FLOOR_COUNTS / 127.0f);
    int used = drain_slam(15.0f, 50.0f);
    CHECK(used >= floor_reports);               /* not shorter than the floor */
    CHECK(used <= CURSOR_SLAM_MAX_REPORTS);      /* and capped */

    /* S2: a large tuned range scales the slam up (margin*max_counts > floor),
     * still capped. */
    cursor_track_set_gain(8.0f, 60.0f);          /* max=60*40=2400; *2=4800<6000 floor */
    cursor_track_start(15.0f, 50.0f);
    int used2 = drain_slam(15.0f, 50.0f);
    CHECK(used2 >= floor_reports);
    cursor_track_set_gain(8.0f, 200.0f);         /* max=8000; *2=16000 -> capped */
    cursor_track_start(15.0f, 50.0f);
    int used3 = drain_slam(15.0f, 50.0f);
    CHECK(used3 == CURSOR_SLAM_MAX_REPORTS);

    /* restore default gain for any later tests */
    cursor_track_set_gain(CURSOR_GAIN_X, CURSOR_GAIN_Y);

    /* --- Entry calibration (Task 4) --- */

    /* C1: a normal raised entry captures vert_top live. */
    cursor_track_start(14.0f, 50.0f);
    CHECK(fabsf(cursor_track_vert_top() - 14.0f) < 1e-3f);

    /* C2: a lazy half-raise (vert above the max) is rejected -> falls back to
     * the last good capture (14 from C1), NOT the bad 35. */
    cursor_track_start(35.0f, 50.0f);
    CHECK(fabsf(cursor_track_vert_top() - 14.0f) < 1e-3f);

    /* C3: a fresh good capture updates the last-good. */
    cursor_track_start(17.0f, 50.0f);
    CHECK(fabsf(cursor_track_vert_top() - 17.0f) < 1e-3f);
    cursor_track_start(40.0f, 50.0f);            /* lazy again */
    CHECK(fabsf(cursor_track_vert_top() - 17.0f) < 1e-3f);  /* falls back to 17 */

    printf(failures ? "FAILURES: %d\n" : "ALL PASS\n", failures);
    return failures ? 1 : 0;
}
