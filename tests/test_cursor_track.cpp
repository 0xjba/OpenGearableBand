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

    printf(failures ? "FAILURES: %d\n" : "ALL PASS\n", failures);
    return failures ? 1 : 0;
}
