#include "cursor_calib.h"
#include "gesture_thresholds.h"
#include <cstdio>
#include <cmath>

static int failures = 0;
#define CHECK(cond) do { if(!(cond)){ printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while(0)

int main(void)
{
    cursor_calib_result_t r;

    /* C1: cold-start + complete (top plausible, bottom valid, sweep>=25) -> SEED both. */
    r = cursor_calib_decide(false, 12.0f, 82.0f, 12.0f, /*bottom*/80.0f, /*valid*/true);
    CHECK(r.decision == CAL_SEED);
    CHECK(r.apply == true);
    CHECK(fabsf(r.new_top - 12.0f) < 1e-3f);
    CHECK(fabsf(r.new_bottom - 80.0f) < 1e-3f);

    /* C2: cold-start, no bottom captured -> run on defaults, apply nothing. */
    r = cursor_calib_decide(false, 12.0f, 82.0f, 12.0f, 0.0f, false);
    CHECK(r.decision == CAL_REJECT);
    CHECK(r.reason == CAL_REASON_COLD_START_DEFAULT);
    CHECK(r.apply == false);

    /* A1: have-calib, large shift -> ADOPT, blend both toward (20,60). */
    r = cursor_calib_decide(true, 12.0f, 82.0f, 20.0f, 60.0f, true);
    CHECK(r.decision == CAL_ADOPT);
    CHECK(r.apply == true);
    CHECK(fabsf(r.new_top    - (12.0f + CAL_BLEND_ALPHA * (20.0f - 12.0f))) < 1e-2f);
    CHECK(fabsf(r.new_bottom - (82.0f + CAL_BLEND_ALPHA * (60.0f - 82.0f))) < 1e-2f);

    /* A2: have-calib, both shifts < CAL_MIN_DELTA -> below-min-delta no-op. */
    r = cursor_calib_decide(true, 12.0f, 82.0f, 13.0f, 81.0f, true);
    CHECK(r.decision == CAL_REJECT);
    CHECK(r.reason == CAL_REASON_BELOW_MIN_DELTA);
    CHECK(r.apply == false);

    /* A3: have-calib, no bottom this session (mid-air) -> SHADOW-TRANSLATE, apply nothing. */
    r = cursor_calib_decide(true, 12.0f, 82.0f, 14.0f, 0.0f, false);
    CHECK(r.decision == CAL_SHADOW_TRANSLATE);
    CHECK(r.reason == CAL_REASON_MID_AIR_SHADOW);
    CHECK(r.apply == false);
    CHECK(fabsf(r.shadow_bottom - (14.0f + (82.0f - 12.0f))) < 1e-3f);  /* top + prior span */

    /* A4: implausible top (45 > CAL_TOP_MAX) even with a valid bottom -> REJECT implausible-top. */
    r = cursor_calib_decide(true, 12.0f, 82.0f, 45.0f, 80.0f, true);
    CHECK(r.decision == CAL_REJECT);
    CHECK(r.reason == CAL_REASON_IMPLAUSIBLE_TOP);
    CHECK(r.apply == false);

    /* A5: valid bottom but sweep too small (25 - 12 = 13 < CAL_SWEEP_MIN_DEG) -> insufficient-sweep. */
    r = cursor_calib_decide(true, 12.0f, 82.0f, 12.0f, 25.0f, true);
    CHECK(r.decision == CAL_REJECT);
    CHECK(r.reason == CAL_REASON_INSUFFICIENT_SWEEP);
    CHECK(r.apply == false);

    printf(failures ? "FAILURES: %d\n" : "ALL PASS\n", failures);
    return failures ? 1 : 0;
}
