#include "cursor_track.h"
#include "gesture_thresholds.h"
#include <cstdio>
#include <cmath>

static int failures = 0;
#define CHECK(cond) do { if(!(cond)){ printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while(0)

int main(void)
{
    float dx, dy;
    const float V = CURSOR_ROLL_SHADOW_REVALIDATE + 1.0f; /* shadow: X valid */

    /* 1. Unchanged angles -> no movement. */
    cursor_track_start(-60.0f, 50.0f);
    cursor_track_update(-60.0f, 50.0f, false, V, &dx, &dy);
    CHECK(dx == 0.0f && dy == 0.0f);

    /* 2. Pitch->dy, roll->dx, scaled by gain (X valid). */
    cursor_track_start(-60.0f, 50.0f);
    cursor_track_update(-58.0f, 53.0f, false, V, &dx, &dy);  /* dpitch=+2 droll=+3 */
    CHECK(fabsf(dy - CURSOR_GAIN_Y * 2.0f) < 1e-3f);
    CHECK(fabsf(dx - CURSOR_GAIN_X * 3.0f) < 1e-3f);

    /* 3. Cone gate: shadow below INVALIDATE -> X gated, Y still moves. */
    cursor_track_start(-60.0f, 50.0f);
    cursor_track_update(-60.0f, 50.0f, false, V, &dx, &dy);                          /* valid=true */
    cursor_track_update(-58.0f, 55.0f, false, CURSOR_ROLL_SHADOW_INVALIDATE - 1.0f, &dx, &dy);
    CHECK(dx == 0.0f);
    CHECK(fabsf(dy - CURSOR_GAIN_Y * 2.0f) < 1e-3f);

    /* 4. Hysteresis: shadow between thresholds holds the prior state. */
    cursor_track_start(-60.0f, 50.0f);
    cursor_track_update(-60.0f, 50.0f, false, V, &dx, &dy);                          /* valid=true */
    cursor_track_update(-60.0f, 51.0f, false,
                        (CURSOR_ROLL_SHADOW_INVALIDATE + CURSOR_ROLL_SHADOW_REVALIDATE) * 0.5f,
                        &dx, &dy);                                                    /* between -> holds */
    CHECK(dx != 0.0f);

    /* 4b. Hysteresis (inverse): a previously-INVALID state is also held in
     * the band -- catches an accidental ">= INVALIDATE" revalidation bug. */
    cursor_track_start(-60.0f, 50.0f);
    cursor_track_update(-60.0f, 50.0f, false, CURSOR_ROLL_SHADOW_INVALIDATE - 1.0f, &dx, &dy); /* valid=false */
    cursor_track_update(-60.0f, 51.0f, false,
                        (CURSOR_ROLL_SHADOW_INVALIDATE + CURSOR_ROLL_SHADOW_REVALIDATE) * 0.5f,
                        &dx, &dy);                                                    /* between -> holds invalid */
    CHECK(dx == 0.0f);

    /* 4c. After stop(), update is a safe no-op until a fresh start. */
    cursor_track_stop();
    cursor_track_update(-60.0f, 80.0f, false, V, &dx, &dy);
    CHECK(dx == 0.0f && dy == 0.0f);

    /* 5. Freeze: at_rest + tiny delta -> no motion. */
    cursor_track_start(-60.0f, 50.0f);
    cursor_track_update(-60.0f, 50.0f, true, V, &dx, &dy);
    CHECK(dx == 0.0f && dy == 0.0f);

    /* 6. Freeze releases on a move above the release delta even if at_rest. */
    cursor_track_start(-60.0f, 50.0f);
    cursor_track_update(-59.0f, 50.0f, true, V, &dx, &dy);  /* dpitch=+1 > release */
    CHECK(fabsf(dy - CURSOR_GAIN_Y * 1.0f) < 1e-3f);

    /* 7a. Roll wrap at +/-180 is handled (not a teleport). */
    cursor_track_start(-60.0f, 179.0f);
    cursor_track_update(-60.0f, -179.0f, false, V, &dx, &dy);  /* raw -358 -> wrap +2 */
    CHECK(fabsf(dx - CURSOR_GAIN_X * 2.0f) < 1e-3f);

    /* 7b. A true glitch beyond MAX_DELTA is discarded. */
    cursor_track_start(0.0f, 0.0f);
    cursor_track_update(0.0f, CURSOR_MAX_DELTA_DEG + 10.0f, false, V, &dx, &dy);
    CHECK(dx == 0.0f);

    if (failures == 0) { printf("ALL PASS\n"); return 0; }
    printf("%d FAILURES\n", failures);
    return 1;
}
