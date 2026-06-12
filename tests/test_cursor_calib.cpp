#include "cursor_calib.h"
#include "gesture_thresholds.h"
#include <cstdio>
#include <cmath>

static int failures = 0;
#define CHECK(cond) do { if(!(cond)){ printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while(0)

/* Fill buf[0..n) with `val`. */
static void fill(float *buf, int n, float val) { for (int i=0;i<n;i++) buf[i]=val; }

/* Test-only visibility into the plateau extractor (defined in cursor_calib.cpp). */
typedef struct { bool found; float median; float var; int len; bool any_lowvar; } plateau_t;
extern "C" plateau_t cursor_calib_find_plateau_TEST(const float *v, int n, float top_now);

int main(void)
{
    float v[VERT_HIST_SAMPLES];

    /* T0 (stub): empty/zero history -> REJECT (no plateau). */
    fill(v, VERT_HIST_SAMPLES, 0.0f);
    cursor_calib_result_t r = cursor_calib_decide(false, 12.0f, 82.0f, 12.0f,
                                                  v, VERT_HIST_SAMPLES);
    CHECK(r.decision == CAL_REJECT);
    CHECK(r.apply == false);

    /* P1: a clean rest (flat ~80 for 1s) then a raise to ~12 -> plateau at 80. */
    fill(v, VERT_HIST_SAMPLES, 80.0f);                 /* whole buffer rest @80 */
    for (int i = VERT_HIST_SAMPLES - 50; i < VERT_HIST_SAMPLES; i++)
        v[i] = 80.0f - (float)(i - (VERT_HIST_SAMPLES - 50)); /* last 50: ramp down to ~30 */
    {
        plateau_t p = cursor_calib_find_plateau_TEST(v, VERT_HIST_SAMPLES, 12.0f);
        CHECK(p.found);
        CHECK(fabsf(p.median - 80.0f) < 1.5f);
        CHECK(p.len >= CAL_PLATEAU_DWELL);
    }

    /* P2: no rest (constant slow drift, never still) -> no plateau. */
    for (int i = 0; i < VERT_HIST_SAMPLES; i++) v[i] = 10.0f + 0.5f * i; /* var huge */
    {
        plateau_t p = cursor_calib_find_plateau_TEST(v, VERT_HIST_SAMPLES, 12.0f);
        CHECK(!p.found);
    }

    /* P3: a low-var rest but too close to top (rest @25, top 12, sweep 13 < 25)
     *     -> not a qualifying plateau, but any_lowvar flags it (insufficient sweep). */
    fill(v, VERT_HIST_SAMPLES, 25.0f);
    {
        plateau_t p = cursor_calib_find_plateau_TEST(v, VERT_HIST_SAMPLES, 12.0f);
        CHECK(!p.found);
        CHECK(p.any_lowvar);
    }

    /* P4 (slow-raise / invisible-starvation guard): rest plateau sits at the
     *    OLD edge of the buffer, followed by a long slow climb that fills the rest.
     *    Plateau must still be found (buffer is deep enough at 300). */
    fill(v, VERT_HIST_SAMPLES, 78.0f);                 /* default fill */
    for (int i = 0; i < 40; i++) v[i] = 78.0f;         /* oldest 40 = rest @78 */
    for (int i = 40; i < VERT_HIST_SAMPLES; i++)
        v[i] = 78.0f - 0.25f * (float)(i - 40);        /* slow ~0.25 deg/sample climb */
    {
        plateau_t p = cursor_calib_find_plateau_TEST(v, VERT_HIST_SAMPLES, 12.0f);
        CHECK(p.found);
        CHECK(fabsf(p.median - 78.0f) < 2.0f);
    }

    /* P5: two rests (one at top dwell @14, one lowered @70); pick the HIGH-vert one. */
    fill(v, VERT_HIST_SAMPLES, 70.0f);                 /* lowered rest fills most */
    for (int i = VERT_HIST_SAMPLES - 40; i < VERT_HIST_SAMPLES; i++) v[i] = 14.0f; /* recent top dwell */
    {
        plateau_t p = cursor_calib_find_plateau_TEST(v, VERT_HIST_SAMPLES, 12.0f);
        CHECK(p.found);
        CHECK(fabsf(p.median - 70.0f) < 1.5f);         /* the lowered rest, NOT the 14 dwell */
    }

    printf(failures ? "FAILURES: %d\n" : "ALL PASS\n", failures);
    return failures ? 1 : 0;
}
