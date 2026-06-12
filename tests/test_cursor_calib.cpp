#include "cursor_calib.h"
#include "gesture_thresholds.h"
#include <cstdio>
#include <cmath>

static int failures = 0;
#define CHECK(cond) do { if(!(cond)){ printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while(0)

/* Fill buf[0..n) with `val`. */
static void fill(float *buf, int n, float val) { for (int i=0;i<n;i++) buf[i]=val; }

int main(void)
{
    float v[VERT_HIST_SAMPLES];

    /* T0 (stub): empty/zero history -> REJECT (no plateau). */
    fill(v, VERT_HIST_SAMPLES, 0.0f);
    cursor_calib_result_t r = cursor_calib_decide(false, 12.0f, 82.0f, 12.0f,
                                                  v, VERT_HIST_SAMPLES);
    CHECK(r.decision == CAL_REJECT);
    CHECK(r.apply == false);

    printf(failures ? "FAILURES: %d\n" : "ALL PASS\n", failures);
    return failures ? 1 : 0;
}
