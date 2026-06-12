#include "cursor_calib.h"
#include "gesture_thresholds.h"
#include <math.h>

/* Internal plateau result (kept in the .cpp; the test file re-declares a
 * layout-compatible struct for the TEST shim below). */
typedef struct { bool found; float median; float var; int len; bool any_lowvar; } plateau_t;

/* Median of a[0..n) via insertion sort on a local copy (n <= VERT_HIST_SAMPLES). */
static float median_of(const float *a, int n)
{
    if (n > VERT_HIST_SAMPLES) n = VERT_HIST_SAMPLES;
    if (n <= 0) return 0.0f;
    float tmp[VERT_HIST_SAMPLES];
    for (int i = 0; i < n; i++) tmp[i] = a[i];
    for (int i = 1; i < n; i++) {
        float k = tmp[i]; int j = i - 1;
        while (j >= 0 && tmp[j] > k) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = k;
    }
    return (n & 1) ? tmp[n / 2] : 0.5f * (tmp[n / 2 - 1] + tmp[n / 2]);
}

/* Scan chronological vert[] for the highest-median low-variance resting plateau.
 * Greedy O(n) segmentation: grow a run via incremental (Welford) variance; when a
 * sample would push the run variance >= CAL_PLATEAU_VAR, close the run and start a
 * new one. A run qualifies as the resting bottom if it is long enough AND lowered
 * enough (median >= top_now + CAL_SWEEP_MIN_DEG). any_lowvar records that *some*
 * qualifying-length still run existed (even if too close to top) so the caller can
 * distinguish "no rest at all" from "rest, but insufficient sweep". */
static plateau_t find_plateau(const float *v, int n, float top_now)
{
    plateau_t best = { false, 0.0f, 0.0f, 0, false };
    float best_median = -1.0f;

    int   run_start = 0;
    double mean = 0.0, M2 = 0.0;
    int    cnt = 0;

    for (int i = 0; i <= n; i++) {
        bool close_run = (i == n);

        if (!close_run) {
            int    newcnt  = cnt + 1;
            double d       = (double)v[i] - mean;
            double newmean = mean + d / newcnt;
            double newM2   = M2 + d * ((double)v[i] - newmean);
            double var     = (newcnt > 1) ? newM2 / newcnt : 0.0;
            /* The variance gate is the ONLY thing that breaks a run: a drift slow
             * enough to stay under CAL_PLATEAU_VAR is absorbed into the run. That's
             * fine -- median (not mean) keeps the anchor honest; reported `var` shows
             * the inflated spread to telemetry. */
            if (cnt > 0 && var >= CAL_PLATEAU_VAR) {
                close_run = true;            /* exclude v[i]; restart the run at i */
            } else {
                mean = newmean; M2 = newM2; cnt = newcnt;
            }
        }

        if (close_run) {
            if (cnt >= CAL_PLATEAU_DWELL) {
                float med = median_of(&v[run_start], cnt);
                float var = (float)(cnt > 1 ? M2 / cnt : 0.0);
                best.any_lowvar = true;      /* a long still run exists */
                if (med >= top_now + CAL_SWEEP_MIN_DEG && med > best_median) {
                    best_median = med;
                    best.found  = true;
                    best.median = med;
                    best.var    = var;
                    best.len    = cnt;
                }
            }
            run_start = i;
            if (i < n) { mean = v[i]; M2 = 0.0; cnt = 1; } else { cnt = 0; }
        }
    }
    return best;
}

/* Test-only shim: expose find_plateau to the host unit test. */
extern "C" plateau_t cursor_calib_find_plateau_TEST(const float *v, int n, float top_now)
{
    return find_plateau(v, n, top_now);
}

static float cal_blend(float old_v, float new_v)
{
    return old_v + CAL_BLEND_ALPHA * (new_v - old_v);
}

cursor_calib_result_t cursor_calib_decide(bool have_calib,
                                          float prior_top, float prior_bottom,
                                          float top_now,
                                          const float *vert_chrono, int n)
{
    cursor_calib_result_t r;
    r.decision = CAL_REJECT;
    r.reason   = CAL_REASON_NO_PLATEAU;
    r.apply    = false;
    r.new_top  = prior_top;
    r.new_bottom = prior_bottom;
    r.shadow_bottom = 0.0f;

    bool top_plausible = (top_now <= CAL_TOP_MAX);
    plateau_t p = find_plateau(vert_chrono, n, top_now);

    r.plateau_found    = p.found;
    r.plateau_var      = p.var;
    r.plateau_n        = p.len;
    r.bottom_candidate = p.found ? p.median : 0.0f;
    r.sweep_deg        = p.found ? (p.median - top_now) : 0.0f;

    bool ritual_complete = top_plausible && p.found;  /* sweep enforced inside find_plateau */

    /* ---- Cold-start: every boot starts here (RAM-only, no prior calibration). ---- */
    if (!have_calib) {
        if (ritual_complete) {
            r.decision    = CAL_SEED;
            r.reason      = CAL_REASON_OK;
            r.new_top     = top_now;
            r.new_bottom  = p.median;
            r.apply       = true;
        } else {
            r.decision = CAL_REJECT;
            r.reason   = CAL_REASON_COLD_START_DEFAULT;  /* run on compile-time defaults */
            r.apply    = false;
        }
        return r;
    }

    /* ---- have_calib branch: filled in Task 4. ---- */
    (void)prior_bottom; (void)cal_blend;
    r.decision = CAL_REJECT;
    r.reason   = CAL_REASON_NO_PLATEAU;
    r.apply    = false;
    return r;
}
