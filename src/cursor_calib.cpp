#include "cursor_calib.h"
#include "gesture_thresholds.h"
#include <math.h>

cursor_calib_result_t cursor_calib_decide(bool have_calib,
                                          float prior_top, float prior_bottom,
                                          float top_now,
                                          const float *vert_chrono, int n)
{
    (void)have_calib; (void)top_now; (void)vert_chrono; (void)n;
    cursor_calib_result_t r;
    r.decision = CAL_REJECT;
    r.reason   = CAL_REASON_NO_PLATEAU;
    r.apply    = false;
    r.new_top  = prior_top;
    r.new_bottom = prior_bottom;
    r.plateau_found = false;
    r.plateau_var = 0.0f;
    r.plateau_n = 0;
    r.bottom_candidate = 0.0f;
    r.sweep_deg = 0.0f;
    r.shadow_bottom = 0.0f;
    return r;
}
