#include "mic_vad.h"
#include <math.h>

float mic_vad_block_rms(const int16_t *s, size_t n)
{
    if (n == 0) return 0.0f;
    double acc = 0.0;
    for (size_t i = 0; i < n; i++) { double v = (double)s[i]; acc += v * v; }
    return (float)sqrt(acc / (double)n);
}

float mic_vad_band_sum(const float *e, size_t n, int lo, int hi)
{
    if (n == 0 || lo > hi) return 0.0f;
    if (lo < 0) lo = 0;
    if (hi >= (int)n) hi = (int)n - 1;
    float acc = 0.0f;
    for (int i = lo; i <= hi; i++) acc += e[i];
    return acc;
}
