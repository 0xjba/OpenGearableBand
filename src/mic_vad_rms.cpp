#include "mic_vad.h"
#include <math.h>

float mic_vad_block_rms(const int16_t *s, size_t n)
{
    if (n == 0) return 0.0f;
    double acc = 0.0;
    for (size_t i = 0; i < n; i++) { double v = (double)s[i]; acc += v * v; }
    return (float)sqrt(acc / (double)n);
}
