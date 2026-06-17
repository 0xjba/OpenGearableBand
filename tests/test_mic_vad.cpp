#include "mic_vad.h"
#include <cstdio>
#include <cmath>
#include <cstdint>

static int failures = 0;
#define CHECK(c) do { if(!(c)){ printf("FAIL line %d: %s\n", __LINE__, #c); failures++; } } while(0)

int main(void) {
    /* All-zero block -> RMS 0. */
    int16_t z[64] = {0};
    CHECK(mic_vad_block_rms(z, 64) == 0.0f);

    /* Constant +1000 -> RMS 1000. */
    int16_t c[100];
    for (int i = 0; i < 100; i++) c[i] = 1000;
    CHECK(fabsf(mic_vad_block_rms(c, 100) - 1000.0f) < 1e-3f);

    /* Full-scale square wave +/-10000 -> RMS 10000. */
    int16_t sq[100];
    for (int i = 0; i < 100; i++) sq[i] = (i % 2) ? 10000 : -10000;
    CHECK(fabsf(mic_vad_block_rms(sq, 100) - 10000.0f) < 1e-3f);

    /* Band-sum: sum of bins [lo,hi] inclusive, clamped to [0,n). */
    float e[8] = {0,1,2,3,4,5,6,7};
    CHECK(mic_vad_band_sum(e, 8, 2, 4) == 9.0f);      /* 2+3+4 */
    CHECK(mic_vad_band_sum(e, 8, 0, 100) == 28.0f);   /* hi clamped to 7 */
    CHECK(mic_vad_band_sum(e, 8, 5, 1) == 0.0f);      /* lo>hi -> 0 */
    CHECK(mic_vad_band_sum(e, 0, 0, 3) == 0.0f);      /* empty */
    CHECK(mic_vad_band_sum(e, 8, -2, 3) == 6.0f);     /* lo clamped to 0: 0+1+2+3 */

    printf(failures ? "FAILURES: %d\n" : "ALL PASS\n", failures);
    return failures ? 1 : 0;
}
