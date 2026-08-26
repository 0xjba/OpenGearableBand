/*
 * M3c.2 stub for gesture_mode: audio_stream gates on gesture_mode_get() ==
 * MODE_DICTATION. Without the full gesture FSM here, this lets the test app
 * force the mode on/off (serial `d`) so the LC3 uplink can be exercised.
 */
#include "gesture_mode.h"

static volatile GestureMode forced_mode = MODE_IDLE;

extern "C" GestureMode gesture_mode_get(void)
{
	return forced_mode;
}

/* Test-only setter (not part of the real gesture_mode API). */
extern "C" void gesture_mode_stub_set(GestureMode m)
{
	forced_mode = m;
}
