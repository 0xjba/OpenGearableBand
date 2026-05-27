#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/hrs.h>
#include <math.h>
#include <zephyr/sys/atomic.h>

#include "WearableDSP.h"
#include "power_ctrl.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// Device Pointers
// User asked to map MAX30102 to external I2C (i2c0).
const struct device *max30102_dev = DEVICE_DT_GET(DT_NODELABEL(max30102));

// The Xiao BLE Sense built-in IMU is an LSM6DS3 on the internal I2C bus. 
// Its node label is typically 'lsm6ds3tr_c' in the standard Zephyr board tree.
const struct device *imu_dev = DEVICE_DT_GET(DT_NODELABEL(lsm6ds3tr_c));

// Global Buffers
float ppg_buffer[BUFFER_SIZE];
float imu_buffer[BUFFER_SIZE];
volatile int sample_index = 0;

// DSP processing buffers (decoupled so the acquisition thread can keep
// writing the next window while the DSP thread is still crunching).
float dsp_ppg_buffer[BUFFER_SIZE];
float dsp_imu_buffer[BUFFER_SIZE];

// DSP Instance
WearableDSP dsp;

// Acquisition tick: 10 ms period (100 Hz). Binary semaphore so a missed
// deadline doesn't queue catch-up reads — overruns are detected separately.
K_SEM_DEFINE(acq_tick_sem, 0, 1);
K_SEM_DEFINE(dsp_process_sem, 0, 1);

// SIGN_MOT wake signal from the LSM6DSL's embedded Significant Motion
// engine.  The ISR cannot do real work; it only gives this semaphore,
// which the power state machine (step 6) blocks on.  Binary because
// rapid back-to-back fires (which shouldn't happen given how sig-mot
// is designed) should not queue up wake events.
K_SEM_DEFINE(motion_wake_sem, 0, 1);

// Acquisition-active gate.  Set true inside start_acquisition(), cleared
// in stop_acquisition().  acq_thread refuses to process samples unless
// this is true -- defends against spurious sem fires of unknown origin
// (observed: a Sample 0 line appearing at boot ~49 ms in, between the
// MAX30102 shutdown and LSM6DSL enable, with all-zero accel values and
// no apparent caller of k_sem_give(acq_tick_sem)).  Also gives us a
// clean per-burst sample counter for accurate Hz reporting.
static volatile bool acq_active = false;
static volatile uint32_t acq_burst_samples = 0;   // total ticks since burst start (incl. discards)
static volatile uint32_t acq_burst_stored = 0;    // stored samples since burst start

// Number of samples to discard at the start of every acquisition burst.
// The MAX30102 produces a ~16 000-count linear ramp over the first ~5
// FIFO entries after coming out of SHDN (LED warm-up + ADC settle +
// possibly stale FIFO contents).  That ramp is a giant step input to
// the 4th-order band-pass IIR and biases autocorrelation peak-finding
// for many samples after.  Discarding 10 samples (100 ms) is well past
// the observed end of the ramp and adds invisible latency to the user.
#define ACQ_WARMUP_SAMPLES                10

// Cross-thread observation of the DSP's per-window state.  The DSP
// thread bumps latest_window_seq each cycle and writes the latest
// motion and wear classifications; the power-state thread polls them.
// atomic_t so reads/writes are torn-read-safe across threads.
//
// latest_wear_state exists specifically so the power state machine can
// notice if the band is removed mid-WORKOUT and bail out -- without it,
// the user removing the band while exercising left WORKOUT permanently
// glued (motion classifier returns stale data when wear gate rejects,
// and the wear gate's "0 BPM" output doesn't propagate to the state
// machine).
static atomic_t latest_window_seq = ATOMIC_INIT(0);
static atomic_t latest_motion_state = ATOMIC_INIT(STATIONARY);
static atomic_t latest_wear_state = ATOMIC_INIT(WEAR_NOT_WORN);

// LSM6DSL INT1 pin spec, pulled from the board DTS (lsm6ds3tr_c.irq-gpios
// = <&gpio0 11 GPIO_ACTIVE_HIGH>).  Using GPIO_DT_SPEC_GET means a board
// rev that moved the pin would not silently miscompile.
static const struct gpio_dt_spec lsm6dsl_int1 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(lsm6ds3tr_c), irq_gpios);
static struct gpio_callback lsm6dsl_int1_cb_data;

static void lsm6dsl_int1_isr(const struct device *, struct gpio_callback *, uint32_t)
{
    /* Edge-triggered: just signal the state machine and get out.  Any
     * actual work (reading FUNC_SRC1, deciding state, etc.) belongs in
     * the consuming thread, not in the ISR context.
     */
    k_sem_give(&motion_wake_sem);
}

static int motion_wake_init(void)
{
    if (!gpio_is_ready_dt(&lsm6dsl_int1)) {
        LOG_ERR("LSM6DSL INT1 pin not ready");
        return -ENODEV;
    }
    int err = gpio_pin_configure_dt(&lsm6dsl_int1, GPIO_INPUT);
    if (err) {
        LOG_ERR("INT1 configure failed (%d)", err);
        return err;
    }
    err = gpio_pin_interrupt_configure_dt(&lsm6dsl_int1, GPIO_INT_EDGE_TO_ACTIVE);
    if (err) {
        LOG_ERR("INT1 edge-interrupt configure failed (%d)", err);
        return err;
    }
    gpio_init_callback(&lsm6dsl_int1_cb_data, lsm6dsl_int1_isr, BIT(lsm6dsl_int1.pin));
    err = gpio_add_callback(lsm6dsl_int1.port, &lsm6dsl_int1_cb_data);
    if (err) {
        LOG_ERR("INT1 add_callback failed (%d)", err);
        return err;
    }
    LOG_INF("LSM6DSL INT1 (P0.%d) callback installed", lsm6dsl_int1.pin);
    return 0;
}

static volatile uint32_t acq_overruns = 0;
static int64_t window_start_ms = 0;

static void acq_timer_handler(struct k_timer *) {
    if (k_sem_count_get(&acq_tick_sem) > 0) {
        acq_overruns++;
    }
    k_sem_give(&acq_tick_sem);
}
K_TIMER_DEFINE(acq_timer, acq_timer_handler, NULL);

// Bluetooth Advertisement Data
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                  BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void bt_ready(int err) {
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }
    LOG_INF("Bluetooth initialized");

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }
    LOG_INF("Advertising successfully started");
}

// --- 100Hz Data Acquisition Thread --- //
// Driven by k_timer (acq_timer_handler) firing every 10 ms.  The thread
// blocks on acq_tick_sem and never sleeps for a relative duration, so I2C
// latency cannot push the sampling cadence below the chip's 100 Hz rate.
void acq_thread_entry(void *, void *, void *) {
    LOG_INF("Data acquisition thread started");
    window_start_ms = k_uptime_get();

    while (1) {
        k_sem_take(&acq_tick_sem, K_FOREVER);

        // Drop any tick that arrives while acquisition isn't supposed to
        // be running (boot, IDLE between snapshots, transitions).
        if (!acq_active) {
            continue;
        }

        if (sample_index >= BUFFER_SIZE) {
            continue;
        }

        struct sensor_value red_val = {0};
        struct sensor_value ir_val = {0};
        struct sensor_value accel_x = {0};
        struct sensor_value accel_y = {0};
        struct sensor_value accel_z = {0};

        if (sensor_sample_fetch(max30102_dev) != 0) {
            LOG_ERR("MAX30102 fetch failed");
            continue;
        }
        sensor_channel_get(max30102_dev, SENSOR_CHAN_RED, &red_val);
        sensor_channel_get(max30102_dev, SENSOR_CHAN_IR, &ir_val);

        if (sensor_sample_fetch(imu_dev) != 0) {
            LOG_ERR("IMU fetch failed");
            continue;
        }
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel_x);
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

        // Reference for NLMS = Signal Magnitude Vector (SMV) of all three
        // accelerometer axes.  Wrist gestures / typing are dominated by
        // X- and Y-axis motion (flexion + rotation), so a Z-only reference
        // misses most of the kinetic energy that's actually corrupting the
        // PPG.  Gravity contributes a ~1 g DC offset to SMV; the DSP layer
        // subtracts the per-window mean before NLMS, so only the AC kinetic
        // component reaches the filter.
        float ax = (float)sensor_value_to_double(&accel_x);
        float ay = (float)sensor_value_to_double(&accel_y);
        float az = (float)sensor_value_to_double(&accel_z);
        float smv = sqrtf(ax * ax + ay * ay + az * az);

        // HARDWARE FIX: discard the first ACQ_WARMUP_SAMPLES of every
        // burst to flush the MAX30102 wake-from-SHDN artifact (see the
        // comment on ACQ_WARMUP_SAMPLES).  Fetches above ran regardless
        // so the chip's FIFO continues to drain on schedule; only the
        // store + downstream pipeline is gated.
        acq_burst_samples++;
        if (acq_burst_samples <= ACQ_WARMUP_SAMPLES) {
            continue;
        }

        // First STORED sample of the burst marks the true start of
        // the first measurement window.  Resetting window_start_ms
        // here means the reported Hz reflects the chip's actual
        // sample rate, not the wall-clock that included the warmup
        // discards.
        if (acq_burst_stored == 0) {
            window_start_ms = k_uptime_get();
        }

        ppg_buffer[sample_index] = (float)sensor_value_to_double(&ir_val);
        imu_buffer[sample_index] = smv;

        // Log the first 5 stored samples of each acquisition burst --
        // enough to confirm sensors warmed up cleanly without flooding
        // the serial console on every snapshot.
        if (acq_burst_stored < 5) {
            LOG_INF("Sample %u: PPG(IR)=%.1f, accel=(%.2f, %.2f, %.2f) SMV=%.3f",
                    acq_burst_stored, (double)ppg_buffer[sample_index],
                    (double)ax, (double)ay, (double)az, (double)smv);
        }

        sample_index++;
        acq_burst_stored++;

        if (sample_index >= BUFFER_SIZE) {
            int64_t now = k_uptime_get();
            int64_t elapsed_ms = now - window_start_ms;
            // First window of a burst processes BUFFER_SIZE stored
            // samples; subsequent overlapped windows process
            // BUFFER_SIZE/2 new samples on top of the carried half.
            uint32_t samples_in_window =
                (acq_burst_stored == BUFFER_SIZE) ? BUFFER_SIZE
                                                  : (BUFFER_SIZE / 2);
            float measured_hz =
                (elapsed_ms > 0)
                    ? ((float)samples_in_window * 1000.0f / (float)elapsed_ms)
                    : 0.0f;
            LOG_INF("Window full: %u samples in %lld ms => %.2f Hz "
                    "(overruns=%u)",
                    samples_in_window, elapsed_ms,
                    (double)measured_hz, acq_overruns);

            memcpy(dsp_ppg_buffer, ppg_buffer, sizeof(float) * BUFFER_SIZE);
            memcpy(dsp_imu_buffer, imu_buffer, sizeof(float) * BUFFER_SIZE);

            // 50 % overlap: keep the second half as the start of the next window.
            int overlap = BUFFER_SIZE / 2;
            memmove(ppg_buffer, &ppg_buffer[overlap], sizeof(float) * overlap);
            memmove(imu_buffer, &imu_buffer[overlap], sizeof(float) * overlap);
            sample_index = overlap;

            window_start_ms = now;
            k_sem_give(&dsp_process_sem);
        }
    }
}

// Define the high-priority acquisition thread
K_THREAD_DEFINE(acq_thread_id, 2048, acq_thread_entry, NULL, NULL, NULL, 5, 0, 0);

// --- Background DSP Thread --- //
static const char *wear_state_str(WearState s) {
    switch (s) {
        case WEAR_NOT_WORN:    return "NOT_WORN";
        case WEAR_STABILIZING: return "STABILIZING";
        case WEAR_WORN:        return "WORN";
        default:               return "?";
    }
}

void dsp_thread_entry(void *, void *, void *) {
    LOG_INF("DSP thread started");
    while (1) {
        k_sem_take(&dsp_process_sem, K_FOREVER);

        float final_bpm = dsp.processHeartRate(dsp_ppg_buffer, dsp_imu_buffer);
        WearState ws = dsp.getWearState();
        MotionState ms = dsp.getMotionState();

        LOG_INF("Wear=%s  BPM=%.2f", wear_state_str(ws), (double)final_bpm);

        // Publish motion + wear state + window sequence for the power
        // state machine to observe.  Sequence increments unconditionally
        // so a poller can detect "new window" without missing any.
        atomic_set(&latest_motion_state, (atomic_val_t)ms);
        atomic_set(&latest_wear_state, (atomic_val_t)ws);
        atomic_inc(&latest_window_seq);

        // Only push HRS notifications when the device is actually worn.
        // During NOT_WORN / STABILIZING the BPM is meaningless, and pushing
        // 0 would just confuse any connected client.
        if (ws == WEAR_WORN) {
            bt_hrs_notify((uint16_t)final_bpm);
        }
    }
}

// Define the processing thread (Low priority so it doesn't block the timer)
K_THREAD_DEFINE(dsp_thread_id, 4096, dsp_thread_entry, NULL, NULL, NULL, 7, 0, 0);

// --- Serial reset thread ---
// Polls the console UART (USB CDC ACM on the Xiao Sense) for an 'r' or 'R'
// character and triggers a full chip reboot via sys_reboot().  Useful when
// you can't easily double-tap the reset button (e.g. wrist-strapped, or
// you want to restart cleanly without re-entering the bootloader).
static const struct device *console_uart =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

void reset_thread_entry(void *, void *, void *) {
    if (!device_is_ready(console_uart)) {
        LOG_WRN("Console UART not ready; serial-reset disabled");
        return;
    }
    LOG_INF("Serial-reset armed: press 'r' on the console to reboot");

    uint8_t c;
    while (1) {
        if (uart_poll_in(console_uart, &c) == 0) {
            if (c == 'r' || c == 'R') {
                LOG_INF("Serial reset requested -- rebooting");
                k_sleep(K_MSEC(50));  // let the log line drain over USB
                sys_reboot(SYS_REBOOT_COLD);
                // not reached
            }
        } else {
            k_sleep(K_MSEC(50));
        }
    }
}

K_THREAD_DEFINE(reset_thread_id, 1024, reset_thread_entry, NULL, NULL, NULL, 10, 0, 0);

// ============================================================================
//  Power state machine (steps 6 + 7)
// ============================================================================
//
//   IDLE          MAX30102 shutdown, acq_timer stopped, LSM6DSL sig-motion
//                 engine armed, 5-minute snapshot k_timer running, BLE
//                 advertising at default cadence.  CPU is in WFI between
//                 BLE events and timer ticks (CONFIG_PM=y).
//
//   SNAPSHOT      Triggered by the 5-min snapshot timer.  Wakes MAX30102,
//                 starts the 10ms acq_timer, runs the existing DSP for
//                 SNAPSHOT_DURATION_MS, then back to IDLE.  15s window is
//                 long enough for the wear state machine (3 windows =
//                 ~7.7 s) plus a few WORN windows for Kalman to converge.
//
//   WORKOUT       Triggered by SIGN_MOT INT1 (sustained walking/running
//                 detected by the LSM6DSL embedded engine).  Same acq +
//                 DSP setup as SNAPSHOT, but it stays on -- transitioning
//                 back to IDLE only after WORKOUT_EXIT_COOLDOWN_WINDOWS
//                 consecutive non-HEAVY (STATIONARY or MICRO_MOTION)
//                 windows from the DSP.  We do a time-gated verification
//                 first (WORKOUT_VERIFY_MS) so a brief gesture or a
//                 single staircase doesn't drop the system into
//                 continuous mode for nothing.
//
// Transition events are delivered via semaphores, so the state-machine
// thread runs at the kernel's lowest priority and spends almost all of
// its time blocked (= CPU asleep).

enum PowerState {
    PS_IDLE,
    PS_SNAPSHOT,
    PS_WORKOUT_VERIFY,
    PS_WORKOUT,
};

static volatile PowerState power_state = PS_IDLE;
static const char *power_state_str(PowerState s) {
    switch (s) {
        case PS_IDLE:           return "IDLE";
        case PS_SNAPSHOT:       return "SNAPSHOT";
        case PS_WORKOUT_VERIFY: return "WORKOUT_VERIFY";
        case PS_WORKOUT:        return "WORKOUT";
        default:                return "?";
    }
}

// Tunables.  Comments cite the reasoning so anyone (including future me)
// can revisit without re-deriving from scratch.
//
// TODO(power-v2): SNAPSHOT_INTERVAL_MS = 2 min is the *testing* value so
// we don't have to wait 5 minutes between observations during bring-up.
// Bump back to (5 * 60 * 1000) before shipping.  Battery math: at 5 min
// the duty cycle is 15 / 300 = 5%, at 2 min it's 15 / 120 = 12.5% --
// noticeable but not catastrophic.
#define SNAPSHOT_INTERVAL_MS              (2 * 60 * 1000)  // TESTING: 2 min
#define SNAPSHOT_FIRST_BOOT_MS            (30 * 1000)      // 30s for first IDLE after boot
// Snapshot duration budget at 22 s:
//   * 7.7 s consumed by wear-state stabilization (3 windows x 2.56 s)
//   * remaining 14.3 s yields ~5 WORN measurement windows (Kalman gets
//     enough samples to behave as a low-pass average rather than just
//     tracking the most recent reading).
// At 15 s we only got 2 measurements -- the BPM was close to truth but
// noisier than necessary.  22 s is a better noise/duty-cycle trade.
#define SNAPSHOT_DURATION_MS              (22 * 1000)
#define WORKOUT_VERIFY_MS                 (3 * 60 * 1000)  // 3 min motion confirmation
#define WORKOUT_VERIFY_MOTION_RATIO_PCT   70               // >=70% of windows in motion
// Cooldown for WORKOUT exit: 60 windows of NOT HEAVY motion (= STATIONARY or
// MICRO).  Empirically (v0.2 jogging trace), requiring 117 consecutive pure
// STATIONARY windows was unreachable in real life -- typing, shifting, or
// reaching for things produced MICRO bursts that reset the streak forever,
// effectively trapping WORKOUT mode on (observed: still in WORKOUT after
// 15 min of clear post-jog rest).  Treating MICRO as "non-heavy" exits
// WORKOUT during low-intensity activity too -- a slow leisurely walk will
// drop back to SNAPSHOT cadence, which is the right product call: the
// 5-min snapshot still tracks the user, the continuous 100 Hz path is
// reserved for activities where second-to-second HR really changes (jog,
// HIIT, cycling).  Only sustained HEAVY (imu_var > 0.5) resets the streak.
#define WORKOUT_EXIT_COOLDOWN_WINDOWS     60               // ~2.5 min at 2.56s/window
#define WORKOUT_NOT_WORN_EXIT_WINDOWS     2                // ~5s, exit if band removed

// Snapshot cadence.  The k_timer just gives a semaphore; the state
// machine consumes it.  Started in transition_to_idle(), stopped on the
// way out of IDLE.
K_SEM_DEFINE(snapshot_tick_sem, 0, 1);
static void snapshot_tick_handler(struct k_timer *) {
    k_sem_give(&snapshot_tick_sem);
}
K_TIMER_DEFINE(snapshot_tick, snapshot_tick_handler, NULL);

// latest_window_seq and latest_motion_state are declared at file scope
// above (near the other shared state) so dsp_thread_entry can publish
// to them before the power-state thread block is even defined.

// Acquisition helpers ---------------------------------------------------
// We toggle the existing acq_timer rather than the threads -- threads
// stay alive and just block on their semaphores when the timer's stopped.

static void start_acquisition(void) {
    sample_index = 0;
    acq_overruns = 0;
    acq_burst_samples = 0;
    acq_burst_stored = 0;
    // window_start_ms will be reset by acq_thread when the first
    // post-warmup stored sample lands; this initial value just
    // protects against an early "buffer full" check pulling an
    // uninitialised delta.
    window_start_ms = k_uptime_get();
    // Set the gate BEFORE starting the timer so the very first tick
    // arrives at acq_thread with acq_active already true.
    acq_active = true;
    k_timer_start(&acq_timer, K_MSEC(10), K_MSEC(10));
}

static void stop_acquisition(void) {
    // Clear the gate BEFORE stopping the timer so any tick already in
    // flight on the way to acq_thread will be dropped at the gate check
    // rather than processed in some weird half-state.
    acq_active = false;
    k_timer_stop(&acq_timer);
    // Drain any pending tick semaphore so the next start_acquisition()
    // doesn't process a stale tick before the timer's first new fire.
    k_sem_reset(&acq_tick_sem);
}

// State transitions -----------------------------------------------------
// Centralised so every state entry runs through the same audit log line
// and so the order of "configure sensors" -> "set state" -> "start
// timers" is consistent.

static void transition_to_idle(void) {
    LOG_INF("PowerState: %s -> IDLE", power_state_str(power_state));
    stop_acquisition();
    max30102_shutdown();
    lsm6dsl_enable_sign_motion();       // re-arm (clears + enables)
    k_sem_reset(&motion_wake_sem);      // drop any stale edge

    // First IDLE after boot: schedule the first snapshot soon (30 s) so
    // the user gets a visible sign of life rather than waiting a full
    // SNAPSHOT_INTERVAL_MS.  Subsequent IDLE entries (after a snapshot
    // or workout) use the normal cadence -- by then there have been
    // recent fresh readings already.
    static bool first_idle_entry = true;
    k_timeout_t initial_delay = first_idle_entry
        ? K_MSEC(SNAPSHOT_FIRST_BOOT_MS)
        : K_MSEC(SNAPSHOT_INTERVAL_MS);
    first_idle_entry = false;

    k_timer_start(&snapshot_tick, initial_delay,
                  K_MSEC(SNAPSHOT_INTERVAL_MS));
    power_state = PS_IDLE;
}

static void transition_to_snapshot(void) {
    LOG_INF("PowerState: %s -> SNAPSHOT", power_state_str(power_state));
    k_timer_stop(&snapshot_tick);
    max30102_wake();
    start_acquisition();
    power_state = PS_SNAPSHOT;
}

static void transition_to_workout_verify(void) {
    LOG_INF("PowerState: %s -> WORKOUT_VERIFY", power_state_str(power_state));
    k_timer_stop(&snapshot_tick);
    lsm6dsl_disable_sign_motion();      // we're tracking motion ourselves now
    max30102_wake();
    start_acquisition();
    power_state = PS_WORKOUT_VERIFY;
}

static void transition_to_workout(void) {
    LOG_INF("PowerState: %s -> WORKOUT", power_state_str(power_state));
    // Acquisition is already running from VERIFY -- nothing to start.
    power_state = PS_WORKOUT;
}

// Per-state run loops --------------------------------------------------

static void run_idle(void) {
    // Wait on either the snapshot timer or a motion wake event.  k_poll
    // lets us block on N semaphores at once; the first one signalled
    // wins.
    struct k_poll_event events[2] = {
        K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
                                 K_POLL_MODE_NOTIFY_ONLY,
                                 &snapshot_tick_sem),
        K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
                                 K_POLL_MODE_NOTIFY_ONLY,
                                 &motion_wake_sem),
    };
    k_poll(events, ARRAY_SIZE(events), K_FOREVER);

    // Motion takes priority over a snapshot tick that fired at the same
    // moment -- we'd rather start tracking the workout than burn the
    // 15s on a snapshot first.
    if (events[1].state == K_POLL_STATE_SEM_AVAILABLE) {
        k_sem_take(&motion_wake_sem, K_NO_WAIT);
        k_sem_reset(&snapshot_tick_sem);  // ignore the tick we didn't service
        transition_to_workout_verify();
        return;
    }
    if (events[0].state == K_POLL_STATE_SEM_AVAILABLE) {
        k_sem_take(&snapshot_tick_sem, K_NO_WAIT);
        transition_to_snapshot();
        return;
    }
}

static void run_snapshot(void) {
    // Just wait for the snapshot duration to expire.  DSP results are
    // pushed over BLE by the existing dsp_thread.
    k_sleep(K_MSEC(SNAPSHOT_DURATION_MS));
    transition_to_idle();
}

static void run_workout_verify(void) {
    // Poll the DSP's per-window motion classification across the verify
    // window.  If the user is really exercising, the DSP will mostly
    // see MICRO_MOTION or HEAVY_MOTION; a brief gesture will settle
    // back to STATIONARY quickly.  Threshold: >=70% of observed
    // windows must be motion-class to commit to WORKOUT.
    //
    // Wear gate: if the user removed the band (or never had it on -- the
    // band might be sitting on a desk while being shaken), the DSP
    // reports NOT_WORN.  Bail out immediately rather than rack up a
    // false-positive verify.
    int64_t deadline = k_uptime_get() + WORKOUT_VERIFY_MS;
    uint32_t seq_last = atomic_get(&latest_window_seq);
    uint32_t windows = 0, motion_windows = 0;
    uint32_t not_worn_streak = 0;

    while (k_uptime_get() < deadline) {
        k_sleep(K_MSEC(500));
        uint32_t seq_now = atomic_get(&latest_window_seq);
        if (seq_now == seq_last) continue;
        seq_last = seq_now;

        WearState w = (WearState)atomic_get(&latest_wear_state);
        if (w == WEAR_NOT_WORN) {
            if (++not_worn_streak >= WORKOUT_NOT_WORN_EXIT_WINDOWS) {
                LOG_INF("VERIFY: NOT_WORN for %u windows -> abort to IDLE",
                        not_worn_streak);
                transition_to_idle();
                return;
            }
            continue;  // don't count motion stats while the band's off
        }
        not_worn_streak = 0;

        MotionState m = (MotionState)atomic_get(&latest_motion_state);
        windows++;
        if (m != STATIONARY) motion_windows++;
    }

    // Need a minimum sample size to avoid a single window dictating
    // the verdict.  Verify period is ~3 min, expected ~70 windows;
    // demanding at least 30 windows guards against edge cases (DSP
    // not running, wear-state stuck in STABILIZING, etc.).
    bool commit = (windows >= 30) &&
                  ((motion_windows * 100u) >= (windows * WORKOUT_VERIFY_MOTION_RATIO_PCT));

    LOG_INF("VERIFY: %u/%u motion windows (%u%%); commit=%d",
            motion_windows, windows,
            windows ? (motion_windows * 100u / windows) : 0, commit);

    if (commit) {
        transition_to_workout();
    } else {
        transition_to_idle();
    }
}

static void run_workout(void) {
    // Two exit conditions:
    //   1. The band came off (NOT_WORN for >= WORKOUT_NOT_WORN_EXIT_WINDOWS).
    //      Snap to IDLE -- a band on the desk should not stay in WORKOUT.
    //   2. Cooldown: WORKOUT_EXIT_COOLDOWN_WINDOWS consecutive non-HEAVY
    //      windows.  MICRO_MOTION counts toward the streak; only sustained
    //      HEAVY resets it.  This is an explicit product call: continuous
    //      100 Hz mode is reserved for high-variance activities (jog,
    //      cycling, HIIT) where HR really changes second-to-second.  A
    //      slow walk legitimately drops out -- the SNAPSHOT cadence at
    //      5 min still tracks the user with negligible duty cycle.
    uint32_t cooldown_streak = 0;
    uint32_t not_worn_streak = 0;
    uint32_t seq_last = atomic_get(&latest_window_seq);

    while (1) {
        k_sleep(K_MSEC(500));
        uint32_t seq_now = atomic_get(&latest_window_seq);
        if (seq_now == seq_last) continue;
        seq_last = seq_now;

        WearState w = (WearState)atomic_get(&latest_wear_state);
        if (w == WEAR_NOT_WORN) {
            if (++not_worn_streak >= WORKOUT_NOT_WORN_EXIT_WINDOWS) {
                LOG_INF("WORKOUT: NOT_WORN for %u windows -> exit to IDLE",
                        not_worn_streak);
                transition_to_idle();
                return;
            }
            cooldown_streak = 0;  // don't credit non-worn windows
            continue;
        }
        not_worn_streak = 0;

        MotionState m = (MotionState)atomic_get(&latest_motion_state);
        if (m != HEAVY_MOTION) {
            if (++cooldown_streak >= WORKOUT_EXIT_COOLDOWN_WINDOWS) {
                LOG_INF("WORKOUT: %u non-heavy windows -> exit to IDLE",
                        cooldown_streak);
                transition_to_idle();
                return;
            }
        } else {
            cooldown_streak = 0;
        }
    }
}

// State machine thread -------------------------------------------------

void power_state_thread_entry(void *, void *, void *) {
    LOG_INF("Power state machine starting");
    // Initial state: IDLE (sensors off, snapshot timer running).
    transition_to_idle();

    while (1) {
        switch (power_state) {
            case PS_IDLE:           run_idle();           break;
            case PS_SNAPSHOT:       run_snapshot();       break;
            case PS_WORKOUT_VERIFY: run_workout_verify(); break;
            case PS_WORKOUT:        run_workout();        break;
        }
    }
}
K_THREAD_DEFINE(power_state_thread_id, 2048, power_state_thread_entry,
                NULL, NULL, NULL, 8, 0, 0);

static void init_xiao_pins() {
    // Battery divider and high-current-charge pins.  The IMU power pin
    // (P1.08) is NOT touched here: the Xiao Sense board DTS declares a
    // regulator-fixed-sync on that pin with regulator-boot-on, so the
    // regulator framework drives it.  Configuring it as a raw GPIO output
    // races the regulator and can leave the LSM6DSL un-powered when
    // sensor_sample_fetch runs.
    const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

    if (device_is_ready(gpio0)) {
        // VBAT_ENABLE_PIN = P0.14 (cleared = enabled)
        gpio_pin_configure(gpio0, 14, GPIO_OUTPUT_INACTIVE);
        // HICHG_PIN = P0.13 (cleared = 100 mA mode)
        gpio_pin_configure(gpio0, 13, GPIO_OUTPUT_INACTIVE);
        LOG_INF("Xiao BLE Sense power pins configured.");
    } else {
        LOG_ERR("GPIO0 device not ready.");
    }
}

int main(void) {
    LOG_INF("Starting Wearable HR Monitor");
    
    init_xiao_pins();

    // The IMU regulator (regulator-boot-on) has already powered the LSM6DSL
    // during SYS_INIT.  No manual delay needed here.

    if (!device_is_ready(max30102_dev)) {
        LOG_ERR("MAX30102 not ready!");
    }

    if (!device_is_ready(imu_dev)) {
        LOG_ERR("IMU not ready!");
    }

    int err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
    }

    // GPIO callback for SIGN_MOT INT1.  Just installing the callback
    // here; the actual sig-motion engine enable, and the acq_timer
    // start, are owned by the power state machine (it transitions
    // into IDLE on boot, which configures both).
    motion_wake_init();

    // From this point on, the power state machine thread (started
    // automatically by K_THREAD_DEFINE) drives all sensor power and
    // acquisition timing.  main() has no more work to do.

    while (1) {
        // Main thread can handle background tasks such as battery reading
        // (SAADC on P0.31) and updating BAS.
        k_sleep(K_MSEC(1000));
    }
    return 0;
}
