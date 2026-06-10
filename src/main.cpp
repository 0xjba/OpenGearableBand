#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/reboot.h>
#include <hal/nrf_power.h>  // NRF_POWER->GPREGRET for UF2 bootloader entry
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
#include "gesture_mode.h"
#include "ble_hid.h"
#include "cursor_pipeline.h"
#include <zephyr/settings/settings.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// Device Pointers
// User asked to map MAX30102 to external I2C (i2c0).
const struct device *max30102_dev = DEVICE_DT_GET(DT_NODELABEL(max30102));

// The Xiao BLE Sense built-in IMU is an LSM6DS3 on the internal I2C bus. 
// Its node label is typically 'lsm6ds3tr_c' in the standard Zephyr board tree.
const struct device *imu_dev = DEVICE_DT_GET(DT_NODELABEL(lsm6ds3tr_c));

// Global Buffers.
//   imu_smv_buffer carries the Signal Magnitude Vector for motion-state
//   variance classification (orientation-agnostic, single scalar).
//   imu_{x,y,z}_buffer carry the raw per-axis accelerations -- needed
//   as separate NLMS references because SMV's magnitude operation
//   frequency-doubles the AC content (a 2.5 Hz arm swing becomes 5 Hz
//   in SMV) which makes it useless for cancelling the original 2.5 Hz
//   PPG motion artifact.
float ppg_buffer[BUFFER_SIZE];
float imu_smv_buffer[BUFFER_SIZE];
float imu_x_buffer[BUFFER_SIZE];
float imu_y_buffer[BUFFER_SIZE];
float imu_z_buffer[BUFFER_SIZE];
volatile int sample_index = 0;

// DSP processing buffers (decoupled so the acquisition thread can keep
// writing the next window while the DSP thread is still crunching).
float dsp_ppg_buffer[BUFFER_SIZE];
float dsp_imu_smv[BUFFER_SIZE];
float dsp_imu_x[BUFFER_SIZE];
float dsp_imu_y[BUFFER_SIZE];
float dsp_imu_z[BUFFER_SIZE];

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

// Set true by the gesture-mode callback when AIR_MOUSE / SURFACE is
// active.  Controls two behaviours:
//   1) When the power state machine wants to enter IDLE, stop_acquisition
//      will only actually halt the timer if gesture_needs_acq is false;
//      otherwise the timer keeps ticking on IMU samples for the gesture
//      detector.
//   2) When acq_thread runs a tick and gesture_needs_acq is true while
//      MAX30102 is shut down (IDLE power), the thread takes a "gesture-
//      only" fast path: read IMU + feed gesture_mode_update_accel, skip
//      MAX fetch + HR pipeline entirely.
static volatile bool gesture_needs_acq = false;

// Tracks whether MAX30102 is currently awake.  Set to false by default
// (max30102_shutdown() runs in transition_to_idle() at boot before any
// other state is reached).  Maintained by the power state machine via
// power_max_set_awake() below; checked by the acq thread to decide
// whether the gesture-only fast path applies (MAX shut down -> no PPG,
// only IMU + gesture mode).
static volatile bool max_is_awake = false;
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
//
// Primary advertising payload lists the HRS, BAS, and HIDS service UUIDs
// so hosts can filter on whichever they care about during discovery.
// Fitness apps look for HRS (0x180D), hosts pairing as mouse look for
// HIDS (0x1812).  We have 31 bytes of payload available; three 16-bit
// UUIDs in BT_DATA_UUID16_ALL cost 8 bytes (1 type + 1 len + 3 x 2 UUID)
// plus 3 bytes for the flags = 11 bytes total.  Plenty of room.
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                  BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL)),
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

    // Load any previously-stored bonded-host keys.  Required for the
    // HID service (which insists on encrypted/bonded pairing); the
    // existing HRS doesn't require it but doesn't object either.
    if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
        int sl_err = settings_load();
        if (sl_err) {
            LOG_WRN("settings_load failed (err %d) -- continuing without "
                    "persisted bonds", sl_err);
        }
    }

    // Register the HID mouse service.  The GATT service definition is
    // installed automatically by BT_GATT_SERVICE_DEFINE; this call
    // gives the module a hook for future runtime state init and logs
    // the registration so traces show the service came up.
    int hid_err = ble_hid_init();
    if (hid_err) {
        LOG_ERR("HID init failed (err %d)", hid_err);
        /* Continue anyway -- HRS still works without HID. */
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }
    LOG_INF("Advertising successfully started (HRS + BAS + HIDS)");
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

        // "Gesture-only" mode: MAX30102 is currently shut down but
        // gesture_mode (AIR_MOUSE / SURFACE) wants continuous IMU
        // samples to drive orientation + cursor.  In this path we
        // skip MAX entirely -- the HR pipeline doesn't get any signal
        // but the gesture detector keeps running.
        bool gesture_only = !max_is_awake && gesture_needs_acq;

        if (sample_index >= BUFFER_SIZE && !gesture_only) {
            continue;
        }

        struct sensor_value red_val = {0};
        struct sensor_value ir_val = {0};
        struct sensor_value accel_x = {0};
        struct sensor_value accel_y = {0};
        struct sensor_value accel_z = {0};

        // Read the IMU first so the gesture detector still gets fed
        // even if MAX fetch is skipped / fails.
        if (sensor_sample_fetch(imu_dev) != 0) {
            LOG_ERR("IMU fetch failed");
            continue;
        }
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel_x);
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

        float ax = (float)sensor_value_to_double(&accel_x);
        float ay = (float)sensor_value_to_double(&accel_y);
        float az = (float)sensor_value_to_double(&accel_z);
        float smv = sqrtf(ax * ax + ay * ay + az * az);

        // Read the gyro from the same fetch (gyro enabled in prj.conf).
        // Zephyr returns rad/s -- gesture_mode_update_gyro expects rad/s
        // (flick detector + the new flip-signature buffer convert to
        // deg internally as needed).
        struct sensor_value gyro_x = {0}, gyro_y = {0}, gyro_z = {0};
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gyro_x);
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gyro_z);
        float gx_rps = (float)sensor_value_to_double(&gyro_x);
        float gy_rps = (float)sensor_value_to_double(&gyro_y);
        float gz_rps = (float)sensor_value_to_double(&gyro_z);

        // Feed the gesture-mode detector at the sample rate.  Cheap
        // (1 IIR per axis + a classifier branch); does its own dwell
        // counting internally so we don't need to gate it ourselves.
        gesture_mode_update_accel(ax, ay, az);
        gesture_mode_update_gyro(gx_rps, gy_rps, gz_rps);

        // In gesture-only mode we're done with this tick -- no PPG to
        // store, no DSP to drive.
        if (gesture_only) {
            continue;
        }

        // MAX30102 fetch -- only run on the HR pipeline (power state
        // is in a sampling-active mode).  If it fails here something
        // is wrong with the chip / bus state; keep the error level so
        // we notice in traces.
        if (sensor_sample_fetch(max30102_dev) != 0) {
            LOG_ERR("MAX30102 fetch failed");
            continue;
        }
        sensor_channel_get(max30102_dev, SENSOR_CHAN_RED, &red_val);
        sensor_channel_get(max30102_dev, SENSOR_CHAN_IR, &ir_val);

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

        ppg_buffer[sample_index]     = (float)sensor_value_to_double(&ir_val);
        imu_smv_buffer[sample_index] = smv;
        imu_x_buffer[sample_index]   = ax;
        imu_y_buffer[sample_index]   = ay;
        imu_z_buffer[sample_index]   = az;

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

            // Snapshot all five acquisition buffers into the DSP
            // copies so the DSP thread sees a coherent 5.12 s window
            // even as new samples land into the acquisition buffers.
            memcpy(dsp_ppg_buffer, ppg_buffer,     sizeof(float) * BUFFER_SIZE);
            memcpy(dsp_imu_smv,    imu_smv_buffer, sizeof(float) * BUFFER_SIZE);
            memcpy(dsp_imu_x,      imu_x_buffer,   sizeof(float) * BUFFER_SIZE);
            memcpy(dsp_imu_y,      imu_y_buffer,   sizeof(float) * BUFFER_SIZE);
            memcpy(dsp_imu_z,      imu_z_buffer,   sizeof(float) * BUFFER_SIZE);

            // 50% overlap: keep the second half as the start of the
            // next window, for all five acquisition buffers.
            int overlap = BUFFER_SIZE / 2;
            memmove(ppg_buffer,     &ppg_buffer[overlap],     sizeof(float) * overlap);
            memmove(imu_smv_buffer, &imu_smv_buffer[overlap], sizeof(float) * overlap);
            memmove(imu_x_buffer,   &imu_x_buffer[overlap],   sizeof(float) * overlap);
            memmove(imu_y_buffer,   &imu_y_buffer[overlap],   sizeof(float) * overlap);
            memmove(imu_z_buffer,   &imu_z_buffer[overlap],   sizeof(float) * overlap);
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

        float final_bpm = dsp.processHeartRate(dsp_ppg_buffer,
                                               dsp_imu_smv);
        WearState ws = dsp.getWearState();
        MotionState ms = dsp.getMotionState();
        bool steady_needed = dsp.needsSteady();

        if (steady_needed) {
            // Motion detected on a worn device.  The MICRO / HEAVY DSP
            // paths are parked on feature/motion-path-experiments until
            // they reach production quality; in this build we hold
            // Kalman and tell the app to prompt the user to hold still.
            LOG_INF("Wear=%s  BPM=%.2f  STAY_STEADY (motion=%d)",
                    wear_state_str(ws), (double)final_bpm, (int)ms);
        } else {
            LOG_INF("Wear=%s  BPM=%.2f", wear_state_str(ws), (double)final_bpm);
        }

        // Publish motion + wear state + window sequence for the power
        // state machine to observe.  Sequence increments unconditionally
        // so a poller can detect "new window" without missing any.
        atomic_set(&latest_motion_state, (atomic_val_t)ms);
        atomic_set(&latest_wear_state, (atomic_val_t)ws);
        atomic_inc(&latest_window_seq);

        // Push HRS notifications only when:
        //   (a) the device is actually worn, AND
        //   (b) the wrist is stationary (Kalman just got updated with a
        //       trusted reading).
        // Suppressing during motion gives the phone-side app a clear
        // signal -- the absence of recent HR notifications is the
        // "stay steady" prompt.  See needsSteady() in WearableDSP.h.
        if (ws == WEAR_WORN && !steady_needed) {
            bt_hrs_notify((uint16_t)final_bpm);
        }
    }
}

// Define the processing thread (Low priority so it doesn't block the timer)
K_THREAD_DEFINE(dsp_thread_id, 4096, dsp_thread_entry, NULL, NULL, NULL, 7, 0, 0);

// --- Serial reset thread ---
// Listens on the console UART (USB CDC ACM on the Xiao Sense) for
// single-char commands and reacts.  Useful when you can't easily
// double-tap the reset button (e.g. wrist-strapped, or you want to
// restart cleanly without re-entering the bootloader).
//
// Commands:
//   'r' / 'R' -> sys_reboot(SYS_REBOOT_COLD): full chip reset.  The
//                bootloader runs, finds no magic in GPREGRET, jumps
//                straight to the application.  Same behavior as a
//                single physical RESET press.
//   'b' / 'B' -> Enter UF2 flash mode without taking the band off.
//                Writes the Adafruit nRF52 bootloader's "stay in UF2"
//                magic (0x57) into GPREGRET, then soft-resets.  After
//                the reset the bootloader reads GPREGRET, sees the
//                magic, and stays in flash-mode (XIAO-SENSE volume
//                appears on the host) instead of jumping to the app.
//                Same behavior as a double-tap on the physical RESET
//                button.  Saves the wrist-off step when iterating.
//                Magic value verified against the Adafruit nrf52
//                bootloader source: DFU_MAGIC_UF2_RESET == 0x57.
//
// Implementation note (the v0.3 fix): the first cut used
// uart_poll_in() in a loop.  That works on a hardware UART but not
// reliably on USB CDC ACM: the Zephyr cdc_acm driver fills its RX ring
// buffer from the IRQ handler, and on this NCS version uart_poll_in
// returns -1 forever in pure-poll mode (host bytes never make it into
// the buffer the polling API drains).  Symptom was "logs flow out,
// typed commands have no effect" -- reproduced in screen too, which
// rules out Arduino IDE DTR quirks.
//
// Fix: enable RX interrupt + callback.  The callback drains the FIFO
// into a single-byte "pending command" slot which the thread polls and
// then acts on.  We deliberately do NOT call sys_reboot() from the ISR
// context -- it's allowed on Cortex-M but mixing it with USB CDC
// teardown invites a hang.
static const struct device *console_uart =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

// Set by uart_rx_cb (ISR context).  0 = no pending command, otherwise
// the ASCII byte of the command.  volatile because main-thread reads
// must observe ISR writes immediately; no memory-ordering hazard
// because a stray "old" value just delays the action by one poll cycle.
static volatile uint8_t pending_cmd = 0;

// PPG quality probe request.  Set by the 'q' console command, consumed
// by the power state thread (so all MAX/acq/power manipulation stays on
// one thread).  See run_ppg_probe().
static volatile bool probe_requested = false;

// Tap calibration mode -- set by the 'c' serial command.  When true,
// every chip-tap event logs extra diagnostic info (current TAP_THS
// value, activity-gate state).  Helps tune TAP_THS empirically per
// the discipline established 2026-06-07: pick threshold from real
// data, not datasheet guesses.
static volatile bool tap_calibration_mode = false;

// Current chip TAP_THS_6D register value (the threshold).  Tracked
// in firmware so '+'/'-' serial commands can adjust it at runtime
// without re-reading from the chip.  Updated by lsm6dsl_tap_set_threshold().
// Initial value matches what lsm6dsl_tap_engine_enable(0x08) writes.
// 1 LSB ≈ 62.5 mg at FS=±2g.
static volatile uint8_t current_tap_ths = 0x08;

// Serial-console "mouse test mode" -- set by the 'm' command.  While
// active, WASD inject cursor deltas, 1/2 generate clicks, comma/period
// scroll.  Lets us prove the cursor pipeline + HID + host integration
// end-to-end before Item 2 wires gyro tracking.  Any unrecognized char
// in this mode exits back to normal command parsing.
static volatile bool mouse_test_active = false;

static void uart_rx_cb(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);
    if (!uart_irq_update(dev)) {
        return;
    }
    if (!uart_irq_rx_ready(dev)) {
        return;
    }
    uint8_t c;
    while (uart_fifo_read(dev, &c, 1) == 1) {
        // Mouse-test commands intercept the byte stream when active.
        // We do the injections directly here at ISR context because
        // cursor_pipeline_inject_* takes a mutex and the mutex is
        // safe-but-not-recommended from ISRs.  Use a small "pending
        // motion command" code that the reset thread will consume in
        // thread context.
        if (mouse_test_active) {
            switch (c) {
            case 'w': case 'W': pending_cmd = 'w'; continue;
            case 'a': case 'A': pending_cmd = 'a'; continue;
            case 's': case 'S': pending_cmd = 's'; continue;
            case 'd': case 'D': pending_cmd = 'd'; continue;
            case '1':           pending_cmd = '1'; continue;
            case '2':           pending_cmd = '2'; continue;
            case ',':           pending_cmd = ','; continue;
            case '.':           pending_cmd = '.'; continue;
            case 'm': case 'M': pending_cmd = 'M'; continue;  // exit
            default: continue;  // ignore newlines / unknown
            }
        }
        if (c == 'r' || c == 'R') {
            pending_cmd = 'r';
        } else if (c == 'b' || c == 'B') {
            pending_cmd = 'b';
        } else if (c == 'm' || c == 'M') {
            pending_cmd = 'm';
        } else if (c == 'g' || c == 'G') {
            // Calibration helper: dump the current filtered gravity
            // vector so the user can hold the band in a pose, type 'g',
            // and capture the (gx, gy, gz) reading for orientation-
            // classifier tuning.
            pending_cmd = 'g';
        } else if (c == 't' || c == 'T') {
            // Stage 1 stand-in for the LSM6DSL chip-embedded double-
            // tap event.  Lets us verify the FSM (entry, exit, cooldown
            // re-engage) over serial before Stage 2 wires the real
            // chip event into the GPIO callback.
            pending_cmd = 't';
        } else if (c == 'y' || c == 'Y') {
            // Stage 1 stand-in for triple-tap -- the SURFACE entry
            // trigger.  Same Stage 2 deferral story as 't'.
            pending_cmd = 'y';
        } else if (c == 'c' || c == 'C') {
            // Tap calibration mode toggle.  When on, chip-tap events
            // log extra diagnostic info (current TAP_THS + activity
            // gate state).  Used with '+'/'-' to find the right
            // threshold for the user's specific environment.
            pending_cmd = 'c';
        } else if (c == '+') {
            // Lower TAP_THS by 2 LSB (~125 mg) -- more sensitive.
            pending_cmd = '+';
        } else if (c == '-') {
            // Raise TAP_THS by 2 LSB (~125 mg) -- less sensitive.
            pending_cmd = '-';
        } else if (c == 'q' || c == 'Q') {
            // PPG quality probe -- 20 s capture + perfusion-index
            // scorecard for finalising the band wear position.
            pending_cmd = 'q';
        }
        // Other chars are intentionally ignored -- silently dropping
        // newlines / CR / unknown chars keeps the listener unbothered
        // by terminal line-ending settings.
    }
}

void reset_thread_entry(void *, void *, void *) {
    if (!device_is_ready(console_uart)) {
        LOG_WRN("Console UART not ready; serial-reset disabled");
        return;
    }

    int err = uart_irq_callback_user_data_set(console_uart,
                                              uart_rx_cb, NULL);
    if (err < 0) {
        LOG_WRN("uart_irq_callback_user_data_set failed (%d); "
                "serial-reset disabled", err);
        return;
    }
    uart_irq_rx_enable(console_uart);

    LOG_INF("Serial console armed:");
    LOG_INF("  'r'=reboot   'b'=UF2 bootloader");
    LOG_INF("  'g'=dump current gravity vector (pose calibration)");
    LOG_INF("  't'=simulate chip double-tap (AIR_MOUSE entry)");
    LOG_INF("  'y'=simulate chip triple-tap (SURFACE entry)");
    LOG_INF("  'm'=force mouse test mode (wasd=move, 1/2=click, ,/.=scroll, m again exits)");
    LOG_INF("  'c'=toggle chip-tap calibration logging");
    LOG_INF("  '+'=lower TAP_THS (more sensitive)   '-'=raise TAP_THS (less sensitive)");
    LOG_INF("  'q'=PPG quality probe (20s perfusion-index capture for wear position)");

    /* Per-step cursor delta for the mouse-test injects.  10 pixels
     * per press gives a clearly visible cursor jump on the host. */
    const float MOUSE_TEST_STEP = 10.0f;

    while (1) {
        uint8_t cmd = pending_cmd;
        if (cmd == 'r') {
            pending_cmd = 0;
            LOG_INF("Serial reset requested -- rebooting");
            k_sleep(K_MSEC(50));  // let the log line drain over USB
            sys_reboot(SYS_REBOOT_COLD);
            // not reached
        } else if (cmd == 'b') {
            pending_cmd = 0;
            LOG_INF("Bootloader jump requested -- entering UF2 mode");
            k_sleep(K_MSEC(50));  // let the log line drain over USB
            // Adafruit nRF52 UF2 bootloader checks GPREGRET on boot
            // and stays in flash-mode when it sees 0x57.  Direct
            // register access -- the HAL header gives us NRF_POWER
            // but the field is a plain volatile uint32_t so this
            // is the least-version-fragile way to set it.
            NRF_POWER->GPREGRET = 0x57;
            sys_reboot(SYS_REBOOT_COLD);
            // not reached
        } else if (cmd == 'm') {
            pending_cmd = 0;
            mouse_test_active = true;
            // Force mode to AIR_MOUSE so the cursor pipeline accepts
            // and publishes our injected deltas regardless of wrist
            // orientation.  Manual mode override is fine for testing.
            gesture_mode_set(MODE_AIR_MOUSE);
            LOG_INF("Mouse test ENABLED (mode forced AIR_MOUSE).  "
                    "wasd=move, 1=left, 2=right, ,=scroll up, "
                    ".=scroll down, m=exit");
        } else if (cmd == 'M') {
            pending_cmd = 0;
            mouse_test_active = false;
            gesture_mode_set(MODE_IDLE);
            LOG_INF("Mouse test DISABLED (mode -> IDLE)");
        } else if (cmd == 'w') {
            pending_cmd = 0;
            cursor_pipeline_inject_motion(0.0f, -MOUSE_TEST_STEP);
            LOG_INF("mouse test: up");
        } else if (cmd == 's') {
            pending_cmd = 0;
            cursor_pipeline_inject_motion(0.0f, +MOUSE_TEST_STEP);
            LOG_INF("mouse test: down");
        } else if (cmd == 'a') {
            pending_cmd = 0;
            cursor_pipeline_inject_motion(-MOUSE_TEST_STEP, 0.0f);
            LOG_INF("mouse test: left");
        } else if (cmd == 'd') {
            pending_cmd = 0;
            cursor_pipeline_inject_motion(+MOUSE_TEST_STEP, 0.0f);
            LOG_INF("mouse test: right");
        } else if (cmd == '1') {
            pending_cmd = 0;
            LOG_INF("mouse test: LEFT click");
            cursor_pipeline_click(BLE_HID_BTN_LEFT);
        } else if (cmd == '2') {
            pending_cmd = 0;
            LOG_INF("mouse test: RIGHT click");
            cursor_pipeline_click(BLE_HID_BTN_RIGHT);
        } else if (cmd == ',') {
            pending_cmd = 0;
            cursor_pipeline_inject_scroll(+1.0f);
            LOG_INF("mouse test: scroll up");
        } else if (cmd == '.') {
            pending_cmd = 0;
            cursor_pipeline_inject_scroll(-1.0f);
            LOG_INF("mouse test: scroll down");
        } else if (cmd == 'g') {
            pending_cmd = 0;
            // Do a FRESH IMU read here rather than reading the cached
            // gx_filt / gy_filt / gz_filt -- if the acq pipeline isn't
            // running (e.g. power state IDLE and no gesture active),
            // the filter would be stale and the dump would be wrong.
            // Direct sensor_sample_fetch always gives us the chip's
            // latest data.
            float gx_raw = 0.0f, gy_raw = 0.0f, gz_raw = 0.0f;
            if (sensor_sample_fetch(imu_dev) == 0) {
                struct sensor_value vx, vy, vz;
                sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &vx);
                sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &vy);
                sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &vz);
                gx_raw = (float)sensor_value_to_double(&vx);
                gy_raw = (float)sensor_value_to_double(&vy);
                gz_raw = (float)sensor_value_to_double(&vz);
            } else {
                LOG_WRN("CALIBRATION: IMU fetch failed");
            }
            // Also report the filtered gravity for comparison -- if
            // it differs from the raw, the acq pipeline isn't keeping
            // up with the current pose.
            float gx_f, gy_f, gz_f;
            gesture_mode_get_gravity(&gx_f, &gy_f, &gz_f);
            LOG_INF("CALIBRATION raw_g=[%.3f, %.3f, %.3f]  "
                    "filt_g=[%.3f, %.3f, %.3f]",
                    (double)gx_raw, (double)gy_raw, (double)gz_raw,
                    (double)gx_f, (double)gy_f, (double)gz_f);
            LOG_INF("CALIBRATION raw magnitudes=[%.2f %.2f %.2f]  "
                    "orientation=%s  mode=%s  cooldown=%d ms",
                    (double)(gx_raw >= 0 ? gx_raw : -gx_raw),
                    (double)(gy_raw >= 0 ? gy_raw : -gy_raw),
                    (double)(gz_raw >= 0 ? gz_raw : -gz_raw),
                    wrist_orientation_str(gesture_mode_get_orientation()),
                    gesture_mode_str(gesture_mode_get()),
                    gesture_mode_get_cursor_cooldown_remaining() * 10);
        } else if (cmd == 't') {
            pending_cmd = 0;
            LOG_INF("Simulating chip double-tap event");
            gesture_mode_on_chip_double_tap();
        } else if (cmd == 'y') {
            pending_cmd = 0;
            LOG_INF("Simulating chip triple-tap event");
            gesture_mode_on_chip_triple_tap();
        } else if (cmd == 'c') {
            pending_cmd = 0;
            tap_calibration_mode = !tap_calibration_mode;
            LOG_INF("Tap calibration mode %s -- TAP_THS=0x%02x "
                    "(~%d mg at FS=2g).  Use '+'/'-' to adjust.",
                    tap_calibration_mode ? "ON" : "OFF",
                    current_tap_ths,
                    current_tap_ths * 62);  /* ~62.5 mg per LSB at ±2g */
        } else if (cmd == '+') {
            pending_cmd = 0;
            /* Lower threshold by 2 LSB (~125 mg), bounded at 1 (so we
             * never go below the minimum useful value). */
            uint8_t new_ths = (current_tap_ths > 2) ? (current_tap_ths - 2) : 1;
            int err = lsm6dsl_tap_set_threshold(new_ths);
            if (err == 0) {
                current_tap_ths = new_ths;
                LOG_INF("TAP_THS lowered to 0x%02x (~%d mg) -- more sensitive",
                        new_ths, new_ths * 62);
            }
        } else if (cmd == '-') {
            pending_cmd = 0;
            /* Raise threshold by 2 LSB (~125 mg), bounded at 0x1F (max
             * 5-bit value, ~1.94g). */
            uint8_t new_ths = (current_tap_ths < 0x1D) ? (current_tap_ths + 2) : 0x1F;
            int err = lsm6dsl_tap_set_threshold(new_ths);
            if (err == 0) {
                current_tap_ths = new_ths;
                LOG_INF("TAP_THS raised to 0x%02x (~%d mg) -- less sensitive",
                        new_ths, new_ths * 62);
            }
        } else if (cmd == 'q') {
            pending_cmd = 0;
            // Hand the probe to the power state thread (it owns MAX /
            // acq / power transitions -- doing them here would race).
            // Wake the power loop out of run_idle()'s k_poll so it
            // picks up the request promptly.
            LOG_INF("PPG probe requested -- power thread will run a 20s capture");
            probe_requested = true;
            k_sem_give(&motion_wake_sem);
        }
        k_sleep(K_MSEC(50));
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

// Early-abort the snapshot if the wear-state machine reports NOT_WORN
// for this many consecutive windows.  ~5 s window-step at 2.56 s/window.
// Saves ~14 s of MAX30102 LED current per unworn snapshot (band on
// desk, in a bag, etc).  STABILIZING and WORN both reset the streak,
// so a slow band-on mid-snapshot is not prematurely killed.
#define SNAPSHOT_NOT_WORN_ABORT_WINDOWS   2
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
    // If gesture_mode (AIR_MOUSE / SURFACE) still needs the IMU
    // sampling pipeline running, don't actually stop -- just let
    // the power-state side go to IDLE while gesture keeps using the
    // tick.  acq_thread's gesture_only path skips MAX entirely so
    // PPG power stays off.
    if (gesture_needs_acq) {
        LOG_DBG("stop_acquisition deferred: gesture mode still active");
        return;
    }
    // Clear the gate BEFORE stopping the timer so any tick already in
    // flight on the way to acq_thread will be dropped at the gate check
    // rather than processed in some weird half-state.
    acq_active = false;
    k_timer_stop(&acq_timer);
    // Drain any pending tick semaphore so the next start_acquisition()
    // doesn't process a stale tick before the timer's first new fire.
    k_sem_reset(&acq_tick_sem);
}

// Acquisition-request callback for gesture_mode.  See the comment on
// gesture_needs_acq above.  Wired in main() via
// gesture_mode_set_acq_request_cb().
static void gesture_acq_request_handler(bool needs) {
    gesture_needs_acq = needs;
    if (needs) {
        // Make sure the timer is running.  If acq is already active
        // (e.g. during SNAPSHOT), this is a no-op; the timer was
        // already ticking and we just flipped the flag.
        if (!acq_active) {
            LOG_INF("gesture mode: starting gesture-only acquisition");
            start_acquisition();
        } else {
            LOG_INF("gesture mode: piggybacking on existing acquisition");
        }
    } else {
        // Gesture mode no longer needs the timer.  If we're in an HR
        // sampling state (SNAPSHOT / WORKOUT), leave it running -- it
        // was going anyway.  If we're in IDLE, the timer was kept
        // alive purely for gesture; stop it now to restore the IDLE
        // power profile.
        if (power_state == PS_IDLE) {
            LOG_INF("gesture mode: ending gesture-only acquisition");
            // Call the actual stop directly here -- stop_acquisition()
            // would short-circuit because we haven't cleared
            // gesture_needs_acq yet from its perspective.  But we
            // just did clear it above.  Belt-and-braces re-clear:
            gesture_needs_acq = false;
            acq_active = false;
            k_timer_stop(&acq_timer);
            k_sem_reset(&acq_tick_sem);
        }
    }
}

// State transitions -----------------------------------------------------
// Centralised so every state entry runs through the same audit log line
// and so the order of "configure sensors" -> "set state" -> "start
// timers" is consistent.

static void transition_to_idle(void) {
    LOG_INF("PowerState: %s -> IDLE", power_state_str(power_state));
    stop_acquisition();
    max30102_shutdown();
    max_is_awake = false;
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
    max_is_awake = true;
    start_acquisition();
    power_state = PS_SNAPSHOT;
}

static void transition_to_workout_verify(void) {
    LOG_INF("PowerState: %s -> WORKOUT_VERIFY", power_state_str(power_state));
    k_timer_stop(&snapshot_tick);
    lsm6dsl_disable_sign_motion();      // we're tracking motion ourselves now
    max30102_wake();
    max_is_awake = true;
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

    // INT1 fired -- demux the source.  The pin is shared between
    // sig-motion (routed via INT1_CTRL bit 6) and the chip-embedded
    // tap engine (routed via MD1_CFG); both can fire on the same
    // edge and we have to read each source register to know what
    // happened.
    if (events[1].state == K_POLL_STATE_SEM_AVAILABLE) {
        k_sem_take(&motion_wake_sem, K_NO_WAIT);

        uint8_t tap_src = 0, func_src1 = 0;
        lsm6dsl_read_tap_src(&tap_src);
        lsm6dsl_read_func_src1(&func_src1);

        bool tap_fired = (tap_src & (1U << 6)) != 0;     // TAP_IA
        bool sigm_fired = (func_src1 & (1U << 6)) != 0;  // SIGN_MOTION_IA

        // Tap-event dispatch.  Under SINGLE_DOUBLE_TAP=0 mode the
        // chip emits ONLY SINGLE_TAP events -- each shock that
        // passes the chip's SHOCK+QUIET hardware debounce fires
        // independently.  Firmware multi-tap counter in
        // gesture_mode_on_chip_single_tap() derives double-tap
        // (AIR_MOUSE entry) and triple-tap (SURFACE entry) from
        // consecutive single-tap arrivals with its own ~250 ms
        // timing window.
        //
        // The DOUBLE_TAP bit in TAP_SRC will never be set under
        // SDT=0 -- we don't read it.  Reading TAP_SRC also drops
        // the LIR latch and de-asserts INT1, freeing the chip to
        // fire the next event.
        //
        // This path does NOT change power state.
        if (tap_fired) {
            char axis = (tap_src & 0x04) ? 'X' :
                        (tap_src & 0x02) ? 'Y' :
                        (tap_src & 0x01) ? 'Z' : '?';
            char sign = (tap_src & 0x08) ? '-' : '+';

            /* When calibration mode is on, dump the raw TAP_SRC byte
             * + current threshold for empirical-tuning visibility.
             * The gesture-mode handler will additionally log the
             * activity-gate decision and sequence count. */
            if (tap_calibration_mode) {
                LOG_INF("[CAL] tap event: TAP_SRC=0x%02x axis=%c sign=%c "
                        "(threshold=0x%02x ~%d mg)",
                        tap_src, axis, sign,
                        current_tap_ths,
                        current_tap_ths * 62);
            }

            /* Snapshot the FIFO contents for bio-acoustic feature
             * extraction.  This wakes the background worker thread
             * which logs the classification (snap vs band-tap).
             * Independent of the multi-tap counter path; both run
             * for every tap event. */
            gesture_mode_bio_acoustic_on_tap();

            gesture_mode_on_chip_single_tap(axis, sign);
        }

        // Sig-motion event: standard workout-verify path.  We only
        // honour sig-motion when it actually fired (not just because
        // a stale wake from earlier left the semaphore set).
        if (sigm_fired) {
            k_sem_reset(&snapshot_tick_sem);  // ignore tick we didn't service
            transition_to_workout_verify();
            return;
        }

        // Neither bit set: stale signal, just stay in IDLE and let
        // the poll loop re-arm.
        if (!tap_fired && !sigm_fired) {
            LOG_DBG("INT1 fired but no source bit set "
                    "(TAP_SRC=0x%02x FUNC_SRC1=0x%02x)",
                    tap_src, func_src1);
        }
        return;
    }
    if (events[0].state == K_POLL_STATE_SEM_AVAILABLE) {
        k_sem_take(&snapshot_tick_sem, K_NO_WAIT);
        transition_to_snapshot();
        return;
    }
}

static void run_snapshot(void) {
    // Run for up to SNAPSHOT_DURATION_MS, but bail to IDLE early if
    // the wear-state machine reports NOT_WORN for
    // SNAPSHOT_NOT_WORN_ABORT_WINDOWS consecutive windows.  Saves the
    // MAX30102 LED current and CPU cycles that would otherwise burn
    // for the full 22 s while the band sits unworn on a desk.
    //
    // Any non-NOT_WORN state (STABILIZING or WORN) resets the streak,
    // so a band donned mid-snapshot is not killed by an earlier brief
    // NOT_WORN observation.
    int64_t deadline = k_uptime_get() + SNAPSHOT_DURATION_MS;
    uint32_t seq_last = atomic_get(&latest_window_seq);
    uint32_t not_worn_streak = 0;

    while (k_uptime_get() < deadline) {
        k_sleep(K_MSEC(500));
        uint32_t seq_now = atomic_get(&latest_window_seq);
        if (seq_now == seq_last) continue;
        seq_last = seq_now;

        WearState w = (WearState)atomic_get(&latest_wear_state);
        if (w == WEAR_NOT_WORN) {
            if (++not_worn_streak >= SNAPSHOT_NOT_WORN_ABORT_WINDOWS) {
                LOG_INF("SNAPSHOT: NOT_WORN for %u windows -> abort to IDLE",
                        not_worn_streak);
                transition_to_idle();
                return;
            }
        } else {
            not_worn_streak = 0;
        }
    }
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

// PPG quality probe ----------------------------------------------------
//
// Runs entirely on the power-state thread so MAX / acq / power-state
// manipulation stays single-threaded.  Reuses the tested
// transition_to_snapshot() (wake MAX, start acq, PS_SNAPSHOT) and
// transition_to_idle() (shut MAX, restore IDLE) so no new power
// sequencing is introduced -- the only difference from a normal
// snapshot is the fixed duration and the probe accumulation.
#define PPG_PROBE_DURATION_MS  20000

static void run_ppg_probe(void) {
    LOG_INF("=== PPG PROBE: %d s capture -- hold still, forearm at heart height ===",
            PPG_PROBE_DURATION_MS / 1000);
    transition_to_snapshot();   // wake MAX, start acq, stop snapshot_tick
    dsp_probe_start();

    int64_t deadline = k_uptime_get() + PPG_PROBE_DURATION_MS;
    while (k_uptime_get() < deadline) {
        k_sleep(K_MSEC(250));
    }

    dsp_probe_finish_and_log();
    transition_to_idle();       // shut MAX, restore IDLE, restart snapshot_tick
}

// State machine thread -------------------------------------------------

void power_state_thread_entry(void *, void *, void *) {
    LOG_INF("Power state machine starting");
    // Initial state: IDLE (sensors off, snapshot timer running).
    transition_to_idle();

    while (1) {
        // PPG probe takes priority over normal power-state servicing.
        // probe_requested is set by the 'q' console command, which also
        // gives motion_wake_sem to break run_idle() out of its k_poll.
        if (probe_requested) {
            probe_requested = false;
            run_ppg_probe();
            continue;
        }
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

    // Initialise the gesture-mode detector state machine up front so
    // its first sample lands in well-defined memory.  The detector
    // itself runs from the acquisition thread once samples start
    // flowing.
    gesture_mode_init();
    // Wire gesture_mode -> acquisition request callback so
    // AIR_MOUSE / SURFACE modes can keep the IMU sampling pipeline
    // alive even when the HR power state is IDLE.
    gesture_mode_set_acq_request_cb(gesture_acq_request_handler);

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

    // Enable the chip-embedded tap engine.  Tap stays on for the whole
    // device lifetime -- it shares INT1 with sig-motion (routed via
    // MD1_CFG vs INT1_CTRL respectively, no register conflict), and
    // the dispatcher in run_idle() demuxes which event source fired.
    //
    // Initial TAP_THS = 0x08 (~500 mg at FS=±2g): a moderate threshold
    // we'll tune empirically via calibration mode in a later stage.
    // Don't fail boot on tap-enable error -- log and continue so we
    // still get basic gesture / HR functionality even if the I2C
    // sequence hits a transient error.
    {
        int err = lsm6dsl_tap_engine_enable(0x08);
        if (err) {
            LOG_ERR("LSM6DSL tap engine enable failed (%d) -- continuing "
                    "without chip tap detection", err);
        }
    }

    // Bio-acoustic capture pipeline (Stage E).  Enables FIFO ring
    // buffer at 1.66 kHz + starts the worker thread.  Must happen
    // AFTER tap engine enable so the FIFO ODR override correctly
    // supersedes the tap engine's 416 Hz default.
    gesture_mode_bio_acoustic_init();

    // Start the cursor publish thread.  Idle until the gesture mode
    // transitions into a cursor-bearing mode (AIR_MOUSE or SURFACE),
    // so this is safe to start unconditionally at boot.
    cursor_pipeline_init();

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
