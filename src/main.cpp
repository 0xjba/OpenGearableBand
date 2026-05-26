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

#include "WearableDSP.h"

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
    uint32_t count = 0;
    window_start_ms = k_uptime_get();

    while (1) {
        k_sem_take(&acq_tick_sem, K_FOREVER);

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

        ppg_buffer[sample_index] = (float)sensor_value_to_double(&ir_val);
        imu_buffer[sample_index] = smv;

        if (count < 10) {
            LOG_INF("Sample %u: PPG(IR)=%.1f, accel=(%.2f, %.2f, %.2f) SMV=%.3f",
                    count, (double)ppg_buffer[sample_index],
                    (double)ax, (double)ay, (double)az, (double)smv);
        }

        sample_index++;
        count++;

        if (sample_index >= BUFFER_SIZE) {
            int64_t now = k_uptime_get();
            int64_t elapsed_ms = now - window_start_ms;
            // Measured rate over the half-window (256 samples after the
            // first window; BUFFER_SIZE samples for the very first one).
            uint32_t samples_in_window =
                (count == BUFFER_SIZE) ? BUFFER_SIZE : (BUFFER_SIZE / 2);
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

        LOG_INF("Wear=%s  BPM=%.2f", wear_state_str(ws), (double)final_bpm);

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

    // 10 ms periodic tick drives the acquisition thread.  k_timer fires from
    // the system timer ISR, so the cadence is independent of I2C latency or
    // DSP processing time.
    k_timer_start(&acq_timer, K_MSEC(10), K_MSEC(10));

    while (1) {
        // Main thread can handle background tasks such as battery reading
        // (SAADC on P0.31) and updating BAS.
        k_sleep(K_MSEC(1000));
    }
    return 0;
}
