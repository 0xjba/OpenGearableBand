#include "ble_hid.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

LOG_MODULE_REGISTER(ble_hid, LOG_LEVEL_INF);

/*
 * --- HID descriptor (report map) ---------------------------------------
 *
 * Standard mouse with 3 buttons + X/Y + scroll wheel.  Report ID is 1.
 * Generated from the canonical HID descriptor pattern -- see
 *   https://www.usb.org/sites/default/files/hut1_4.pdf  (HID Usage Tables)
 * and the Zephyr peripheral_hids sample for the same layout.
 *
 * Report layout (1 report ID byte + 4 payload bytes = 5 bytes total):
 *   byte 0:  Report ID = 1
 *   byte 1:  Buttons (bit0=left, bit1=right, bit2=middle, bits3-7 pad)
 *   byte 2:  X delta  (int8, -127..127)
 *   byte 3:  Y delta  (int8, -127..127)
 *   byte 4:  Wheel    (int8, -127..127)
 *
 * Hosts parse this descriptor on connect to know how to interpret the
 * notifications.  Standardised so Windows / Mac / Linux / Android all
 * see "a mouse" without any vendor-specific driver.
 */
#define HID_REPORT_ID_MOUSE     1
#define HID_INPUT_REPORT_LEN    5

static const uint8_t hid_report_map[] = {
    0x05, 0x01,        /* Usage Page (Generic Desktop)              */
    0x09, 0x02,        /* Usage (Mouse)                             */
    0xA1, 0x01,        /* Collection (Application)                  */
    0x09, 0x01,        /*   Usage (Pointer)                         */
    0xA1, 0x00,        /*   Collection (Physical)                   */
    0x85, HID_REPORT_ID_MOUSE,  /*     Report ID (1)                 */

    /* Buttons -- 3 bits + 5 bits pad */
    0x05, 0x09,        /*     Usage Page (Button)                   */
    0x19, 0x01,        /*     Usage Minimum (Button 1)              */
    0x29, 0x03,        /*     Usage Maximum (Button 3)              */
    0x15, 0x00,        /*     Logical Minimum (0)                   */
    0x25, 0x01,        /*     Logical Maximum (1)                   */
    0x95, 0x03,        /*     Report Count (3)                      */
    0x75, 0x01,        /*     Report Size (1)                       */
    0x81, 0x02,        /*     Input (Data,Var,Abs)                  */
    0x95, 0x01,        /*     Report Count (1)                      */
    0x75, 0x05,        /*     Report Size (5)                       */
    0x81, 0x03,        /*     Input (Const,Var,Abs) -- padding      */

    /* X / Y / Wheel -- 3 x int8 */
    0x05, 0x01,        /*     Usage Page (Generic Desktop)          */
    0x09, 0x30,        /*     Usage (X)                             */
    0x09, 0x31,        /*     Usage (Y)                             */
    0x09, 0x38,        /*     Usage (Wheel)                         */
    0x15, 0x81,        /*     Logical Minimum (-127)                */
    0x25, 0x7F,        /*     Logical Maximum (127)                 */
    0x75, 0x08,        /*     Report Size (8)                       */
    0x95, 0x03,        /*     Report Count (3)                      */
    0x81, 0x06,        /*     Input (Data,Var,Rel)                  */

    0xC0,              /*   End Collection (Physical)               */
    0xC0,              /* End Collection (Application)              */
};

/*
 * --- HID Information characteristic value -----------------------------
 *
 * Per the HOGP spec (HID Service section 7.4):
 *   bytes 0-1: bcdHID (HID Class Spec version, little-endian) = 0x0111
 *   byte   2 : bCountryCode (0 = not localised)
 *   byte   3 : Flags (bit0 = RemoteWake, bit1 = NormallyConnectable)
 */
static const uint8_t hid_info_value[4] = {
    0x11, 0x01,   /* bcdHID = 1.11 */
    0x00,         /* not localised */
    0x02,         /* NormallyConnectable */
};

/*
 * --- Service state ---------------------------------------------------
 */
static volatile bool input_notify_enabled = false;
static struct bt_conn *current_conn = NULL;

/*
 * --- GATT characteristic handlers ------------------------------------
 */

static ssize_t read_report_map(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             hid_report_map, sizeof(hid_report_map));
}

static ssize_t read_hid_info(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             hid_info_value, sizeof(hid_info_value));
}

static uint8_t protocol_mode = 0x01;  /* 0=Boot, 1=Report.  Hosts may
                                       * write to switch; we honour
                                       * Report mode only. */

static ssize_t read_protocol_mode(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &protocol_mode, sizeof(protocol_mode));
}

static ssize_t write_protocol_mode(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   const void *buf, uint16_t len,
                                   uint16_t offset, uint8_t flags)
{
    /* We only support Report mode; silently accept writes but always
     * report back Report mode on next read. */
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(buf);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    return len;
}

static uint8_t hid_control_point = 0x00;

static ssize_t write_hid_control_point(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       const void *buf, uint16_t len,
                                       uint16_t offset, uint8_t flags)
{
    /* 0 = suspend, 1 = exit suspend.  We don't implement suspend
     * handling beyond accepting the write. */
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(buf);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    return len;
}

/* Notification subscription change for the input report. */
static void input_report_ccc_changed(const struct bt_gatt_attr *attr,
                                     uint16_t value)
{
    ARG_UNUSED(attr);
    input_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("HID input-report notifications %s",
            input_notify_enabled ? "enabled" : "disabled");
}

/* Report Reference descriptor (0x2908) for the input report.  Pairs
 * the Report Map's Report ID with the input characteristic. */
static const uint8_t input_report_reference[2] = {
    HID_REPORT_ID_MOUSE,
    0x01,    /* Type 1 = Input Report */
};

static ssize_t read_input_report_reference(struct bt_conn *conn,
                                           const struct bt_gatt_attr *attr,
                                           void *buf, uint16_t len,
                                           uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             input_report_reference,
                             sizeof(input_report_reference));
}

/*
 * --- GATT service declaration ----------------------------------------
 *
 * HID Service (UUID 0x1812) with the standard characteristic set:
 *   - Protocol Mode  (0x2A4E) read/write_wo_response
 *   - Report         (0x2A4D) input report, notify, with
 *                              Report Reference descriptor (0x2908)
 *   - Report Map     (0x2A4B) read
 *   - HID Information(0x2A4A) read
 *   - HID Control Pt (0x2A4C) write_wo_response
 *
 * All read/notify characteristics require encrypted reads, per HOGP
 * spec.  The existing HRS in main.cpp remains unauthenticated so
 * fitness apps still work without pairing prompts.
 */
BT_GATT_SERVICE_DEFINE(hid_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),

    /* Protocol Mode */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_PROTOCOL_MODE,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
        read_protocol_mode, write_protocol_mode, NULL),

    /* Input Report (the one we notify on) */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        NULL, NULL, NULL),
    BT_GATT_CCC(input_report_ccc_changed,
        BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF,
        BT_GATT_PERM_READ_ENCRYPT,
        read_input_report_reference, NULL, NULL),

    /* Report Map (the descriptor that tells the host how to parse) */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ_ENCRYPT,
        read_report_map, NULL, NULL),

    /* HID Information */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ_ENCRYPT,
        read_hid_info, NULL, NULL),

    /* HID Control Point */
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT,
        BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE_ENCRYPT,
        NULL, write_hid_control_point, &hid_control_point),
);

/*
 * --- Connection tracking ----------------------------------------------
 */

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_WRN("Connection failed (err 0x%02x)", err);
        return;
    }
    /* Track only one host at a time for the cursor target.  If a
     * second host connects (multi-link mode), we keep the first as
     * the cursor target; this is a v1 simplification. */
    if (current_conn == NULL) {
        current_conn = bt_conn_ref(conn);
        LOG_INF("HID: host connected, cursor target set");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    if (current_conn == conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
        input_notify_enabled = false;
        LOG_INF("HID: host disconnected (reason 0x%02x), cursor target cleared",
                reason);
    }
}

BT_CONN_CB_DEFINE(ble_hid_conn_cb) = {
    .connected = connected,
    .disconnected = disconnected,
};

/*
 * --- Public API -------------------------------------------------------
 */

int ble_hid_init(void)
{
    /* Service is registered automatically via BT_GATT_SERVICE_DEFINE.
     * Nothing to do here for v1 -- the function exists so main can
     * call it as a hook for future state initialisation. */
    LOG_INF("HID mouse service registered (report-map size %u bytes)",
            (unsigned)sizeof(hid_report_map));
    return 0;
}

int ble_hid_send_report(uint8_t buttons, int8_t dx, int8_t dy, int8_t scroll)
{
    if (current_conn == NULL) {
        return -ENOTCONN;
    }
    if (!input_notify_enabled) {
        /* Host is connected but hasn't subscribed to notifications --
         * silently drop.  Common during the brief window after
         * connection before the host writes the CCC. */
        return -EAGAIN;
    }

    /* Compose the 5-byte report: report-ID prefix is handled by the
     * notification (we send the payload starting from byte 1). */
    uint8_t report[HID_INPUT_REPORT_LEN];
    report[0] = HID_REPORT_ID_MOUSE;
    report[1] = buttons & 0x07;  /* mask to 3 valid button bits */
    report[2] = (uint8_t)dx;
    report[3] = (uint8_t)dy;
    report[4] = (uint8_t)scroll;

    /* Find the input-report value attribute.  It's the second entry
     * in our service (after the primary service declaration).  We use
     * an offset-from-known-attribute pattern that the Zephyr CCC
     * machinery understands. */
    const struct bt_gatt_attr *report_attr = &hid_svc.attrs[4];

    int err = bt_gatt_notify(current_conn, report_attr,
                             report, sizeof(report));
    if (err && err != -ENOTCONN) {
        LOG_WRN("HID notify failed: %d", err);
    }
    return err;
}

bool ble_hid_notifications_enabled(void)
{
    return current_conn != NULL && input_notify_enabled;
}
