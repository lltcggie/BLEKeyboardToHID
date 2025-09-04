// "tusb.h" and "btstack.h" cannot be included at the same time because of a conflict.
// So the source code for tinyUSB and the source code for btstack need to be separated.

#include "usb_send.h"
#include "logger.h"
#include <inttypes.h>
#include <stdio.h>
#include "pico/stdlib.h"

// TinyUSB for USB HID
#include "tusb.h"
#include "class/hid/hid_device.h"

// BLE
#include "ble_receive.h"

// HID usage definitions
#define HID_USAGE_KEY_KEYBOARD_LEFTCONTROL 0xE0
#define HID_USAGE_KEY_KEYBOARD_LEFTSHIFT 0xE1
#define HID_USAGE_KEY_KEYBOARD_LEFTALT 0xE2
#define HID_USAGE_KEY_KEYBOARD_LEFT_GUI 0xE3
#define HID_USAGE_KEY_KEYBOARD_RIGHTCONTROL 0xE4
#define HID_USAGE_KEY_KEYBOARD_RIGHTSHIFT 0xE5
#define HID_USAGE_KEY_KEYBOARD_RIGHTALT 0xE6
#define HID_USAGE_KEY_KEYBOARD_RIGHT_GUI 0xE7

// repeating timer for process tinyUSB
static repeating_timer_t tinyUSB_timer;

// HID report descriptor for keyboard
uint8_t const hid_report_descriptor[] = {
    HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),
    HID_USAGE(HID_USAGE_DESKTOP_KEYBOARD),
    HID_COLLECTION(HID_COLLECTION_APPLICATION),
    /* 8 bits Modifier Keys (Shift, Control, Alt) */
    HID_USAGE_PAGE(HID_USAGE_PAGE_KEYBOARD),
    HID_USAGE_MIN(224),
    HID_USAGE_MAX(231),
    HID_LOGICAL_MIN(0),
    HID_LOGICAL_MAX(1),
    HID_REPORT_COUNT(8),
    HID_REPORT_SIZE(1),
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
    /* 8 bit reserved */
    HID_REPORT_COUNT(1),
    HID_REPORT_SIZE(8),
    HID_INPUT(HID_CONSTANT),
    /* 6-byte Keycodes */
    HID_USAGE_PAGE(HID_USAGE_PAGE_KEYBOARD),
    HID_USAGE_MIN(0),
    HID_USAGE_MAX(255),
    HID_LOGICAL_MIN(0),
    HID_LOGICAL_MAX(255),
    HID_REPORT_COUNT(6),
    HID_REPORT_SIZE(8),
    HID_INPUT(HID_DATA | HID_ARRAY | HID_ABSOLUTE),
    /* 5-bit LED Indicator Kana | Compose | ScrollLock | CapsLock | NumLock */
    HID_USAGE_PAGE(HID_USAGE_PAGE_LED),
    HID_USAGE_MIN(1),
    HID_USAGE_MAX(5),
    HID_REPORT_COUNT(5),
    HID_REPORT_SIZE(1),
    HID_OUTPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
    /* led padding */
    HID_REPORT_COUNT(1),
    HID_REPORT_SIZE(3),
    HID_OUTPUT(HID_CONSTANT),
    HID_COLLECTION_END};

// queue for HID keyboard reports
#define HID_KEYBOARD_REPORT_QUEUE_SIZE 8
typedef struct
{
    uint8_t modifier;
    uint8_t keycode[6];
} hid_keyboard_report_queue_t;
hid_keyboard_report_queue_t hid_keyboard_report_queue[HID_KEYBOARD_REPORT_QUEUE_SIZE];
static int hid_keyboard_report_queue_count = 0;

/**
 * Enqueue a HID keyboard report
 */
static void hid_enqueue_keyboard_report(uint8_t modifier, const uint8_t keycode[6])
{
    if (hid_keyboard_report_queue_count >= HID_KEYBOARD_REPORT_QUEUE_SIZE)
    {
        log_info("HID keyboard report queue full, dropping report\n");
        return;
    }
    hid_keyboard_report_queue[hid_keyboard_report_queue_count].modifier = modifier;
    memcpy(hid_keyboard_report_queue[hid_keyboard_report_queue_count].keycode, keycode, 6);
    hid_keyboard_report_queue_count++;
}

/**
 * Send or enqueue a HID keyboard report
 */
void hid_send_or_enqueue_keyboard_report(uint8_t modifier, const uint8_t keycode[6])
{
    tud_task();
    if (tud_hid_ready())
    {
        tud_hid_keyboard_report(0, modifier, keycode);
    }
    else
    {
        log_debug("HID keyboard not ready, enqueue report\n");
        hid_enqueue_keyboard_report(modifier, keycode);
    }
}

/**
 * Process HID keyboard report queue
 */
static void hid_process_keyboard_report_queue(void)
{
    if (hid_keyboard_report_queue_count == 0)
    {
        return;
    }

    if (!tud_hid_ready())
    {
        return;
    }

    tud_hid_keyboard_report(0, hid_keyboard_report_queue[0].modifier, hid_keyboard_report_queue[0].keycode);

    // shift queue
    for (int i = 1; i < hid_keyboard_report_queue_count; i++)
    {
        hid_keyboard_report_queue[i - 1] = hid_keyboard_report_queue[i];
    }
    hid_keyboard_report_queue_count--;

    log_debug("HID keyboard report queue count: %d\n", hid_keyboard_report_queue_count);
}

/**
 * TinyUSB timer callback
 */
static bool tinyUSB_timer_callback(repeating_timer_t *rt)
{
    (void)rt;

    ble_wakeup();
    return true;
}


/**
 * Handle USB task
 */
void usb_task_handler()
{
    tud_task();
    hid_process_keyboard_report_queue();
}

/**
 *  btstack thread wakeup from tinyUSB IRQ
 */
void tud_event_hook_cb(uint8_t rhport, uint32_t eventid, bool in_isr)
{
    (void)rhport;
    (void)eventid;

    if (in_isr)
    {
        ble_wakeup();
    }
}

bool tinyUSB_init()
{
    // Initialize TinyUSB
    if (!tusb_init())
    {
        return false;
    }

    // Add repeating timer to force poll tinyUSB
    if (!add_repeating_timer_ms(1, &tinyUSB_timer_callback, NULL, &tinyUSB_timer))
    {
        return false;
    }

    return true;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)bufsize;
}

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)
#define TUD_HID_REPORT_DESC_LEN sizeof(hid_report_descriptor)

uint8_t const desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 0, HID_PROTOCOL_BOOT, TUD_HID_REPORT_DESC_LEN, 0x81, 16, 1),
};

uint8_t const *tud_descriptor_device_cb(void)
{
    static tusb_desc_device_t const desc_device = {
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200,
        .bDeviceClass = 0x00,
        .bDeviceSubClass = 0x00,
        .bDeviceProtocol = 0x00,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
        .idVendor = 0xCafe,
        .idProduct = 0x4004,
        .bcdDevice = 0x0100,
        .iManufacturer = 0x01,
        .iProduct = 0x02,
        .iSerialNumber = 0x03,
        .bNumConfigurations = 0x01};
    return (uint8_t const *)&desc_device;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_configuration;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    static uint16_t _desc_str[32];
    uint8_t chr_count = 1;

    switch (index)
    {
    case 0:
        memcpy(&_desc_str[1], "BLEKeyboardToHID", 16);
        chr_count = 16;
        break;
    case 1:
        memcpy(&_desc_str[1], "Raspberry Pi", 12);
        chr_count = 12;
        break;
    case 2:
        memcpy(&_desc_str[1], "BLE Keyboard to USB", 18);
        chr_count = 18;
        break;
    case 3:
        memcpy(&_desc_str[1], "123456", 6);
        chr_count = 6;
        break;
    default:
        return NULL;
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);

    return _desc_str;
}

// TinyUSB HID callbacks
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return hid_report_descriptor;
}
