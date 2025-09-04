/*
 * Copyright (C) 2020 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */

// "tusb.h" and "btstack.h" cannot be included at the same time because of a conflict.
// So the source code for tinyUSB and the source code for btstack need to be separated.

#include "ble_receive.h"
#include "logger.h"

#include <inttypes.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include <btstack_tlv.h>

#include "btstack.h"

// HID usage definitions
#define HID_USAGE_KEY_KEYBOARD_LEFTCONTROL 0xE0
#define HID_USAGE_KEY_KEYBOARD_LEFTSHIFT 0xE1
#define HID_USAGE_KEY_KEYBOARD_LEFTALT 0xE2
#define HID_USAGE_KEY_KEYBOARD_LEFT_GUI 0xE3
#define HID_USAGE_KEY_KEYBOARD_RIGHTCONTROL 0xE4
#define HID_USAGE_KEY_KEYBOARD_RIGHTSHIFT 0xE5
#define HID_USAGE_KEY_KEYBOARD_RIGHTALT 0xE6
#define HID_USAGE_KEY_KEYBOARD_RIGHT_GUI 0xE7

// USB
#include "usb_send.h"

#define TU_BIT(n) (1UL << (n))

/// Keyboard modifier codes bitmap
typedef enum
{
    KEYBOARD_MODIFIER_LEFTCTRL = TU_BIT(0),   ///< Left Control
    KEYBOARD_MODIFIER_LEFTSHIFT = TU_BIT(1),  ///< Left Shift
    KEYBOARD_MODIFIER_LEFTALT = TU_BIT(2),    ///< Left Alt
    KEYBOARD_MODIFIER_LEFTGUI = TU_BIT(3),    ///< Left Window
    KEYBOARD_MODIFIER_RIGHTCTRL = TU_BIT(4),  ///< Right Control
    KEYBOARD_MODIFIER_RIGHTSHIFT = TU_BIT(5), ///< Right Shift
    KEYBOARD_MODIFIER_RIGHTALT = TU_BIT(6),   ///< Right Alt
    KEYBOARD_MODIFIER_RIGHTGUI = TU_BIT(7)    ///< Right Window
} hid_keyboard_modifier_bm_t;

#include "ble_receive_gatt.h"

// TAG to store remote device address and type in TLV
#define TLV_TAG_HOGD ((((uint32_t)'H') << 24) | (((uint32_t)'O') << 16) | (((uint32_t)'G') << 8) | 'D')

typedef struct
{
    bd_addr_t addr;
    bd_addr_type_t addr_type;
} le_device_addr_t;

static enum {
    W4_WORKING,
    W4_HID_DEVICE_FOUND,
    W4_CONNECTED,
    W4_ENCRYPTED,
    W4_HID_CLIENT_CONNECTED,
    READY,
    W4_TIMEOUT_THEN_SCAN,
    W4_TIMEOUT_THEN_RECONNECT,
} app_state;

static le_device_addr_t remote_device;
static hci_con_handle_t connection_handle;
static uint16_t hids_cid;
static hid_protocol_mode_t protocol_mode = HID_PROTOCOL_MODE_REPORT;

// SDP
static uint8_t hid_descriptor_storage[500];

// used to implement connection timeout and reconnect timer
static btstack_timer_source_t connection_timer;

// USB task timer
static btstack_data_source_t usb_data_source;

// register for events from HCI/GAP and SM
static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;

// used to store remote device in TLV
static const btstack_tlv_t *btstack_tlv_singleton_impl;
static void *btstack_tlv_singleton_context;

// repeating timer for process tinyUSB
static repeating_timer_t tinyUSB_timer;

/**
 * Handle HID Input Report
 */
static void hid_handle_input_report(uint8_t service_index, const uint8_t *report, uint16_t report_len)
{
    if (report_len < 1)
        return;

    btstack_hid_parser_t parser;

    switch (protocol_mode)
    {
    case HID_PROTOCOL_MODE_BOOT:
        btstack_hid_parser_init(&parser,
                                btstack_hid_get_boot_descriptor_data(),
                                btstack_hid_get_boot_descriptor_len(),
                                HID_REPORT_TYPE_INPUT, report, report_len);
        break;

    default:
        btstack_hid_parser_init(&parser,
                                hids_client_descriptor_storage_get_descriptor_data(hids_cid, service_index),
                                hids_client_descriptor_storage_get_descriptor_len(hids_cid, service_index),
                                HID_REPORT_TYPE_INPUT, report, report_len);
        break;
    }

    uint8_t modifier = 0;
    uint8_t keys[6] = {0};
    int key_count = 0;

    uint16_t usage_page;
    uint16_t usage;
    int32_t value;
    while (btstack_hid_parser_has_more(&parser))
    {
        btstack_hid_parser_get_field(&parser, &usage_page, &usage, &value);
        if (usage_page != HID_USAGE_PAGE_KEYBOARD)
        {
            continue;
        }

        switch (usage)
        {
        case HID_USAGE_KEY_KEYBOARD_LEFTSHIFT:
            if (value)
            {
                modifier |= KEYBOARD_MODIFIER_LEFTSHIFT;
            }
            break;
        case HID_USAGE_KEY_KEYBOARD_RIGHTSHIFT:
            if (value)
            {
                modifier |= KEYBOARD_MODIFIER_RIGHTSHIFT;
            }
            break;
        case HID_USAGE_KEY_KEYBOARD_LEFTCONTROL:
            if (value)
            {
                modifier |= KEYBOARD_MODIFIER_LEFTCTRL;
            }
            break;
        case HID_USAGE_KEY_KEYBOARD_RIGHTCONTROL:
            if (value)
            {
                modifier |= KEYBOARD_MODIFIER_RIGHTCTRL;
            }
            break;
        case HID_USAGE_KEY_KEYBOARD_LEFTALT:
            if (value)
            {
                modifier |= KEYBOARD_MODIFIER_LEFTALT;
            }
            break;
        case HID_USAGE_KEY_KEYBOARD_RIGHTALT:
            if (value)
            {
                modifier |= KEYBOARD_MODIFIER_RIGHTALT;
            }
            break;
        case HID_USAGE_KEY_KEYBOARD_LEFT_GUI:
            if (value)
            {
                modifier |= KEYBOARD_MODIFIER_LEFTGUI;
            }
            break;
        case HID_USAGE_KEY_KEYBOARD_RIGHT_GUI:
            if (value)
            {
                modifier |= KEYBOARD_MODIFIER_RIGHTGUI;
            }
            break;
        case 0x00:
            break;
        default:
            if (value && key_count < 6)
            {
                keys[key_count++] = (uint8_t)usage;
            }
            else if (value)
            {
                printf("WARNING: too many keys pressed, ignoring key 0x%02x\n", usage);
            }
            break;
        }
    }

    // Send USB HID report
    hid_send_or_enqueue_keyboard_report(modifier, keys);
}

/**
 * @section Test if advertisement contains HID UUID
 * @param packet
 * @param size
 * @returns true if it does
 */
static bool adv_event_contains_hid_service(const uint8_t *packet)
{
    const uint8_t *ad_data = gap_event_advertising_report_get_data(packet);
    uint8_t ad_len = gap_event_advertising_report_get_data_length(packet);
    return ad_data_contains_uuid16(ad_len, ad_data, ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE);
}

/**
 * Start scanning
 */
static void hog_start_scan(void)
{
    printf("Scanning for LE HID devices...\n");
    app_state = W4_HID_DEVICE_FOUND;
    // Passive scanning, 100% (scan interval = scan window)
    gap_set_scan_parameters(0, 48, 48);
    gap_start_scan();
}

/**
 * Handle timeout for outgoing connection
 * @param ts
 */
static void hog_connection_timeout(btstack_timer_source_t *ts)
{
    UNUSED(ts);
    printf("Timeout - abort connection\n");
    gap_connect_cancel();
    hog_start_scan();
}

/**
 * Handle USB task
 * @param ts
 */
static void usb_task_data_source_handler(btstack_data_source_t *ds, btstack_data_source_callback_type_t callback_type)
{
    UNUSED(ds);
    UNUSED(callback_type);
    usb_task_handler();
}

/**
 * Connect to remote device but set timer for timeout
 */
static void hog_connect(void)
{
    // set timer
    btstack_run_loop_set_timer(&connection_timer, 10000);
    btstack_run_loop_set_timer_handler(&connection_timer, &hog_connection_timeout);
    btstack_run_loop_add_timer(&connection_timer);
    app_state = W4_CONNECTED;
    gap_connect(remote_device.addr, remote_device.addr_type);
}

/**
 * Handle timer event to trigger reconnect
 * @param ts
 */
static void hog_reconnect_timeout(btstack_timer_source_t *ts)
{
    UNUSED(ts);
    switch (app_state)
    {
    case W4_TIMEOUT_THEN_RECONNECT:
        hog_connect();
        break;
    case W4_TIMEOUT_THEN_SCAN:
        hog_start_scan();
        break;
    default:
        break;
    }
}

/**
 * Start connecting after boot up: connect to last used device if possible, start scan otherwise
 */
static void hog_start_connect(void)
{
    // check if we have a bonded device
    btstack_tlv_get_instance(&btstack_tlv_singleton_impl, &btstack_tlv_singleton_context);
    if (btstack_tlv_singleton_impl)
    {
        int len = btstack_tlv_singleton_impl->get_tag(btstack_tlv_singleton_context, TLV_TAG_HOGD, (uint8_t *)&remote_device, sizeof(remote_device));
        if (len == sizeof(remote_device))
        {
            printf("Bonded, connect to device with %s address %s ...\n", remote_device.addr_type == 0 ? "public" : "random", bd_addr_to_str(remote_device.addr));
            hog_connect();
            return;
        }
    }
    // otherwise, scan for HID devices
    hog_start_scan();
}

/**
 * In case of error, disconnect and start scanning again
 */
static void handle_outgoing_connection_error(void)
{
    printf("Error occurred, disconnect and start over\n");
    gap_disconnect(connection_handle);
    hog_start_scan();
}

/**
 * Handle GATT Client Events dependent on current state
 *
 * @param packet_type
 * @param channel
 * @param packet
 * @param size
 */
static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    uint8_t status;

    if (hci_event_packet_get_type(packet) != HCI_EVENT_GATTSERVICE_META)
    {
        return;
    }

    switch (hci_event_gattservice_meta_get_subevent_code(packet))
    {
    case GATTSERVICE_SUBEVENT_HID_SERVICE_CONNECTED:
        status = gattservice_subevent_hid_service_connected_get_status(packet);
        switch (status)
        {
        case ERROR_CODE_SUCCESS:
            printf("HID service client connected, found %d services\n",
                   gattservice_subevent_hid_service_connected_get_num_instances(packet));

            // store device as bonded
            if (btstack_tlv_singleton_impl)
            {
                btstack_tlv_singleton_impl->store_tag(btstack_tlv_singleton_context, TLV_TAG_HOGD, (const uint8_t *)&remote_device, sizeof(remote_device));
            }
            // done
            printf("Ready - please start typing or mousing..\n");
            app_state = READY;
            break;
        default:
            printf("HID service client connection failed, status 0x%02x.\n", status);
            handle_outgoing_connection_error();
            break;
        }
        break;

    case GATTSERVICE_SUBEVENT_HID_SERVICE_DISCONNECTED:
        printf("HID service client disconnected\n");
        hog_start_connect();
        break;

    case GATTSERVICE_SUBEVENT_HID_REPORT:
        hid_handle_input_report(
            gattservice_subevent_hid_report_get_service_index(packet),
            gattservice_subevent_hid_report_get_report(packet),
            gattservice_subevent_hid_report_get_report_len(packet));
        break;

    default:
        break;
    }
}

/* LISTING_START(packetHandler): Packet Handler */
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    /* LISTING_PAUSE */
    UNUSED(channel);
    UNUSED(size);
    uint8_t event;
    /* LISTING_RESUME */
    switch (packet_type)
    {
    case HCI_EVENT_PACKET:
        event = hci_event_packet_get_type(packet);
        switch (event)
        {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING)
                break;
            btstack_assert(app_state == W4_WORKING);

            hog_start_connect();
            break;
        case GAP_EVENT_ADVERTISING_REPORT:
            if (app_state != W4_HID_DEVICE_FOUND)
                break;
            if (adv_event_contains_hid_service(packet) == false)
                break;
            // stop scan
            gap_stop_scan();
            // store remote device address and type
            gap_event_advertising_report_get_address(packet, remote_device.addr);
            remote_device.addr_type = gap_event_advertising_report_get_address_type(packet);
            // connect
            printf("Found, connect to device with %s address %s ...\n", remote_device.addr_type == 0 ? "public" : "random", bd_addr_to_str(remote_device.addr));
            hog_connect();
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            if (app_state == W4_ENCRYPTED)
            {
                printf("HID service client connection failed.\n");
                handle_outgoing_connection_error();
                connection_handle = HCI_CON_HANDLE_INVALID;
            }
            else if (app_state == READY)
            {
                connection_handle = HCI_CON_HANDLE_INVALID;
                switch (app_state)
                {
                case READY:
                    printf("\nDisconnected, try to reconnect...\n");
                    app_state = W4_TIMEOUT_THEN_RECONNECT;
                    break;
                default:
                    printf("\nDisconnected, start over...\n");
                    app_state = W4_TIMEOUT_THEN_SCAN;
                    break;
                }
                // set timer
                btstack_run_loop_set_timer(&connection_timer, 100);
                btstack_run_loop_set_timer_handler(&connection_timer, &hog_reconnect_timeout);
                btstack_run_loop_add_timer(&connection_timer);
            }
            break;
        case HCI_EVENT_META_GAP:
            // wait for connection complete
            if (hci_event_gap_meta_get_subevent_code(packet) != GAP_SUBEVENT_LE_CONNECTION_COMPLETE)
                break;
            if (app_state != W4_CONNECTED)
                return;
            btstack_run_loop_remove_timer(&connection_timer);
            connection_handle = gap_subevent_le_connection_complete_get_connection_handle(packet);
            // request security
            app_state = W4_ENCRYPTED;
            sm_request_pairing(connection_handle);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}
/* LISTING_END */

/* @section HCI packet handler
 *
 * @text The SM packet handler receives Security Manager Events required for pairing.
 * It also receives events generated during Identity Resolving
 * see Listing SMPacketHandler.
 */

/* LISTING_START(SMPacketHandler): Scanning and receiving advertisements */

static void sm_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET)
        return;

    bd_addr_t addr;
    bd_addr_type_t addr_type;
    bool connect_to_service = false;

    uint8_t event = hci_event_packet_get_type(packet);
    switch (event)
    {
    case SM_EVENT_JUST_WORKS_REQUEST:
        printf("Just works requested\n");
        sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
        break;
    case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
        printf("Confirming numeric comparison: %06" PRIu32 "\n", sm_event_numeric_comparison_request_get_passkey(packet));
        sm_numeric_comparison_confirm(sm_event_passkey_display_number_get_handle(packet));
        break;
    case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
        printf("Display Passkey: %06" PRIu32 "\n", sm_event_passkey_display_number_get_passkey(packet));
        break;
    case SM_EVENT_PAIRING_COMPLETE:
        switch (sm_event_pairing_complete_get_status(packet))
        {
        case ERROR_CODE_SUCCESS:
            printf("Pairing complete, success\n");
            connect_to_service = true;
            break;
        case ERROR_CODE_CONNECTION_TIMEOUT:
            printf("Pairing failed, timeout\n");
            break;
        case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
            printf("Pairing failed, disconnected\n");
            break;
        case ERROR_CODE_AUTHENTICATION_FAILURE:
            printf("Pairing failed, reason = %u\n", sm_event_pairing_complete_get_reason(packet));
            break;
        default:
            break;
        }
        break;
    case SM_EVENT_REENCRYPTION_COMPLETE:
        switch (sm_event_reencryption_complete_get_status(packet))
        {
        case ERROR_CODE_SUCCESS:
            printf("Re-encryption complete, success\n");
            break;
        case ERROR_CODE_CONNECTION_TIMEOUT:
            printf("Re-encryption failed, timeout\n");
            break;
        case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
            printf("Re-encryption failed, disconnected\n");
            printf("Deleting local bonding information...\n");
            sm_event_reencryption_complete_get_address(packet, addr);
            addr_type = sm_event_reencryption_started_get_addr_type(packet);
            gap_delete_bonding(addr_type, addr);
            break;
        case ERROR_CODE_PIN_OR_KEY_MISSING:
            printf("Re-encryption failed, bonding information missing\n\n");
            printf("Assuming remote lost bonding information\n");
            printf("Deleting local bonding information and start new pairing...\n");
            sm_event_reencryption_complete_get_address(packet, addr);
            addr_type = sm_event_reencryption_started_get_addr_type(packet);
            gap_delete_bonding(addr_type, addr);
            sm_request_pairing(sm_event_reencryption_complete_get_handle(packet));
            break;
        default:
            break;
        }
        connect_to_service = true;
        break;
    default:
        break;
    }

    if (connect_to_service)
    {
        // continue - query primary services
        printf("Search for HID service.\n");
        app_state = W4_HID_CLIENT_CONNECTED;
        hids_client_connect(connection_handle, handle_gatt_client_event, protocol_mode, &hids_cid);
    }
}
/* LISTING_END */

bool ble_init()
{

    // initialize CYW43 driver architecture (will enable BT if/because CYW43_ENABLE_BLUETOOTH == 1)
    if (cyw43_arch_init())
    {
        printf("failed to initialise cyw43_arch\n");
        return false;
    }

#if WANT_HCI_DUMP
    // disable HCI packet dump
    hci_dump_enable_packet_log(false);
#endif

#if 0
    // Clear Pairings
    btstack_tlv_get_instance(&btstack_tlv_singleton_impl, &btstack_tlv_singleton_context);
    if (btstack_tlv_singleton_impl){
        btstack_tlv_singleton_impl->delete_tag(btstack_tlv_singleton_context, TLV_TAG_HOGD);
    }

    for (int i = 0; i < le_device_db_max_count(); i++){
        le_device_db_remove(i);
    }
#endif

    /* LISTING_START(HogBootHostSetup): HID-over-GATT Host Setup */

    l2cap_init();

    // setup SM: Display only
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_DISPLAY_ONLY);
    sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_BONDING | SM_AUTHREQ_MITM_PROTECTION);

    //
    gatt_client_init();

    // security settings
    // https://bluekitchen-gmbh.com/blesa-gatt-client-vulnerabilities/
    gatt_client_set_required_security_level(LEVEL_3);

    // setup ATT server - only needed if LE Peripheral does ATT queries on its own, e.g. Android and iOS
    att_server_init(profile_data, NULL, NULL);

    hids_client_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));

    // register for events from HCI
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // register for events from Security Manager
    sm_event_callback_registration.callback = &sm_packet_handler;
    sm_add_event_handler(&sm_event_callback_registration);

    /* LISTING_END */

    // Disable stdout buffering
    setvbuf(stdin, NULL, _IONBF, 0);

    app_state = W4_WORKING;

    // Register USB task
    btstack_run_loop_set_data_source_handler(&usb_data_source, &usb_task_data_source_handler);
    btstack_run_loop_base_enable_data_source_callbacks(&usb_data_source, DATA_SOURCE_CALLBACK_POLL);
    btstack_run_loop_base_add_data_source(&usb_data_source);

    // Turn on the device
    hci_power_control(HCI_POWER_ON);

    return true;
}

void ble_run_loop_execute()
{
    btstack_run_loop_execute();
}

void ble_wakeup()
{
    btstack_run_loop_poll_data_sources_from_irq();
}
