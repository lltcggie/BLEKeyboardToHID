#ifndef USB_SEND_H
#define USB_SEND_H

#include <stdint.h>
#include <stdbool.h>

bool tinyUSB_init();
void usb_task_handler();

void hid_send_or_enqueue_keyboard_report(uint8_t modifier, const uint8_t keycode[6]);

#endif // USB_SEND_H
