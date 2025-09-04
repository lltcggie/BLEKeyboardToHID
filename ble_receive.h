#ifndef BLE_RECEIVE_H
#define BLE_RECEIVE_H

#include <stdint.h>
#include <stdbool.h>

bool ble_init();
void ble_run_loop_execute();

void ble_wakeup();

#endif // BLE_RECEIVE_H
