#include "ble_receive.h"
#include "usb_send.h"
#include "logger.h"
#include "pico/stdlib.h"
#include <stdio.h>

int main(int argc, const char *argv[])
{
    (void)argc;
    (void)argv;

    stdio_init_all();

    if (!ble_init())
    {
        log_error("failed to initialise ble\n");
        return -1;
    }

    if (!tinyUSB_init())
    {
        log_error("failed to initialise tinyUSB\n");
        return -1;
    }

    ble_run_loop_execute();

    return 0;
}
