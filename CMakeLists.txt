# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(BLEKeyboardToHID C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(BLEKeyboardToHID logger.c usb_send.c ble_receive.c main.c )

pico_set_program_name(BLEKeyboardToHID "BLEKeyboardToHID")
pico_set_program_version(BLEKeyboardToHID "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(BLEKeyboardToHID 1)
pico_enable_stdio_usb(BLEKeyboardToHID 0)

# Add the standard library to the build
target_link_libraries(BLEKeyboardToHID
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(BLEKeyboardToHID PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/build/generated/BLEKeyboardToHID_gatt_header
)

# Add compile definitions
target_compile_definitions(BLEKeyboardToHID PRIVATE
    #WANT_HCI_DUMP=1 # This enables btstack debug
    #ENABLE_SEGGER_RTT=1
    )

pico_btstack_make_gatt_header(BLEKeyboardToHID PRIVATE "${CMAKE_CURRENT_LIST_DIR}/ble_receive_gatt.gatt")

# Add any user requested libraries
target_link_libraries(BLEKeyboardToHID 
    pico_stdlib
    pico_stdio_uart
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none
    tinyusb_device
    tinyusb_board
    )

pico_add_extra_outputs(BLEKeyboardToHID)

