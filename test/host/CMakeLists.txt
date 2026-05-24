cmake_minimum_required(VERSION 3.16)
project(hid_proxy_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)

set(MAIN_DIR   ${CMAKE_SOURCE_DIR}/../../main)
set(VENDOR_DIR ${CMAKE_SOURCE_DIR}/vendor)

add_executable(run_tests
    runner.c
    test_ch9329_pack.c
    test_led_duty.c
    test_button_gesture.c
    test_device_store.c
    nvs_stub.c
    ${MAIN_DIR}/ch9329_pack.c
    ${MAIN_DIR}/led_duty.c
    ${MAIN_DIR}/button_gesture.c
    ${MAIN_DIR}/device_store.c
    ${VENDOR_DIR}/unity.c
)

target_include_directories(run_tests PRIVATE
    ${MAIN_DIR}
    ${VENDOR_DIR}
    ${CMAKE_SOURCE_DIR}
)
