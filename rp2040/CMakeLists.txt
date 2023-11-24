cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from submodule
# note: this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)

project(roadrunner)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(roadrunner_v1)
pico_enable_stdio_usb(roadrunner_v1 1)
pico_enable_stdio_uart(roadrunner_v1 0)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/generated)

pico_generate_pio_header(roadrunner_v1 ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

target_sources(roadrunner_v1 PRIVATE neopixel.c as5600.c ir_sensor.c tmcuart.c main.c)

target_link_libraries(roadrunner_v1 PRIVATE pico_stdlib hardware_pio hardware_i2c)
pico_add_extra_outputs(roadrunner_v1)
