cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from submodule
# note: this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)

project(roadrunner)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/generated)

# UART with RGB LED ordering
add_executable(roadrunner_v1_uart_rgb)
pico_enable_stdio_usb(roadrunner_v1_uart_rgb 1)
pico_enable_stdio_uart(roadrunner_v1_uart_rgb 0)
pico_generate_pio_header(roadrunner_v1_uart_rgb ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
target_sources(roadrunner_v1_uart_rgb PRIVATE neopixel.c as5600.c ir_sensor.c tmcuart.c main.c)
target_link_libraries(roadrunner_v1_uart_rgb PRIVATE pico_stdlib hardware_pio hardware_i2c)
pico_add_extra_outputs(roadrunner_v1_uart_rgb)

# UART with GRB LED ordering
add_executable(roadrunner_v1_uart_grb)
pico_enable_stdio_usb(roadrunner_v1_uart_grb 1)
pico_enable_stdio_uart(roadrunner_v1_uart_grb 0)
target_compile_definitions(roadrunner_v1_uart_grb PUBLIC GRB_LED_ORDER=1)
pico_generate_pio_header(roadrunner_v1_uart_grb ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
target_sources(roadrunner_v1_uart_grb PRIVATE neopixel.c as5600.c ir_sensor.c tmcuart.c main.c)
target_link_libraries(roadrunner_v1_uart_grb PRIVATE pico_stdlib hardware_pio hardware_i2c)
pico_add_extra_outputs(roadrunner_v1_uart_grb)

# I2C with RGB LED ordering
add_executable(roadrunner_v1_i2c_rgb)
pico_enable_stdio_usb(roadrunner_v1_i2c_rgb 1)
pico_enable_stdio_uart(roadrunner_v1_i2c_rgb 0)
target_compile_definitions(roadrunner_v1_i2c_rgb PUBLIC IS_I2C_TARGET=1)
pico_generate_pio_header(roadrunner_v1_i2c_rgb ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
target_sources(roadrunner_v1_i2c_rgb PRIVATE neopixel.c as5600.c ir_sensor.c i2c_target.c main.c)
target_link_libraries(roadrunner_v1_i2c_rgb PRIVATE pico_i2c_slave pico_stdlib hardware_pio hardware_i2c)
pico_add_extra_outputs(roadrunner_v1_i2c_rgb)

# I2C with GRB LED ordering
add_executable(roadrunner_v1_i2c_grb)
pico_enable_stdio_usb(roadrunner_v1_i2c_grb 1)
pico_enable_stdio_uart(roadrunner_v1_i2c_grb 0)
target_compile_definitions(roadrunner_v1_i2c_grb PUBLIC IS_I2C_TARGET=1)
target_compile_definitions(roadrunner_v1_i2c_grb PUBLIC GRB_LED_ORDER=1)
pico_generate_pio_header(roadrunner_v1_i2c_grb ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
target_sources(roadrunner_v1_i2c_grb PRIVATE neopixel.c as5600.c ir_sensor.c i2c_target.c main.c)
target_link_libraries(roadrunner_v1_i2c_grb PRIVATE pico_i2c_slave pico_stdlib hardware_pio hardware_i2c)
pico_add_extra_outputs(roadrunner_v1_i2c_grb)

# USB serial with RGB LED ordering
add_executable(roadrunner_v1_usbserial_rgb)
pico_enable_stdio_usb(roadrunner_v1_usbserial_rgb 1)
pico_enable_stdio_uart(roadrunner_v1_usbserial_rgb 0)
target_compile_definitions(roadrunner_v1_usbserial_rgb PUBLIC USB_SERIAL_COMMS=1)
pico_generate_pio_header(roadrunner_v1_usbserial_rgb ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
target_sources(roadrunner_v1_usbserial_rgb PRIVATE neopixel.c as5600.c ir_sensor.c usbserial.c main.c)
target_link_libraries(roadrunner_v1_usbserial_rgb PRIVATE pico_stdlib hardware_pio hardware_i2c)
pico_add_extra_outputs(roadrunner_v1_usbserial_rgb)

# USB serial with GRB LED ordering
add_executable(roadrunner_v1_usbserial_grb)
pico_enable_stdio_usb(roadrunner_v1_usbserial_grb 1)
pico_enable_stdio_uart(roadrunner_v1_usbserial_grb 0)
target_compile_definitions(roadrunner_v1_usbserial_grb PUBLIC USB_SERIAL_COMMS=1)
target_compile_definitions(roadrunner_v1_usbserial_grb PUBLIC GRB_LED_ORDER=1)
pico_generate_pio_header(roadrunner_v1_usbserial_grb ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
target_sources(roadrunner_v1_usbserial_grb PRIVATE neopixel.c as5600.c ir_sensor.c usbserial.c main.c)
target_link_libraries(roadrunner_v1_usbserial_grb PRIVATE pico_stdlib hardware_pio hardware_i2c)
pico_add_extra_outputs(roadrunner_v1_usbserial_grb)

# UART serial with RGB LED ordering
add_executable(roadrunner_v1_uartserial_rgb)
pico_enable_stdio_usb(roadrunner_v1_uartserial_rgb 10)
pico_enable_stdio_uart(roadrunner_v1_uartserial_rgb 1)
target_compile_definitions(roadrunner_v1_uartserial_rgb PUBLIC UART_SERIAL_COMMS=1)
pico_generate_pio_header(roadrunner_v1_uartserial_rgb ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
target_sources(roadrunner_v1_uartserial_rgb PRIVATE neopixel.c as5600.c ir_sensor.c uartserial.c main.c)
target_link_libraries(roadrunner_v1_uartserial_rgb PRIVATE pico_stdlib hardware_pio hardware_i2c)
pico_add_extra_outputs(roadrunner_v1_uartserial_rgb)

# UART serial with GRB LED ordering
add_executable(roadrunner_v1_uartserial_grb)
pico_enable_stdio_usb(roadrunner_v1_uartserial_grb 0)
pico_enable_stdio_uart(roadrunner_v1_uartserial_grb 1)
target_compile_definitions(roadrunner_v1_uartserial_grb PUBLIC UART_SERIAL_COMMS=1)
target_compile_definitions(roadrunner_v1_uartserial_grb PUBLIC GRB_LED_ORDER=1)
pico_generate_pio_header(roadrunner_v1_uartserial_grb ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
target_sources(roadrunner_v1_uartserial_grb PRIVATE neopixel.c as5600.c ir_sensor.c uartserial.c main.c)
target_link_libraries(roadrunner_v1_uartserial_grb PRIVATE pico_stdlib hardware_pio hardware_i2c)
pico_add_extra_outputs(roadrunner_v1_uartserial_grb)