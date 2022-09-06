
cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(Pico900)

pico_sdk_init()

add_executable(pico900
    pico900.c
  )

target_link_libraries(pico900
  pico_stdlib
  pico_multicore)

pico_enable_stdio_usb(pico900 0)
pico_enable_stdio_uart(pico900 1)

pico_add_extra_outputs(pico900)

