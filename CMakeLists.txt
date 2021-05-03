
cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(920M_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

if (TARGET tinyusb_device)
  add_executable(pico900
    pico900.c
  )

    target_link_libraries(pico900 pico_stdlib)
    pico_enable_stdio_usb(pico900 1)

    pico_add_extra_outputs(pico900)

elseif (PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()
    
