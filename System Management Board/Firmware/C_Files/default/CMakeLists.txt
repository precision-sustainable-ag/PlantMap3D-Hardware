cmake_minimum_required(VERSION 3.13)

include(../../../../../pico-sdk/pico_sdk_init.cmake)

project(main C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

include_directories(${CMAKE_SOURCE_DIR})

add_executable(main
    main.c
    #functions.s
)

pico_add_extra_outputs(main)
target_link_libraries(main pico_stdlib hardware_adc)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
