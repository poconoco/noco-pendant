set(PICO_BOARD pico2 CACHE STRING "Board type")
cmake_minimum_required(VERSION 3.27)

set(ENV{PICO_SDK_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(main C CXX ASM)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(main)

pico_generate_pio_header(main ${CMAKE_CURRENT_LIST_DIR}/lib/WS2812/WS2812.pio
                        OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/WS2812)

target_sources(main PRIVATE
    main.cpp
)

# Add compilation subdirectory
add_subdirectory(./lib/Config)
add_subdirectory(./lib/QMI8658)
add_subdirectory(./lib/WS2812)
add_subdirectory(./faces/base)
add_subdirectory(./faces/fluid)
add_subdirectory(./faces/minecraft)

# Add header file directory
include_directories(./lib/Config)
include_directories(./lib/QMI8658)
include_directories(./lib/WS2812)
include_directories(./faces/base)
include_directories(./faces/fluid)
include_directories(./faces/minecraft)

target_link_libraries(main PRIVATE
    Config
    QMI8658
    WS2812
    Face
    Fluid
    Minecraft
    pico_stdlib
    hardware_pio
    hardware_gpio
)

# enable usb output, disable uart output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)

pico_add_extra_outputs(main)

# add url via pico_set_program_url

