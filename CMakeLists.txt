# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILECOMMANDS ON)
# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/jhees/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_drone C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

include_directories(${PROJECT_SOURCE_DIR}/include)
# Add executable. Default name is the project name, version 0.1

add_executable(pico_drone pico.cpp)

pico_set_program_name(pico_drone "pico_drone")
pico_set_program_version(pico_drone "0.1")

# pico_enable_stdio_uart(pico_drone 0)
# pico_enable_stdio_usb(pico_drone 1)

# Add the standard library to the build
target_link_libraries(pico_drone pico_stdlib)

# Add any user requested libraries
target_link_libraries(pico_drone
        hardware_pwm
        hardware_adc
        hardware_spi
        )

pico_add_extra_outputs(pico_drone)

