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
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(mult_led C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(mult_led mult_led.c)

pico_set_program_name(mult_led "mult_led")
pico_set_program_version(mult_led "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(mult_led 1)

# turns out I had this at 0... :( 
pico_enable_stdio_usb(mult_led 1)

# Add the standard library to the build
target_link_libraries(mult_led
        pico_stdlib)

pico_generate_pio_header(mult_led ${CMAKE_CURRENT_LIST_DIR}/pwm_waveform_output.pio)

# Add the standard include files to the build
target_include_directories(mult_led PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(mult_led 
        hardware_dma
        hardware_pio
        hardware_timer
        hardware_clocks
        hardware_irq
        hardware_pwm
        )

pico_add_extra_outputs(mult_led)

