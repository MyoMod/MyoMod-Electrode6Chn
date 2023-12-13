# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/leon/Programs/pico/pico-sdk")

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(EMGLogger C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(EMGLogger Main.cpp comInterface.cpp max11254.cpp max11254_hal.cpp i2c.cpp timerSync.cpp stateDisplay.cpp processingUnit.cpp) 

# Print size of executable
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND arm-none-eabi-size ${CMAKE_CURRENT_LIST_DIR}/build/${PROJECT_NAME}.elf
    VERBATIM
)
string(APPEND CMAKE_EXE_LINKER_FLAGS "-Wl,--print-memory-usage")

pico_set_program_name(EMGLogger "EMGLogger")
pico_set_program_version(EMGLogger "0.1")

pico_enable_stdio_uart(EMGLogger 1)
pico_enable_stdio_usb(EMGLogger 0)

# Add the standard library to the build
target_link_libraries(EMGLogger
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(EMGLogger PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(EMGLogger 
        hardware_spi
        hardware_i2c
        hardware_dma
        hardware_pio
        hardware_interp
        hardware_timer
        hardware_watchdog
        hardware_clocks
        hardware_adc
        hardware_resets 
        hardware_irq
        hardware_pwm
        pico_unique_id
        pico_multicore
        pico_sync
        )

pico_add_extra_outputs(EMGLogger)

