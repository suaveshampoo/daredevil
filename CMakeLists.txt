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
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(daredevil C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(daredevil daredevil.c )
add_executable(stepper_test stepper_test.c)
add_executable(spin spin.c)
add_executable(lidar_scan lidar_scan.c)

pico_set_program_name(daredevil "daredevil")
pico_set_program_version(daredevil "0.1")
pico_set_program_name(stepper_test "stepper_test")
pico_set_program_version(stepper_test "0.1")
pico_set_program_name(spin "spin")
pico_set_program_version(spin "0.1")
pico_set_program_name(lidar_scan "lidar_scan")
pico_set_program_version(lidar_scan "0.1")

# Generate PIO header
pico_generate_pio_header(daredevil ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Add the standard include files to the build
target_include_directories(daredevil PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)
target_include_directories(stepper_test PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)
target_include_directories(spin PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)
target_include_directories(lidar_scan PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(daredevil
        pico_stdlib
        hardware_uart
        hardware_pio
        )
target_link_libraries(stepper_test
        pico_stdlib
        )
target_link_libraries(spin
        pico_stdlib
        )
target_link_libraries(lidar_scan
        pico_stdlib
        hardware_uart
        )

pico_add_extra_outputs(daredevil)
pico_add_extra_outputs(stepper_test)
pico_add_extra_outputs(spin)
pico_add_extra_outputs(lidar_scan)
pico_enable_stdio_usb(daredevil 1)
pico_enable_stdio_uart(daredevil 0)
pico_enable_stdio_usb(stepper_test 1)
pico_enable_stdio_uart(stepper_test 0)
pico_enable_stdio_usb(spin 1)
pico_enable_stdio_uart(spin 0)
pico_enable_stdio_usb(lidar_scan 1)
pico_enable_stdio_uart(lidar_scan 0)
