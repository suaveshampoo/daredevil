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

function(configure_pico_target target_name)
    pico_set_program_name(${target_name} "${target_name}")
    pico_set_program_version(${target_name} "0.1")
    target_include_directories(${target_name} PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
    )
    pico_add_extra_outputs(${target_name})
    pico_enable_stdio_usb(${target_name} 1)
    pico_enable_stdio_uart(${target_name} 0)
endfunction()

add_executable(daredevil daredevil.c)
add_executable(spin spin.c)
add_executable(lidar_scan lidar_scan.c)
add_executable(lidar_packet_benchmark lidar_packet_benchmark.c lidar_scan.c)
add_executable(merged_scan merged_scan.c spin.c lidar_scan.c)

# Generate PIO header
pico_generate_pio_header(daredevil ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

configure_pico_target(daredevil)
configure_pico_target(spin)
configure_pico_target(lidar_scan)
configure_pico_target(lidar_packet_benchmark)
configure_pico_target(merged_scan)

target_link_libraries(daredevil
        pico_stdlib
        hardware_uart
        hardware_pio
        )
target_link_libraries(spin
        pico_stdlib
        )
target_link_libraries(lidar_scan
        pico_stdlib
        hardware_uart
        )
target_link_libraries(lidar_packet_benchmark
        pico_stdlib
        hardware_uart
        )
target_link_libraries(merged_scan
        pico_stdlib
        hardware_uart
)

target_compile_definitions(lidar_packet_benchmark PRIVATE LIDAR_SCAN_NO_MAIN)
target_compile_definitions(merged_scan PRIVATE SPIN_NO_MAIN LIDAR_SCAN_NO_MAIN)
