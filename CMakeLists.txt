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
set(sdkVersion 1.5.1)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Projeto_Final_U7LFdP C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Projeto_Final_U7LFdP 
    Projeto_Final_U7LFdP.c 
    Microphone_dma.c
    inc/ssd1306_i2c.c)

pico_set_program_name(Projeto_Final_U7LFdP "Projeto_Final_U7LFdP")
pico_set_program_version(Projeto_Final_U7LFdP "0.1")

# Generate PIO header
pico_generate_pio_header(Projeto_Final_U7LFdP ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Projeto_Final_U7LFdP 0)
pico_enable_stdio_usb(Projeto_Final_U7LFdP 1)

# Add the standard library to the build
target_link_libraries(Projeto_Final_U7LFdP
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(Projeto_Final_U7LFdP PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

set(KISSFFT_TOOLS OFF CACHE BOOL "Disable kissfft tools" FORCE)
set(KISSFFT_TEST OFF CACHE BOOL "Disable kissfft tests" FORCE)

set(KISSFFT_STATIC ON CACHE BOOL "Build kissfft as a static library" FORCE)
add_subdirectory(kissfft)  
include_directories(kissfft)  

# Add any user requested libraries
target_link_libraries(Projeto_Final_U7LFdP 
        hardware_i2c
        hardware_dma
        hardware_pio
        hardware_adc
        hardware_timer
        hardware_clocks
        hardware_pwm
        kissfft
        
        )

pico_add_extra_outputs(Projeto_Final_U7LFdP)

