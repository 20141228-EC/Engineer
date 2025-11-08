cmake_minimum_required(VERSION 3.14)

# SEGGER RTT Library Configuration

# Collect all RTT source files in current directory
file(GLOB RTT_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
)

# Collect assembly files
file(GLOB RTT_ASM_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.S"
)

# Create SEGGER RTT static library
add_library(segger_rtt STATIC 
    ${RTT_SOURCES}
    ${RTT_ASM_SOURCES}
)

# Set include directories for RTT library
target_include_directories(segger_rtt PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Compiler specific settings for assembly files
if(RTT_ASM_SOURCES)
    set_property(SOURCE ${RTT_ASM_SOURCES} PROPERTY LANGUAGE ASM)
endif()