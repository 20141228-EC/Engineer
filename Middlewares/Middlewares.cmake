# Add FreeRTOS Kernel
add_subdirectory(Middlewares/FreeRTOS-Kernel)
target_link_libraries(FIRMWARE.elf freertos_kernel)

# Add SEGGER RTT
add_subdirectory(Middlewares/SEGGER_RTT)
target_link_libraries(FIRMWARE.elf segger_rtt)

# Add CustomData Protobuf
add_subdirectory(Middlewares/CustomData)
target_link_libraries(FIRMWARE.elf custom_data)

