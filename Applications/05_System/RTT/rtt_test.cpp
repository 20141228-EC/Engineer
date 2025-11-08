/**
 * @file rtt_test.cpp
 * @brief SEGGER RTT测试和调试功能实现
 * @author Copilot
 * @date 2025-08-26
 */
#include "Core.hpp"
#include "SEGGER_RTT.h"
// #include <stdio.h>
// #include <stdarg.h>

namespace my_engineer {

/* RTT缓冲区定义 */
static char rtt_buffer_up_0[RTT_BUFFER_SIZE_UP_0];   // 主终端上行缓冲区
static char rtt_buffer_down_0[RTT_BUFFER_SIZE_DOWN_0]; // 主终端下行缓冲区  
static char rtt_buffer_up_1[RTT_BUFFER_SIZE_UP_1];   // 调试通道上行缓冲区
static char rtt_buffer_up_2[RTT_BUFFER_SIZE_UP_2];   // 错误通道上行缓冲区

/**
 * @brief 配置RTT通道
 */
void RTT_ConfigureChannels(void)
{
    // 配置通道0 - 主终端
    SEGGER_RTT_ConfigUpBuffer(RTT_CHANNEL_TERMINAL, RTT_CHANNEL_NAME_0, 
                              rtt_buffer_up_0, RTT_BUFFER_SIZE_UP_0, RTT_MODE_DEFAULT);
    SEGGER_RTT_ConfigDownBuffer(RTT_CHANNEL_TERMINAL, RTT_CHANNEL_NAME_0, 
                                rtt_buffer_down_0, RTT_BUFFER_SIZE_DOWN_0, RTT_MODE_DEFAULT);
    
    // 配置通道1 - 调试
    SEGGER_RTT_ConfigUpBuffer(RTT_CHANNEL_DEBUG, RTT_CHANNEL_NAME_1, 
                              rtt_buffer_up_1, RTT_BUFFER_SIZE_UP_1, RTT_MODE_DEFAULT);
    
    // 配置通道2 - 错误
    SEGGER_RTT_ConfigUpBuffer(RTT_CHANNEL_ERROR, RTT_CHANNEL_NAME_2, 
                              rtt_buffer_up_2, RTT_BUFFER_SIZE_UP_2, RTT_MODE_DEFAULT);
}

/**
 * @brief 设置RTT通道模式
 * @note SEGGER RTT官方API没有提供单独设置模式的函数，如需更改模式请重新调用SEGGER_RTT_ConfigUpBuffer。
 */
void RTT_SetChannelMode(unsigned BufferIndex, unsigned Mode)
{
    // SEGGER_RTT_SetModeUpBuffer(BufferIndex, Mode); // 此API不存在
    // 如需更改模式，请重新调用SEGGER_RTT_ConfigUpBuffer
}

/**
 * @brief 打印RTT通道信息
 */
void RTT_PrintChannelInfo(void)
{
    RTT_INFO("=== RTT Channel Configuration ===\n");
    RTT_INFO("Channel 0 (%s): UP=%d bytes, DOWN=%d bytes\n", 
             RTT_CHANNEL_NAME_0, RTT_BUFFER_SIZE_UP_0, RTT_BUFFER_SIZE_DOWN_0);
    RTT_INFO("Channel 1 (%s): UP=%d bytes\n", 
             RTT_CHANNEL_NAME_1, RTT_BUFFER_SIZE_UP_1);
    RTT_INFO("Channel 2 (%s): UP=%d bytes\n", 
             RTT_CHANNEL_NAME_2, RTT_BUFFER_SIZE_UP_2);
    RTT_INFO("Max UP buffers: %d, Max DOWN buffers: %d\n", 
             SEGGER_RTT_MAX_NUM_UP_BUFFERS, SEGGER_RTT_MAX_NUM_DOWN_BUFFERS);
    RTT_INFO("================================\n");
}

/**
 * @brief RTT初始化
 */
void RTT_Init(void)
{
    // 配置所有RTT通道
    RTT_ConfigureChannels();
    
    // 发送初始化信息
    RTT_INFO("RTT Initialized for STM32H723\n");
    RTT_INFO("System Clock: %lu Hz\n", SystemCoreClock);
    RTT_INFO("Compile Date: %s %s\n", __DATE__, __TIME__);
    
    // 打印通道配置信息
    RTT_PrintChannelInfo();
}

/**
 * @brief 基础RTT功能测试
 */
void RTT_Test_Basic(void)
{
    RTT_INFO("=== RTT Basic Test ===\n");
    
    // 测试字符串输出
    SEGGER_RTT_WriteString(RTT_CHANNEL_TERMINAL, "Hello from STM32H723!\n");
    
    // 测试格式化输出
    SEGGER_RTT_printf(RTT_CHANNEL_TERMINAL, "Counter: %d\n", 42);
    SEGGER_RTT_printf(RTT_CHANNEL_TERMINAL, "Hex: 0x%08X\n", 0x12345678);
    SEGGER_RTT_printf(RTT_CHANNEL_TERMINAL, "Float: %.2f\n", 3.14159f);
    
    // 测试不同日志级别
    RTT_ERROR("This is an error message\n");
    RTT_WARN("This is a warning message\n");
    RTT_INFO("This is an info message\n");
    RTT_DEBUG("This is a debug message\n");
    
    // 测试带颜色的输出
    RTT_ERROR_COLOR("This is a colored error message\n");
    RTT_WARN_COLOR("This is a colored warning message\n");
    RTT_INFO_COLOR("This is a colored info message\n");
    
    // 测试带时间戳的输出
    RTT_LOG_TS("Message with timestamp\n");
    
    RTT_INFO("Basic test completed!\n\n");
}

/**
 * @brief RTT性能测试
 */
void RTT_Test_Performance(void)
{
    RTT_INFO("=== RTT Performance Test ===\n");
    
    uint32_t start_tick = HAL_GetTick();
    
    // 连续输出测试
    for(int i = 0; i < 100; i++) {
        SEGGER_RTT_printf(RTT_CHANNEL_TERMINAL, "Test %03d: Data=0x%08X\n", i, i * 0x12345);
    }
    
    uint32_t end_tick = HAL_GetTick();
    RTT_INFO("100 prints took %lu ms\n", end_tick - start_tick);
    
    // 大块数据传输测试
    char large_buffer[256];
    for(int i = 0; i < sizeof(large_buffer) - 1; i++) {
        large_buffer[i] = 'A' + (i % 26);
    }
    large_buffer[sizeof(large_buffer) - 1] = '\0';
    
    start_tick = HAL_GetTick();
    SEGGER_RTT_WriteString(RTT_CHANNEL_TERMINAL, large_buffer);
    end_tick = HAL_GetTick();
    
    RTT_INFO("\n256 byte transfer took %lu ms\n", end_tick - start_tick);
    RTT_INFO("Performance test completed!\n\n");
}

/**
 * @brief RTT数据类型测试
 */
void RTT_Test_DataTypes(void)
{
    RTT_INFO("=== RTT Data Types Test ===\n");
    
    // 整数类型
    int8_t  i8 = -128;
    uint8_t u8 = 255;
    int16_t i16 = -32768;
    uint16_t u16 = 65535;
    int32_t i32 = -2147483648;
    uint32_t u32 = 4294967295U;
    
    RTT_INFO("int8_t:  %d\n", i8);
    RTT_INFO("uint8_t: %u\n", u8);
    RTT_INFO("int16_t: %d\n", i16);
    RTT_INFO("uint16_t: %u\n", u16);
    RTT_INFO("int32_t: %ld\n", i32);
    RTT_INFO("uint32_t: %lu\n", u32);
    
    // 浮点类型
    float f = 3.14159f;
    double d = 2.718281828459045;
    
    RTT_INFO("float:  %.6f\n", f);
    RTT_INFO("double: %.15f\n", d);
    
    // 字符串
    char device[] = "STM32H723VGT6";
    RTT_INFO("Device: %s\n", device);
    
    // 十六进制
    uint32_t hex_data = 0xDEADBEEF;
    RTT_INFO("Hex: 0x%08X\n", hex_data);
    
    RTT_INFO("Data types test completed!\n\n");
}

/**
 * @brief RTT Printf包装函数
 */
void RTT_Printf(const char* format, ...)
{
    char buffer[256];  // 定义缓冲区
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len > 0) {
        SEGGER_RTT_Write(RTT_CHANNEL_TERMINAL, buffer, len);
    }
}

/**
 * @brief 系统printf重定向到RTT (可选)
 * 取消注释下面的代码可以将printf重定向到RTT
 */
/*
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
    SEGGER_RTT_Write(RTT_CHANNEL_TERMINAL, ptr, len);
    return len;
}
#endif
*/
}