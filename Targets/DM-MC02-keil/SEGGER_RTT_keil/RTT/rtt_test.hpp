/**
 * @file rtt_test.h
 * @brief SEGGER RTT测试和调试功能
 * @author Copilot
 * @date 2025-08-26
 */

#ifndef RTT_TEST_H
#define RTT_TEST_H

#include "SEGGER_RTT.h"
#include "RTTConf.hpp"
#include "sys_common.hpp"
#include <stdint.h>

/* RTT配置参数 */
#define RTT_TEST_ENABLED        1

/* RTT通道配置 - 使用应用层定义 */
#define RTT_CHANNEL_TERMINAL    RTT_APP_CHANNEL_TERMINAL    // 主终端通道
#define RTT_CHANNEL_DEBUG       RTT_APP_CHANNEL_DEBUG       // 调试通道
#define RTT_CHANNEL_ERROR       RTT_APP_CHANNEL_ERROR       // 错误通道

/* RTT缓冲区大小配置 - 使用应用层定义 */
#define RTT_BUFFER_SIZE_UP_0    RTT_APP_BUFFER_SIZE_UP_0    // 上行通道0缓冲区大小 (主终端)
#define RTT_BUFFER_SIZE_DOWN_0  RTT_APP_BUFFER_SIZE_DOWN_0  // 下行通道0缓冲区大小
#define RTT_BUFFER_SIZE_UP_1    RTT_APP_BUFFER_SIZE_UP_1    // 上行通道1缓冲区大小 (调试)
#define RTT_BUFFER_SIZE_UP_2    RTT_APP_BUFFER_SIZE_UP_2    // 上行通道2缓冲区大小 (错误)

/* RTT模式配置 */
#define RTT_MODE_DEFAULT        SEGGER_RTT_MODE_NO_BLOCK_SKIP   // 默认模式：非阻塞跳过

/* RTT通道名称 - 使用应用层定义 */
#define RTT_CHANNEL_NAME_0      RTT_APP_CHANNEL_NAME_0
#define RTT_CHANNEL_NAME_1      RTT_APP_CHANNEL_NAME_1
#define RTT_CHANNEL_NAME_2      RTT_APP_CHANNEL_NAME_2

namespace my_engineer {

/* RTT测试函数 */
void RTT_Init(void);
void RTT_Test_Basic(void);
void RTT_Test_Performance(void);
void RTT_Test_DataTypes(void);
void RTT_Printf(const char* format, ...);

/* RTT通道管理函数 */
void RTT_ConfigureChannels(void);
void RTT_SetChannelMode(unsigned BufferIndex, unsigned Mode);
void RTT_PrintChannelInfo(void);

/* RTT调试宏 */
#if RTT_TEST_ENABLED
    #define RTT_LOG(...)        SEGGER_RTT_printf(RTT_CHANNEL_TERMINAL, __VA_ARGS__)
    #define RTT_DEBUG(...)      SEGGER_RTT_printf(RTT_CHANNEL_DEBUG, "[DEBUG] " __VA_ARGS__)
    #define RTT_ERROR(...)      SEGGER_RTT_printf(RTT_CHANNEL_ERROR, "[ERROR] " __VA_ARGS__)
    #define RTT_INFO(...)       SEGGER_RTT_printf(RTT_CHANNEL_TERMINAL, "[INFO]  " __VA_ARGS__)
    #define RTT_WARN(...)       SEGGER_RTT_printf(RTT_CHANNEL_TERMINAL, "[WARN]  " __VA_ARGS__)
    
    /* 带时间戳的调试宏 */
    #define RTT_LOG_TS(...)     do { SEGGER_RTT_printf(RTT_CHANNEL_TERMINAL, "[%lu] ", HAL_GetTick()); \
                                     SEGGER_RTT_printf(RTT_CHANNEL_TERMINAL, __VA_ARGS__); } while(0)
    
    /* 带颜色的调试宏 (支持RTT Viewer颜色) */
    #define RTT_ERROR_COLOR(...)  do { SEGGER_RTT_WriteString(RTT_CHANNEL_TERMINAL, RTT_COLOR_RED); \
                                       SEGGER_RTT_printf(RTT_CHANNEL_TERMINAL, "[ERROR] " __VA_ARGS__); \
                                       SEGGER_RTT_WriteString(RTT_CHANNEL_TERMINAL, RTT_COLOR_RESET); } while(0)
    
    #define RTT_WARN_COLOR(...)   do { SEGGER_RTT_WriteString(RTT_CHANNEL_TERMINAL, RTT_COLOR_YELLOW); \
                                       SEGGER_RTT_printf(RTT_CHANNEL_TERMINAL, "[WARN]  " __VA_ARGS__); \
                                       SEGGER_RTT_WriteString(RTT_CHANNEL_TERMINAL, RTT_COLOR_RESET); } while(0)
    
    #define RTT_INFO_COLOR(...)   do { SEGGER_RTT_WriteString(RTT_CHANNEL_TERMINAL, RTT_COLOR_GREEN); \
                                       SEGGER_RTT_printf(RTT_CHANNEL_TERMINAL, "[INFO]  " __VA_ARGS__); \
                                       SEGGER_RTT_WriteString(RTT_CHANNEL_TERMINAL, RTT_COLOR_RESET); } while(0)
#else
    #define RTT_LOG(...)
    #define RTT_DEBUG(...)
    #define RTT_ERROR(...)
    #define RTT_INFO(...)
    #define RTT_WARN(...)
    #define RTT_LOG_TS(...)
    #define RTT_ERROR_COLOR(...)
    #define RTT_WARN_COLOR(...)
    #define RTT_INFO_COLOR(...)
#endif

}  // namespace my_engineer

#endif /* RTT_TEST_H */
