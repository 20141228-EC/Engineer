/**
 * @file RTTConf.hpp
 * @brief SEGGER RTT 配置文件
 * @author Copilot  
 * @date 2025-08-26
 * 
 * 此文件定义了 SEGGER RTT 的应用层配置参数
 * 避免与 SEGGER RTT 库本身的定义冲突
 */

#ifndef RTTCONF_HPP
#define RTTCONF_HPP

// 只在未定义时才定义这些配置，避免重定义
#ifndef SEGGER_RTT_MAX_NUM_UP_BUFFERS
#define SEGGER_RTT_MAX_NUM_UP_BUFFERS     3      // 上行缓冲区最大数量
#endif

#ifndef SEGGER_RTT_MAX_NUM_DOWN_BUFFERS
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS   1      // 下行缓冲区最大数量
#endif

#ifndef SEGGER_RTT_PRINTF_BUFFER_SIZE
#define SEGGER_RTT_PRINTF_BUFFER_SIZE     256    // printf缓冲区大小
#endif

#ifndef SEGGER_RTT_MEMCPY_USE_BYTELOOP
#define SEGGER_RTT_MEMCPY_USE_BYTELOOP    0      // 使用memcpy而不是字节循环
#endif

// 应用层自定义配置（不会与SEGGER RTT冲突）
#define RTT_APP_BUFFER_SIZE_UP_0          1024   // 应用层通道0上行缓冲区大小
#define RTT_APP_BUFFER_SIZE_DOWN_0        32     // 应用层通道0下行缓冲区大小
#define RTT_APP_BUFFER_SIZE_UP_1          512    // 应用层通道1上行缓冲区大小
#define RTT_APP_BUFFER_SIZE_UP_2          256    // 应用层通道2上行缓冲区大小

// 应用层RTT通道定义
#define RTT_APP_CHANNEL_TERMINAL          0      // 主终端通道
#define RTT_APP_CHANNEL_DEBUG             1      // 调试通道  
#define RTT_APP_CHANNEL_ERROR             2      // 错误通道

// 应用层RTT通道名称
#define RTT_APP_CHANNEL_NAME_0            "Terminal"
#define RTT_APP_CHANNEL_NAME_1            "Debug"
#define RTT_APP_CHANNEL_NAME_2            "Error"

// RTT颜色控制字符（应用层定义，不与SEGGER RTT冲突）
#define RTT_COLOR_RESET                   "\x1B[0m"         // 重置
#define RTT_COLOR_CLEAR                   "\x1B[2J"         // 清屏
#define RTT_COLOR_BLACK                   "\x1B[2;30m"      // 黑色文本
#define RTT_COLOR_RED                     "\x1B[2;31m"      // 红色文本
#define RTT_COLOR_GREEN                   "\x1B[2;32m"      // 绿色文本
#define RTT_COLOR_YELLOW                  "\x1B[2;33m"      // 黄色文本
#define RTT_COLOR_BLUE                    "\x1B[2;34m"      // 蓝色文本
#define RTT_COLOR_MAGENTA                 "\x1B[2;35m"      // 品红色文本
#define RTT_COLOR_CYAN                    "\x1B[2;36m"      // 青色文本
#define RTT_COLOR_WHITE                   "\x1B[2;37m"      // 白色文本

#define RTT_COLOR_BRIGHT_BLACK            "\x1B[1;30m"      // 亮黑色文本
#define RTT_COLOR_BRIGHT_RED              "\x1B[1;31m"      // 亮红色文本
#define RTT_COLOR_BRIGHT_GREEN            "\x1B[1;32m"      // 亮绿色文本
#define RTT_COLOR_BRIGHT_YELLOW           "\x1B[1;33m"      // 亮黄色文本
#define RTT_COLOR_BRIGHT_BLUE             "\x1B[1;34m"      // 亮蓝色文本
#define RTT_COLOR_BRIGHT_MAGENTA          "\x1B[1;35m"      // 亮品红色文本
#define RTT_COLOR_BRIGHT_CYAN             "\x1B[1;36m"      // 亮青色文本
#define RTT_COLOR_BRIGHT_WHITE            "\x1B[1;37m"      // 亮白色文本

#endif /* RTTCONF_HPP */
