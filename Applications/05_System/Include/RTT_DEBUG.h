/**
 * @file rtt_test.h
 * @brief SEGGER RTT测试和调试功能
 * @author Copilot
 * @date 2025-08-26
 */

#ifndef RTT_DEBUG_H
#define RTT_DEBUG_H

#include "SEGGER_RTT.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* RTT配置参数 */
#define RTT_TEST_ENABLED        1

/* RTT通道配置 - 使用应用层定义 */
#define RTT_CHANNEL_TERMINAL            0       // 主终端通道
#define RTT_CHANNEL_DEBUG               1       // 调试通道
#define RTT_CHANNEL_ERROR               2       // 错误通道

/* RTT缓冲区大小配置 - 使用应用层定义 */
#define RTT_BUFFER_SIZE_UP_Terminal     1024    // 上行通道0缓冲区大小 (主终端)
#define RTT_BUFFER_SIZE_DOWN_Terminal   32      // 下行通道0缓冲区大小
#define RTT_BUFFER_SIZE_UP_Debug        512     // 上行通道1缓冲区大小 (调试)
#define RTT_BUFFER_SIZE_UP_Error        256     // 上行通道2缓冲区大小 (错误)

// 应用层RTT通道名称
#define RTT_CHANNEL_NAME_0              "Terminal"
#define RTT_CHANNEL_NAME_1              "Debug"
#define RTT_CHANNEL_NAME_2              "Error"

/* RTT模式配置 */
#define RTT_MODE_DEFAULT                SEGGER_RTT_MODE_NO_BLOCK_SKIP   // 默认模式：非阻塞跳过

/* RTT缓冲区定义 */
static char rtt_buffer_up_Terminal[RTT_BUFFER_SIZE_UP_Terminal];     // 主终端上行缓冲区
static char rtt_buffer_down_Terminal[RTT_BUFFER_SIZE_DOWN_Terminal]; // 主终端下行缓冲区
static char rtt_buffer_up_Debug[RTT_BUFFER_SIZE_UP_Debug];     // 调试通道上行缓冲区
static char rtt_buffer_up_Error[RTT_BUFFER_SIZE_UP_Error];     // 错误通道上行缓冲区


/*----------------------------------------------------------------------------------------------------------------*/
// 修改日志核心: 解决 %?lu / %?f 以及行被截断(多次分段写导致与其他日志互串)问题
static inline void RTT_LogCore(unsigned ch, const char* levelColor, const char* level,
                               const char* fmt, ...)
{
    #ifndef RTT_CTRL_TEXT_WHITE
    #define RTT_CTRL_TEXT_WHITE  "\x1B[37m"
    #endif
    #ifndef RTT_CTRL_TEXT_BLUE
    #define RTT_CTRL_TEXT_BLUE   "\x1B[34m"
    #endif

    char lineBuf[1024];
    char* out = lineBuf;
    char* end = lineBuf + sizeof(lineBuf) - 1;

    // 时间戳(白) + 级别(彩色)
    int w = snprintf(out,(size_t)(end-out), "%s[%lu]%s[%s]%s ",
                     RTT_CTRL_TEXT_WHITE,
                     (unsigned long)HAL_GetTick(),
                     levelColor, level,
                     RTT_CTRL_TEXT_WHITE);
    if (w<0) w=0; if (w>(int)(end-out)) w=(int)(end-out);
    out += w;

    va_list args;
    va_start(args, fmt);
    const char* p = fmt;

    auto appendNum = [&](const char* numStr){
        int nw = snprintf(out,(size_t)(end-out), "%s%s%s",
                          RTT_CTRL_TEXT_BLUE, numStr, RTT_CTRL_TEXT_WHITE);
        if (nw>0) { if (nw>(int)(end-out)) nw=(int)(end-out); out += nw; }
    };

    while (*p && out < end) {
        if (*p == '%') {
            ++p;
            bool longFlag=false;
            if (*p=='l'){ longFlag=true; ++p; }
            if (!*p) break;
            switch (*p) {
            case '%': *out++='%'; break;
            case 'c': {
                int v=va_arg(args,int); *out++=(char)v;
            } break;
            case 's': {
                const char* s=va_arg(args,const char*);
                if(!s) s="(null)";
                // 普通字符串内的数字也染色
                while(*s && out<end){
                    if (*s>='0' && *s<='9') {
                        const char* start=s;
                        while(*s>='0' && *s<='9') ++s;
                        char tmp[32]; size_t len = (size_t)(s-start);
                        if (len>=sizeof(tmp)) len=sizeof(tmp)-1;
                        memcpy(tmp,start,len); tmp[len]=0;
                        appendNum(tmp);
                    } else {
                        *out++=*s++;
                    }
                }
            } break;
            case 'd':
            case 'u':
            case 'x':
            case 'X': {
                unsigned long v = longFlag
                    ? va_arg(args,unsigned long)
                    : ((*p=='d')? (unsigned long)va_arg(args,int)
                                : (unsigned long)va_arg(args,unsigned));
                char num[32];
                if (*p=='d') snprintf(num,sizeof(num), "%ld", (long)v);
                else if (*p=='u') snprintf(num,sizeof(num), "%lu", v);
                else if (*p=='x') snprintf(num,sizeof(num), "%lx", v);
                else snprintf(num,sizeof(num), "%lX", v);
                appendNum(num);
            } break;
            case 'f': {
                double dv=va_arg(args,double);
                char num[48];
                snprintf(num,sizeof(num), "%f", dv);
                appendNum(num);
            } break;
            case 'V': {
                uint32_t v=va_arg(args,uint32_t);
                char bin[34]; bin[0]='0'; bin[1]='b';
                for (int i=0;i<32;++i) bin[2+i]=(v&(1u<<(31-i)))?'1':'0';
                bin[33]=0;
                char num[96];
                snprintf(num,sizeof(num),"dec=%lu hex=0x%08lX bin=%s",
                         (unsigned long)v,(unsigned long)v,bin);
                // 对其中数字分段染色
                const char* s=num;
                while(*s && out<end){
                    if (*s>='0' && *s<='9') {
                        const char* st=s;
                        while((*s>='0'&&*s<='9')||(*s>='A'&&*s<='F')||(*s>='a'&&*s<='f')||*s=='x'||*s=='X')
                            ++s;
                        char tmp[64]; size_t len=(size_t)(s-st);
                        if(len>=sizeof(tmp)) len=sizeof(tmp)-1;
                        memcpy(tmp,st,len); tmp[len]=0;
                        appendNum(tmp);
                    } else {
                        *out++=*s++;
                    }
                }
            } break;
            case 'F': {
                double dv=va_arg(args,double);
                float fv=(float)dv;
                union{float f; uint32_t u;} conv; conv.f=fv;
                char bin[34]; bin[0]='0'; bin[1]='b';
                for (int i=0;i<32;++i) bin[2+i]=(conv.u&(1u<<(31-i)))?'1':'0';
                bin[33]=0;
                char num[96];
                snprintf(num,sizeof(num),"float=%f hex=0x%08lX bin=%s",
                         (double)fv,(unsigned long)conv.u,bin);
                const char* s=num;
                while(*s && out<end){
                    if (*s>='0' && *s<='9') {
                        const char* st=s;
                        while((*s>='0'&&*s<='9')||(*s=='.')) ++s;
                        char tmp[48]; size_t len=(size_t)(s-st);
                        if(len>=sizeof(tmp)) len=sizeof(tmp)-1;
                        memcpy(tmp,st,len); tmp[len]=0;
                        appendNum(tmp);
                    } else {
                        *out++=*s++;
                    }
                }
            } break;
            default: // 未识别保持原样
                *out++='%';
                if (longFlag && out<end) *out++='l';
                *out++=*p;
                break;
            }
            ++p;
        } else {
            if (*p>='0' && *p<='9') {
                const char* st=p;
                while(*p>='0' && *p<='9') ++p;
                char tmp[32]; size_t len=(size_t)(p-st);
                if(len>=sizeof(tmp)) len=sizeof(tmp)-1;
                memcpy(tmp,st,len); tmp[len]=0;
                appendNum(tmp);
            } else {
                *out++=*p++;
            }
        }
    }

    // 复位
    snprintf(out,(size_t)(end-out), "%s\r\n", RTT_CTRL_RESET);
    *end = '\0';

    SEGGER_RTT_WriteString(ch, lineBuf);
}

// 使用: levelColor 传 RTT_CTRL_TEXT_RED/YELLOW/GREEN; 普通文本/数字自动处理
#define RTT_LOG_ERROR(fmt, ...) RTT_LogCore(RTT_CHANNEL_TERMINAL, RTT_CTRL_TEXT_RED,    "ERROR", fmt, ##__VA_ARGS__)
#define RTT_LOG_WARN(fmt, ...)  RTT_LogCore(RTT_CHANNEL_TERMINAL, RTT_CTRL_TEXT_YELLOW, "WARN",  fmt, ##__VA_ARGS__)
#define RTT_LOG_INFO(fmt, ...)  RTT_LogCore(RTT_CHANNEL_TERMINAL, RTT_CTRL_TEXT_GREEN,  "INFO",  fmt, ##__VA_ARGS__)

/**
 * @brief 配置RTT通道
 */
static inline void RTT_ConfigureChannels(void)
{
    // 配置通道0 - 主终端
    SEGGER_RTT_ConfigUpBuffer(RTT_CHANNEL_TERMINAL, RTT_CHANNEL_NAME_0,
                              rtt_buffer_up_Terminal, RTT_BUFFER_SIZE_UP_Terminal, RTT_MODE_DEFAULT);
    SEGGER_RTT_ConfigDownBuffer(RTT_CHANNEL_TERMINAL, RTT_CHANNEL_NAME_0,
                                rtt_buffer_down_Terminal, RTT_BUFFER_SIZE_DOWN_Terminal, RTT_MODE_DEFAULT);

    // 配置通道1 - 调试
    SEGGER_RTT_ConfigUpBuffer(RTT_CHANNEL_DEBUG, RTT_CHANNEL_NAME_1,
                              rtt_buffer_up_Debug, RTT_BUFFER_SIZE_UP_Debug, RTT_MODE_DEFAULT);
    // 配置通道2 - 错误
    SEGGER_RTT_ConfigUpBuffer(RTT_CHANNEL_ERROR, RTT_CHANNEL_NAME_2,
                              rtt_buffer_up_Error, RTT_BUFFER_SIZE_UP_Error, RTT_MODE_DEFAULT);
}

/**
 * @brief 打印RTT通道信息
 */
static inline void RTT_PrintChannelInfo(void)
{
    RTT_LOG_INFO("=== RTT Channel Configuration ===\n");
    RTT_LOG_INFO("Channel 0 (%s): UP=%d bytes, DOWN=%d bytes\n",
             RTT_CHANNEL_NAME_0, RTT_BUFFER_SIZE_UP_Terminal, RTT_BUFFER_SIZE_DOWN_Terminal);
    RTT_LOG_INFO("Channel 1 (%s): UP=%d bytes\n",
             RTT_CHANNEL_NAME_1, RTT_BUFFER_SIZE_UP_Debug);
    RTT_LOG_INFO("Channel 2 (%s): UP=%d bytes\n",
             RTT_CHANNEL_NAME_2, RTT_BUFFER_SIZE_UP_Error);
    RTT_LOG_INFO("Max UP buffers: %d, Max DOWN buffers: %d\n",
             SEGGER_RTT_MAX_NUM_UP_BUFFERS, SEGGER_RTT_MAX_NUM_DOWN_BUFFERS);
    RTT_LOG_INFO("================================\n");
}

/**
 * @brief RTT初始化
 */
static inline void RTT_Init(void)
{
    SEGGER_RTT_Init();
    // 配置所有RTT通道
    RTT_ConfigureChannels();

    // 发送初始化信息
    RTT_LOG_INFO("RTT Initialized for STM32H723\n");
    RTT_LOG_INFO("System Clock: %lu Hz\n", SystemCoreClock);
    RTT_LOG_INFO("Compile Date: %s %s\n", __DATE__, __TIME__);

    // 打印通道配置信息
    RTT_PrintChannelInfo();
}

/**
 * @brief 基础RTT功能测试
 */
static inline void RTT_Test_Basic(void)
{
    RTT_LOG_INFO("=== RTT Basic Test ===\n");

    RTT_LOG_ERROR("This is an error message\n");
    RTT_LOG_WARN("This is a warning message\n");
    RTT_LOG_INFO("This is an info message\n");
}

/**
 * @brief 内存转储
 */
static inline void RTT_MemDump(const void* buf, size_t len) {
    const uint8_t* p = (const uint8_t*)buf;
    size_t offset = 0;
    while (offset < len) {
        RTT_LOG_INFO("DUMP %04u: ", (unsigned)offset);
        for (int i = 0; i < 16; ++i) {
            if (offset + i < len) {
                RTT_LOG_INFO("%02X ", p[offset + i]);
            } else {
                RTT_LOG_INFO("   ");
            }
        }
        RTT_LOG_INFO("| ");
        for (int i = 0; i < 16 && offset + i < len; ++i) {
            uint8_t c = p[offset + i];
            RTT_LOG_INFO("%c", (c >= 32 && c <= 126) ? c : '.');
        }
        offset += 16;
    }
}

/**
 * @brief 数据类型与结构体测试
 */
static inline void RTT_Test_DataTypes(void)
{
    RTT_LOG_INFO("=== Type Test Begin ===");

    // 基本整数
    int8_t  i8  = -128;
    uint8_t u8  = 255;
    int16_t i16 = -32768;
    uint16_t u16 = 65535;
    int32_t i32 = -2147483647 - 1;
    uint32_t u32 = 4294967295u;
    int64_t i64 = (int64_t)0x8000000000000000ULL;
    uint64_t u64 = 0xFFFFFFFFFFFFFFFFULL;

    RTT_LOG_INFO("i8=%d u8=%u", i8, u8);
    RTT_LOG_INFO("i16=%d u16=%u", i16, u16);
    RTT_LOG_INFO("i32=%d u32=%u", i32, u32);
    RTT_LOG_INFO("i64(low32)=%d u64(low32)=%u", (int)(i64 & 0xFFFFFFFF), (unsigned)(u64 & 0xFFFFFFFF));

    // 扩展格式 %V (uint32 三合一)
    RTT_LOG_INFO("u32 combo=%V", u32);
    RTT_LOG_INFO("SystemCoreClock=%V", SystemCoreClock);

    // 浮点
    float f = 3.1415926f;
    double d = 2.718281828459045;
    union { float f; uint32_t u; } fu; fu.f = f;
    RTT_LOG_INFO("float=%f raw=%V", f, fu.u);
    RTT_LOG_INFO("double(低32位raw)=%f raw32=%V", (float)d, (uint32_t)(*(uint64_t*)&d & 0xFFFFFFFFu));

    // 枚举/布尔
    enum Mode { MODE_IDLE=0, MODE_RUN=1, MODE_ERR=2 } mode = MODE_RUN;
    bool flag_true = true, flag_false = false;
    RTT_LOG_INFO("enum mode=%d boolT=%u boolF=%u", (int)mode, (unsigned)flag_true, (unsigned)flag_false);

    // 位域/结构体
    struct __attribute__((packed)) SBit {
        uint16_t a:5;
        uint16_t b:3;
        uint16_t c:8;
    } sb = { .a=0x1F, .b=0x5, .c=0xA5 };
    RTT_LOG_INFO("BitStruct a=%u b=%u c=%u", sb.a, sb.b, sb.c);

    struct Point { int16_t x; int16_t y; } pt = { .x = -123, .y = 456 };
    RTT_LOG_INFO("Point x=%d y=%d", pt.x, pt.y);

    // 联合
    union UTag {
        uint32_t u32;
        uint8_t  bytes[4];
    } uu;
    uu.u32 = 0x12345678;
    RTT_LOG_INFO("Union u32=%V b0=%u b1=%u b2=%u b3=%u", uu.u32, uu.bytes[0], uu.bytes[1], uu.bytes[2], uu.bytes[3]);

    // 指针
    void* ptr = (void*)&pt;
    RTT_LOG_INFO("Pointer(低32位)=%V", (uint32_t)((uintptr_t)ptr & 0xFFFFFFFFu));

    // 数组
    uint32_t arr[5] = {1,2,3,4,5};
    for (int i=0;i<5;++i) {
        RTT_LOG_INFO("arr[%d]=%V", i, arr[i]);
    }

    // 内存转储
    RTT_LOG_INFO("Memory dump of arr:");
    RTT_MemDump(arr, sizeof(arr));

    RTT_LOG_INFO("=== Type Test End ===");
}

/**
 * @brief 优先级测试
 */
static inline void RTT_Test_Priority(void)
{
    RTT_LOG_INFO("=== Priority Test Begin ===");
    RTT_LOG_ERROR("Error level test value=%V", 0xDEADBEEF);
    RTT_LOG_WARN("Warn level test value=%u", 1234u);
    RTT_LOG_INFO("Info level test value=%X", 0xABCD);
    for (int i=0;i<3;++i) {
        RTT_LOG_ERROR("LoopE %d", i);
        RTT_LOG_WARN("LoopW %d", i);
        RTT_LOG_INFO("LoopI %d", i);
    }
    RTT_LOG_INFO("=== Priority Test End ===");
}

/**
 * @brief 性能测试 (简化 & 使用组合格式)
 */
static inline void RTT_Test_Performance(void)
{
    RTT_LOG_INFO("=== Performance Test Begin ===");
    uint32_t start = HAL_GetTick();
    for (int i=0;i<100;++i) {
        RTT_LOG_INFO("Frame=%03d tag=%V calc=%X", i, (uint32_t)i, (unsigned)(i * 0x12345));
    }
    uint32_t mid = HAL_GetTick();
    char buf[256];
    for (int i=0;i<255;++i) buf[i] = (char)('A' + (i % 26));
    buf[255] = 0;
    RTT_LOG_INFO("BulkStart tick=%V", start);
    RTT_LOG_INFO("%s", buf);
    uint32_t end = HAL_GetTick();
    RTT_LOG_INFO("PrintLoopCost(ms)=%u BulkCost(ms)=%u Total(ms)=%u",
                 (unsigned)(mid - start),
                 (unsigned)(end - mid),
                 (unsigned)(end - start));
    RTT_LOG_INFO("=== Performance Test End ===");
}

/**
 * @brief 综合入口
 */
static inline void RTT_Test_All(void)
{
    RTT_Test_Priority();
    RTT_Test_DataTypes();
    RTT_Test_Performance();
}

#endif // RTT_DEBUG_H
