/**
 * @file Configuration.cpp
 * @author Zoe
 * @brief 所有配置的完成由此文件提供对外入口
 * @email 2328339747@qq.com
 * @date 2024-10-30
 * 
 * @details
 */

#include "Configuration.hpp"
#include "Device.hpp"
#include "Algorithm.hpp"

namespace my_engineer{

/**
 * @brief Application的入口函数
 * @return None
 */
void ApplicationEntryPoint(){

    // 调用各配置初始化函数
    // 顺序一定不要搞错啊！！
    InitAllInterface();
    InitAllDevice(); // 初始化所有设备配置 如电机can总线 帧头 id等
    InitAllAlgo();   // 初始化所有算法，放在设备之后模块之前，使算法能用设备信息，模块能够用算法信息
    InitAllModule(); // 初始化模块PID等
    InitProcess(); // 初始化四个汇总任务
    
    Print("Application Entry Point: All configurations initialized successfully.\n");

    vTaskStartScheduler(); // 启动任务调度器
}

void Print(const char *format, ...) {
    va_list args;
    va_start(args, format);
    char buffer[512];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    auto esp32dev = static_cast<CDevESP32 *>(DeviceIDMap[EDeviceID::DEV_ESP32]);
    if (esp32dev) {
        esp32dev->ESP32_Print("print:%s", buffer);
    }

}

} // namespace my_engineer
