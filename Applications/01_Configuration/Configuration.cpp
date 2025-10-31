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

namespace my_engineer{

/**
 * @brief Application的入口函数
 * @return None
 */
void ApplicationEntryPoint(){

    // 调用各配置初始化函数
    // 顺序一定不要搞错啊！！
    InitAllInterface();
    InitAllDevice();
    InitAllModule();
    InitProcess();
    
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
