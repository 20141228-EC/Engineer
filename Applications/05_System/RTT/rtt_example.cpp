/**
 * @file rtt_example.cpp
 * @brief RTT使用示例
 * @author Copilot
 * @date 2025-08-26
 */
#include "Core.hpp"
namespace my_engineer {
/**
 * @brief 在main函数中初始化和测试RTT
 * 
 * 将以下代码添加到您的main.cpp中：
 */

void RTT_Example_Init(void)
{
    /* 1. 在系统初始化后调用RTT初始化 */
    // HAL_Init();
    // SystemClock_Config();
    // ... 其他初始化 ...
    
    RTT_Init();  // RTT初始化
    
    /* 2. 运行基础测试 */
    RTT_Test_Basic();
    RTT_Test_DataTypes();
    RTT_Test_Performance();
}

/**
 * @brief 在主循环中使用RTT
 */
void RTT_Example_Loop(void)
{
    static uint32_t counter = 0;
    static uint32_t last_tick = 0;
    
    uint32_t current_tick = HAL_GetTick();
    
    // 每秒输出一次状态信息
    if (current_tick - last_tick >= 1000) {
        last_tick = current_tick;
        
        RTT_INFO("System running... Counter: %lu, Time: %lu ms\n", 
                 counter++, current_tick);
        
        // 输出系统状态
        RTT_INFO("Free Heap: %u bytes\n", xPortGetFreeHeapSize());
        RTT_INFO("Stack High Water: %u words\n", uxTaskGetStackHighWaterMark(NULL));
    }
}

/**
 * @brief 在任务中使用RTT进行调试
 */
void RTT_Example_Task(void)
{
    RTT_INFO("Task started: %s\n", pcTaskGetName(NULL));
    
    while(1) {
        // 任务逻辑...
        
        // 调试输出
        RTT_DEBUG("Task %s executing at tick %lu\n", 
                  pcTaskGetName(NULL), xTaskGetTickCount());
        
        vTaskDelay(pdMS_TO_TICKS(5000));  // 5秒延时
    }
}

/**
 * @brief 错误处理中使用RTT
 */
void RTT_Example_ErrorHandler(const char* function, int line, const char* error)
{
    RTT_ERROR("Error in %s:%d - %s\n", function, line, error);
    RTT_ERROR("System will halt!\n");
    
    // 输出系统状态用于调试
    RTT_ERROR("Current tick: %lu\n", HAL_GetTick());
    RTT_ERROR("Free heap: %u\n", xPortGetFreeHeapSize());
    
    while(1) {
        // 系统挂起
    }
}

/**
 * @brief 在中断中使用RTT (注意: 要保持简短)
 */
void RTT_Example_IRQ(void)
{
    static uint32_t irq_count = 0;
    
    // 在中断中应该避免复杂的RTT操作
    // 只适合简单的计数或状态记录
    irq_count++;
    
    // 每1000次中断输出一次 (减少输出频率)
    if (irq_count % 1000 == 0) {
        RTT_INFO("IRQ count: %lu\n", irq_count);
    }
}

/**
 * @brief 数据监控示例
 */
void RTT_Example_DataMonitor(void)
{
    // 模拟传感器数据
    float temperature = 25.5f;
    float voltage = 3.3f;
    uint16_t adc_value = 2048;
    
    RTT_INFO("=== System Monitor ===\n");
    RTT_INFO("Temperature: %.2f°C\n", temperature);
    RTT_INFO("Voltage: %.3fV\n", voltage);
    RTT_INFO("ADC: %u (0x%04X)\n", adc_value, adc_value);
    RTT_INFO("CPU Usage: 25%%\n");  // 注意%%转义
    RTT_INFO("======================\n");
}
}  // namespace my_engineer