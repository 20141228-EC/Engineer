/**
 * @file conf_process.hpp
 * @author Fish_Joe (2328339747@qq.com)
 * @brief 定义任务的优先级，以及一些常用的管理任务的宏定义
 * @version 1.0
 * @date 2024-11-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef CONF_PROCESS_HPP
#define CONF_PROCESS_HPP

#include "conf_common.hpp"

#define proc_DeviceInitTaskPriority     30             ///<专门用于陀螺仪的更新
#define proc_UpdateTaskPriority         24             ///<用于所有设备的更新
#define proc_SystemTaskPriority         20             ///<用于系统控制指令的更新
#define proc_SystemCoreTaskPriority     18             ///<用于系统核心数据的更新   （遥控器，键盘）         
#define proc_ModuleTaskPriority         16             ///<用于模块的更新
#define proc_HeartbeatTaskPriority      4              ///<用于心跳包的发送
#define proc_MonitorTaskPriority        4              ///<用于监视系统状态    

template<typename ConditionFunc>
inline int8_t wait_until_with_timeout(ConditionFunc condition, uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    vTaskDelay(pdMS_TO_TICKS(10)); // 等待10ms，避免CPU占用过高

    while (!condition()) {
        if ((xTaskGetTickCount() - start) > timeout_ticks) {
            return -1;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return 0;
}


#define proc_return() vTaskDelete(nullptr) //传递nullptr给vTaskDelete函数，删除当前任务
#define proc_waitMs(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#define proc_waitUntil(cond) do { vTaskDelay(pdMS_TO_TICKS(10)); } while (!(cond))
#define proc_waitUntilWithTimeout(cond, timeout) wait_until_with_timeout([&]() -> bool { return (cond); }, timeout)



namespace my_engineer {

    // 完成sys_task.cpp中所有任务的创建
    void InitProcess();

}   // namespace my_engineer

#endif // CONF_PROCESS_HPP
