/**
 * @file proc_chassis.cpp
 * @author Fish_Joe (2328339747@qq.com)
 * @brief 底盘任务
 * @version 1.0
 * @date 2024-11-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "mod_chassis.hpp"

namespace my_engineer {

volatile uint32_t g_chassis_loop_dt_ms = 0;
volatile uint32_t g_chassis_loop_dt_max_ms = 0;
volatile uint32_t g_chassis_loop_overrun_cnt = 0;

/**
 * @brief 创建底盘任务
 * 
 * @param argument 
 */
void CModChassis::StartChassisModuleTask(void *argument) {

    // 要求参数为CModChassis类的实例，如果传入为空则删除任务并返回
    if (argument == nullptr) proc_return();

    // 类型转换
    auto &chassis = *static_cast<CModChassis *>(argument);
    TickType_t lastWakeTime = xTaskGetTickCount();
    constexpr TickType_t ctrlPeriodTicks = pdMS_TO_TICKS(1);
    uint32_t lastLoopTickMs = HAL_GetTick();

    // 任务循环
    while (true) {
        // FSM
        switch (chassis.Module_FSMFlag_) {          ///<这里是静态的成员函数没有隐式的this指针，所以要用对应的模快来访问
            
            case FSM_RESET: {

                chassis.chassisInfo.isModuleAvailable = false;
                chassis.comWheelset_.StopComponent();

                proc_waitMs(20);
                lastWakeTime = xTaskGetTickCount();
                lastLoopTickMs = HAL_GetTick();
                continue; // 跳过下面的代码，直接进入下一次循环
            }

            case FSM_INIT: {

                chassis.comWheelset_.StartComponent();

                chassis.chassisCmd = SChassisCmd();
                chassis.chassisInfo.isModuleAvailable = true;
                chassis.Module_FSMFlag_ = FSM_CTRL;
                chassis.moduleStatus = APP_OK;
                lastWakeTime = xTaskGetTickCount();
                lastLoopTickMs = HAL_GetTick();

                continue;
            }

            case FSM_CTRL: {

                // 限制控制量
                chassis.RestrictChassisCommand_();

                // 从上层更新底盘控制量
                // 乘以一个较大值方便调参
                chassis.comWheelset_.wheelsetCmd.speed_X = chassis.chassisCmd.speed_X * 6.06 * 5;
                chassis.comWheelset_.wheelsetCmd.speed_Y = chassis.chassisCmd.speed_Y * 6.06 * 5;
                chassis.comWheelset_.wheelsetCmd.speed_W = chassis.chassisCmd.speed_W * 6.06 * 5;

                uint32_t nowMs = HAL_GetTick();
                g_chassis_loop_dt_ms = nowMs - lastLoopTickMs;
                if (g_chassis_loop_dt_ms > g_chassis_loop_dt_max_ms) {
                    g_chassis_loop_dt_max_ms = g_chassis_loop_dt_ms;
                }
                if (g_chassis_loop_dt_ms > 2U) {
                    g_chassis_loop_overrun_cnt++;
                }
                lastLoopTickMs = nowMs;

                vTaskDelayUntil(&lastWakeTime, ctrlPeriodTicks); // 1000Hz(绝对周期)
                continue;
            }

            default: { chassis.StopModule(); }

        }
    }

    // 任务退出
    chassis.moduleTaskHandle = nullptr;
    proc_return();
}

}   // namespace my_engineer
