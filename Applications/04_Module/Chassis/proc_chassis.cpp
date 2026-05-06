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

                // 轮组电机掉线后尝试重连：仅对“曾在线后离线”的情况触发，避免上电初期反复重置。
                {
                    static bool wheelMotorHadOnline[8] = {false, false, false, false, false, false, false, false};
                    static uint32_t lastReconnectTryMs = 0U;
                    constexpr uint32_t kReconnectTryIntervalMs = 350U;

                    bool needReconnect = false;
                    for (int i = 0; i < 4; i++) {
                        auto *driveMtr = chassis.comWheelset_.motor[i];
                        if (driveMtr != nullptr) {
                            const bool online = driveMtr->IsMotorOnline();
                            if (online) {
                                wheelMotorHadOnline[i] = true;
                            } else if (wheelMotorHadOnline[i]) {
                                needReconnect = true;
                            }
                        }

                        auto *steerMtr = chassis.comWheelset_.steerMotor[i];
                        if (steerMtr != nullptr) {
                            const bool online = steerMtr->IsMotorOnline();
                            if (online) {
                                wheelMotorHadOnline[4 + i] = true;
                            } else if (wheelMotorHadOnline[4 + i]) {
                                needReconnect = true;
                            }
                        }
                    }

                    if (needReconnect) {
                        const uint32_t nowMs = HAL_GetTick();
                        if ((nowMs - lastReconnectTryMs) >= kReconnectTryIntervalMs) {
                            lastReconnectTryMs = nowMs;
                            chassis.Module_FSMFlag_ = FSM_INIT;
                        }
                        vTaskDelayUntil(&lastWakeTime, ctrlPeriodTicks);
                        continue;
                    }
                }

                // 限制控制量
                chassis.RestrictChassisCommand_();

                // 从上层更新底盘控制量
                // 乘以一个较大值方便调参
                chassis.comWheelset_.wheelsetCmd.speed_X = chassis.chassisCmd.speed_X * 80;
                chassis.comWheelset_.wheelsetCmd.speed_Y = chassis.chassisCmd.speed_Y * 80;
                chassis.comWheelset_.wheelsetCmd.speed_W = chassis.chassisCmd.speed_W * 40;

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
