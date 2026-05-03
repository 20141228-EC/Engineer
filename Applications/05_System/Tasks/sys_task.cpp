/**
 * @file sys_task.cpp
 * @author sllllr (2997708711@qq.com)
 * @brief 在这里进行所有层任务的汇总，进行集中调度
 * @version 1.0
 * @date 2026-01-11
 * 
 * @copyright Copyright (c) 2026
 * @details
 * 当然，一些层中内置的任务不会在这里进行汇总，比如BMI088的初始化任务，模块的控制任务等
 */

#include "Core.hpp"
#include "conf_CanTxNode.hpp"

extern IWDG_HandleTypeDef hiwdg1;

namespace my_engineer {

/**
 * @brief 系统层更新任务
 * @note 与其它层独立开来是因为频率不需要那么高(并非，目前为了能对应上设备层1KHz的更新频率，这里也由原来的500Hz改成了1KHz)
 */
void StartSystemUpdateTask(void *argument) {        ///<这里更新的是键鼠、裁判系统、视觉系统、esp32数据、板间通信

    while (true) {

        for (const auto &item : SystemIDMap) {
            // if(item.second->systemID != ESystemID::SYS_BOARD_LINK) {
            //     item.second->UpdateHandler_();
            // }
            item.second->UpdateHandler_();
        }

        proc_waitMs(1); // 1000Hz
    }
}

uint32_t sys_test_n = 0;

/**
 * @brief 其余层的更新任务以及系统核心更新任务（1000Hz）
 * 
 */
void StartUpdateTask(void *argument) {
	
	static uint8_t HalfTickRate = 1;
    
    // 初始化系统核心
    SystemCore.InitSystemCore();                ///<等所有模块初始化完成之后再初始化系统核心，并且是在任务创建的时候初始化

    while (true) {

        sys_test_n++;
		HalfTickRate = 1 - HalfTickRate;        
        
        // 更新所有设备
        for (const auto &item : DeviceIDMap) {
            item.second->UpdateHandler_();
        }

        // 更新所有算法
        for(const auto &item : AlgoIDMap){
            item.second->UpdateHandler_();      ///< 和设备、模块以相同频率更新
        }
		// UpdateImuEkf();

        // 更新系统核心
        SystemCore.UpdateHandler_();            ///<系统核心的更新放在设备更新之后，模块更新之前,以便模块可以使用系统核心的数据   
                                                ///<包括遥控器数据、键盘数据控制命令，自动任务的更新
        // 更新所有模块
        for (const auto &item : ModuleIDMap) {
            item.second->UpdateHandler_();
        }

        // 执行can发送
        TxNode_Can3_200.Transmit(); ///< 履带电机
		// if(HalfTickRate) {              ///<此处的作用是一个分频器，这里可以考虑用信号量控制can的负载                  
		//     TxNode_Can3_280.Transmit(); ///< 机械臂后三轴电机 500Hz
        // }
            
        TxNode_Can2_200.Transmit(); ///< 底盘轮向电机(0x201~0x204)
        TxNode_Can1_1FF.Transmit(); ///< 底盘舵向电机(0x205~0x208)
        TxNode_Can2_1FF.Transmit(); ///< 末端pitch roll和夹爪收放

        proc_waitMs(1); // 1000Hz

    }
}

/**
 * @brief 心跳任务以及看门狗
 * 
 */
void StartHeartbeatTask(void *argument) {

    while (true) {

        // 通信接口心跳
        for (const auto &item : InterfaceIDMap) {
            item.second->HeartbeatHandler_();
        }

        // 设备心跳
        for (const auto &item : DeviceIDMap) {
            item.second->HeartbeatHandler_();
        }

        // 系统心跳
        for (const auto &item : SystemIDMap) {
            item.second->HeartbeatHandler_();
        }

        // 更新系统核心心跳
        SystemCore.HeartbeatHandler_();

        // 模块心跳
        for (const auto &item : ModuleIDMap) {
            item.second->HeartbeatHandler_();
        }

        // 喂狗
        HAL_IWDG_Refresh(&hiwdg1);

        proc_waitMs(10); // 100Hz
    }
}

/**
 * @brief 监控任务,用于测试RTT的调试功能
 */
void StartMonitorTask(void *argument) {
    while(true)
    {
        // RTT_LOG_INFO("Hip L: Set=%.2f, Real=%.2f", 
        //              SystemCore.pchassis_->MIT_L.Set_Angle, 
        //              SystemCore.pchassis_->MIT_L.Angle);
        proc_waitMs(10); // 100Hz
    }
    
}

}   // namespace my_engineer
