/******************************************************************************
 * @brief
 *
 * @file         dev_servo.hpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-04-16
 *
 * @copyright    Copyright (c) 2025
 *
 ******************************************************************************/
// MG995舵机的PWM周期为20ms，高电平脉宽为0.5ms~2.5ms，与角度对应关系为0°~180°
// TIM1配置：Prescaler=274, Period=19999 -> 1MHz / 20000 = 50Hz (20ms)
// 脉冲范围：0.5ms=500计数, 2.5ms=2500计数

#ifndef DEV_SERVO_HPP
#define DEV_SERVO_HPP

#include "dev_common.hpp"

namespace my_engineer {

/**
 * @brief 定义舵机设备类
 * 
 */
class CDevServo final: public CDevBase{
public:
	// 定义舵机初始化参数结构体
	struct SDevInitParam_Servo: public SDevInitParam_Base{
		TIM_HandleTypeDef *servoHalTimHandle = nullptr; ///< 舵机定时器句柄
		uint32_t servoTimChannel = 0; ///< 舵机定时器通道		
	};

	// 定义舵机控制命令结构体
	struct SDevServoCmd{
		bool enable = false; ///< 是否使能舵机
		float_t set_angle = 0.0f; ///< 舵机目标角度
	} servoCmd;

	CDevServo() { deviceType = EDevType::DEV_SERVO; }

	EAppStatus InitDevice(const SDevInitParam_Base *pStructInitParam) final;

private:
	// 定义舵机定时器句柄
	TIM_HandleTypeDef *servoHalTimHandle_ = nullptr;
	uint32_t servoTimChannel_ = 0;

	void UpdateHandler_() final;

	void HeartbeatHandler_() final;

};

} // namespace my_engineer

#endif	
