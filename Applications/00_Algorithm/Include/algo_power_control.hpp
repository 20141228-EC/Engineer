/*
    * @file chassis_power_control.hpp
    * @author yh
    * @brief 底盘功率限制头文件
    * @version 1.0
    * @date 2025-11-18
    *
*/
#ifndef ALGO_POWER_CONTROL_HPP
#define ALGO_POWER_CONTROL_HPP

#include "mod_chassis.hpp"
#include "D:/STM32project/resource/Engineer-2025_Engineer/Applications/03_Device/Include/mtr/mtr_common.hpp"


namespace my_engineer{
class CModChassis;

class ChassisPowerController {
private:
	static constexpr uint16_t kDefaultMaxPower = 120;		//默认最大功率限制
	static constexpr float kTorqueCoeff = 1.99688994e-6f;	//力矩系数(20/16384)*(0.3)*(187/3591)/9.55
	static constexpr float k1 = 1.23e-07f;					//力矩二次方系数
	static constexpr float k2 = 1.453e-07f;					//转速二次方
	static constexpr float kConstant = 4.081f; 				//静态功耗项
	static constexpr int16_t kMotorOutputMax = 16000; 		//电机最大输出限制
	
	//预留最大功率获取接口（后续从裁判系统读取） 
	uint16_t get_chassis_max_power(){
		return kDefaultMaxPower;
	} 

public:
	//功率控制主函数 
	void control(CModChassis* chassis, uint8_t motorIndex);

};

extern ChassisPowerController chassis_power_controller;
 
}	//namespace my_engineer

#endif  // CHASSIS_POWER_CONTROL_HPP
