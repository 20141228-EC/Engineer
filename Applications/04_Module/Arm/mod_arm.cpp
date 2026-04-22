/******************************************************************************
 * @brief        
 * 
 * @file         mod_arm.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2026-01-28
 * 
 * @copyright    Copyright (c) 2026
 * 
 ******************************************************************************/

#include "mod_arm.hpp"



namespace my_engineer {

CModArm *pArm_test = nullptr;

// ///< 全局变量
bool Need_Grav_Compensation = false; ///< 是否启用重力补偿
bool Is_Recording_ArmTorque = true; ///<是否正在记录数据
DataBuffer<float_t> arm_Info[3][10]; ///<用于记录臂的力矩，三个关节，1000个数据点
uint16_t index = 0; ///< 数组索引
bool is_record = false; ///< 是否要记录数据

/**
 * @brief 初始化机械臂模块
 * 
 * @param param 
 * @return EAppStatus 
 */
EAppStatus CModArm::InitModule(SModInitParam_Base &param) {
	// 检查param是否正确
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	// 类型转换
	auto armParam = static_cast<SModInitParam_Arm &>(param);
	moduleID = armParam.moduleID;

	comjoint_.InitComponent(param);
	comRoll_.InitComponent(param);
	comEnd_.InitComponent(param);
	comGrip_.InitComponent(param);
	comGrip_.parentModule = this;  ///< 设置父模块指针，用于Roll耦合补偿


	// 创建任务并注册模块
	CreateModuleTask_();
	RegisterModule_();

	// test
	pArm_test = this;

	Module_FSMFlag_ = FSM_RESET;
	moduleStatus = APP_OK;

	return APP_OK;
}

/**
 * @brief 更新处理
 * 
 */
void CModArm::UpdateHandler_() {
	// 检查模块状态
	static uint8_t HalfTickRate = 0;
	HalfTickRate = 1 - HalfTickRate;
	if (moduleStatus == APP_RESET) return;
	
	if(HalfTickRate) { comRoll_.UpdateComponent(); } ///< 降为500Hz
	// 更新组件
	comjoint_.UpdateComponent();
	comEnd_.UpdateComponent();	///<更新电机数据
	comGrip_.UpdateComponent();

	// 更新模块信息
	armInfo.angle_Yaw = comjoint_.MtrPositToPhyPosit_yaw(comjoint_.jointInfo.posit_yaw);
	armInfo.angle_Pitch1 = comjoint_.MtrPositToPhyPosit_pitch1(comjoint_.jointInfo.posit_pitch1);
	armInfo.angle_Pitch2 = comjoint_.MtrPositToPhyPosit_pitch2(comjoint_.jointInfo.posit_pitch2);
	armInfo.angle_Pitch3 = comjoint_.MtrPositToPhyPosit_pitch3(comjoint_.jointInfo.posit_pitch3);	///<这里是将底层的关节信息转换为用户层的arm信息
	armInfo.angle_Roll = comRoll_.MtrAngleToPhyAngle(comRoll_.rollInfo.angle);
	armInfo.angle_end_pitch =
		comEnd_.MtrPositToPhyPosit_Pitch(comEnd_.endInfo.posit_Pitch);
	armInfo.angle_end_roll =
		comEnd_.MtrPositToPhyPosit_Roll(comEnd_.endInfo.posit_Roll);								///<将电机的机械角度转换为物理角度
	armInfo.length_grip = CComGrip::MtrPositToPhyPosit(static_cast<float_t>(comGrip_.gripInfo.posit_grip));
	armInfo.isAngleArrived_Yaw = comjoint_.jointInfo.isPositArrived_yaw;
	armInfo.isAngleArrived_Pitch1 = comjoint_.jointInfo.isPositArrived_pitch1;
	armInfo.isAngleArrived_Pitch2 = comjoint_.jointInfo.isPositArrived_pitch2;
	armInfo.isAngleArrived_Pitch3 = comjoint_.jointInfo.isPositArrived_pitch3;
	armInfo.isAngleArrived_Roll = comRoll_.rollInfo.isAngleArrived;
	armInfo.isAngleArrived_End_Pitch = comEnd_.endInfo.isPositArrived_Pitch;
	armInfo.isAngleArrived_End_Roll = comEnd_.endInfo.isPositArrived_Roll;
	armInfo.isAngleArrived_Grip = comGrip_.gripInfo.isGripped;
	armInfo.isGripped = comGrip_.gripInfo.isGripped;
	armInfo.holdLength_grip = CComGrip::MtrPositToPhyPosit(
		static_cast<float_t>(comGrip_.gripInfo.holdPosit_Grip));

	if(Need_Grav_Compensation) ///< 启用重力补偿
	{
		Grav_Compemsation_Pitch1();
		Grav_Compemsation_Pitch2();
		Grav_Compemsation_Roll();
	}

	// 填充电机发送缓冲区
	CDevMtrKT::FillCanTxBuffer(comjoint_.motor[CComJoint::P1],							///<用的是关节底层信息的发送
								comjoint_.mtrCanTxNode[CComJoint::P1]->dataBuffer,
								comjoint_.mtrOutputBuffer[CComJoint::P1]);		///< 1.电机 2.can节点信息 3. 输出缓冲区
	CDevMtrKT::FillCanTxBuffer(comjoint_.motor[CComJoint::P2],
								comjoint_.mtrCanTxNode[CComJoint::P2]->dataBuffer,
								comjoint_.mtrOutputBuffer[CComJoint::P2]);
	CDevMtrKT::FillCanTxBuffer(comjoint_.motor[CComJoint::P3],
								comjoint_.mtrCanTxNode[CComJoint::P3]->dataBuffer,
								comjoint_.mtrOutputBuffer[CComJoint::P3]);	
	CDevMtrKT::FillCanTxBuffer(comjoint_.motor[CComJoint::Y],
								comjoint_.mtrCanTxNode[CComJoint::Y]->dataBuffer,
								comjoint_.mtrOutputBuffer[CComJoint::Y]);
	CDevMtrDJI::FillCanTxBuffer(comEnd_.motor[CComEnd::L],
								comEnd_.mtrCanTxNode[CComEnd::L]->dataBuffer,
								comEnd_.mtrOutputBuffer[CComEnd::L]);
	CDevMtrDJI::FillCanTxBuffer(comEnd_.motor[CComEnd::R],
								comEnd_.mtrCanTxNode[CComEnd::R]->dataBuffer,
								comEnd_.mtrOutputBuffer[CComEnd::R]);
	CDevMtrDJI::FillCanTxBuffer(comGrip_.motor,
								comGrip_.mtrCanTxNode->dataBuffer,
								comGrip_.mtrOutputBuffer);

}

/**
 * @brief 心跳处理
 * 
 */
void CModArm::HeartbeatHandler_() {
	// 心跳处理逻辑
}

/**
 * @brief 创建模块任务
 * 
 */
EAppStatus CModArm::CreateModuleTask_() {
	// 任务已存在，删除任务
	if (moduleTaskHandle != nullptr) vTaskDelete(moduleTaskHandle);

	// 创建任务
	xTaskCreate(StartArmModuleTask, "Arm Task",
				512, this, proc_ModuleTaskPriority,
				&moduleTaskHandle);

	return APP_OK;
}

/**
 * @brief 限制机械臂模块的控制命令大小
 * 
 */
EAppStatus CModArm::RestrictArmCommand_() {
	// 检查模块状态
	if (moduleStatus == APP_RESET) {
		armCmd = SArmCmd();
		return APP_ERROR;
	}

	static float_t prevPitch1 = ARM_PITCH1_INIT_ANGLE;
    // 物理限位
    armCmd.set_angle_Yaw =
        std::clamp(armCmd.set_angle_Yaw, ARM_YAW_PHYSICAL_RANGE_MIN, ARM_YAW_PHYSICAL_RANGE_MAX);

	if(armCmd.set_angle_Pitch2 < 13.f && armCmd.set_angle_Roll < 90.f ) { 
		armCmd.set_angle_end_pitch = std::clamp(armCmd.set_angle_end_pitch, 0.f, 90.f);
	}

    armCmd.set_angle_Pitch1 =
        std::clamp(armCmd.set_angle_Pitch1,
                   ARM_PITCH1_PHYSICAL_RANGE_MIN, ARM_PITCH1_PHYSICAL_RANGE_MAX);
	
	if(!armCmd.isCustomCtrl){  //非自定义控制器模式
		armCmd.set_angle_Pitch2 += (armCmd.set_angle_Pitch1 - prevPitch1) * 1;
		prevPitch1 = armCmd.set_angle_Pitch1;
		
		float_t p2UpperLimit = std::min(ARM_P2_MAX_WHEN_P1_MIN + armCmd.set_angle_Pitch1,
														ARM_PITCH2_PHYSICAL_RANGE_MAX);//p2的动态限位受到p1的影响
		
		armCmd.set_angle_Pitch2 =
			std::clamp(armCmd.set_angle_Pitch2, ARM_PITCH2_PHYSICAL_RANGE_MIN, p2UpperLimit);

		float_t p3LowerLimit = std::max(-(armCmd.set_angle_Pitch2 + armCmd.set_angle_Pitch1), ARM_PITCH3_PHYSICAL_RANGE_MIN);//p3动态限位受到p1和p2的影响，因为p3的角度是朝下的所以是负的

		armCmd.set_angle_Pitch3 =
			std::clamp(armCmd.set_angle_Pitch3,
							p3LowerLimit, ARM_PITCH3_PHYSICAL_RANGE_MAX);
	}

	else{ //自定义控制器模式下
		armCmd.set_angle_Pitch2 =
			std::clamp(armCmd.set_angle_Pitch2,
						ARM_PITCH2_PHYSICAL_RANGE_MIN, ARM_PITCH2_PHYSICAL_RANGE_MAX);
		armCmd.set_angle_Pitch3 =
			std::clamp(armCmd.set_angle_Pitch3,
						ARM_PITCH3_PHYSICAL_RANGE_MIN, ARM_PITCH3_PHYSICAL_RANGE_MAX);
	}
	prevPitch1 = armCmd.set_angle_Pitch1; //避免切换的跳变

    armCmd.set_angle_Roll =
        std::clamp(armCmd.set_angle_Roll,
                   ARM_ROLL_PHYSICAL_RANGE_MIN, ARM_ROLL_PHYSICAL_RANGE_MAX);
    armCmd.set_angle_end_pitch =
        std::clamp(armCmd.set_angle_end_pitch,
                   ARM_END_PITCH_PHYSICAL_RANGE_MIN, ARM_END_PITCH_PHYSICAL_RANGE_MAX);
    armCmd.set_length_grip = 
        std::clamp(armCmd.set_length_grip,
            ARM_END_GRIP_PHYSICAL_RANGE_MIN, ARM_END_GRIP_PHYSICAL_RANGE_MAX);

    // 自定义控制器限制
    // if(armCmd.isCustomCtrl) {
    //     armCmd.set_angle_Pitch1 =
    //         std::clamp(armCmd.set_angle_Pitch1,
    //                    18.0f, ARM_PITCH1_PHYSICAL_RANGE_MAX);
    // }

    // 自动控制模式下跳过后续更复杂的动态限位
    if (armCmd.isAutoCtrl) {
        return APP_OK;
    }

    // // 动态关联限位
    // // Pitch1 和 Pitch2 的关联
    // float_t pitch2_min_limit = 0.f;
	// float_t pitch2_max_limit = 23.f + armCmd.set_angle_Pitch1;
    // if (armCmd.set_angle_Pitch1 >= 36.f) {
    //     pitch2_min_limit = 23.f;
    // }
	// if (armCmd.set_angle_Pitch1 >= 65.f) {
    //     pitch2_min_limit = 25.f;
    // }
    // armCmd.set_angle_Pitch2 = std::clamp(armCmd.set_angle_Pitch2, pitch2_min_limit, pitch2_max_limit);

    // Pitch2 和 Roll 的关联,转换了方向之后没有干涉的影响
    // if (armCmd.set_angle_Pitch2 < 20.0f) {
    //     armCmd.set_angle_Roll = std::clamp(armCmd.set_angle_Roll,
    //         ARM_ROLL_PHYSICAL_RANGE_MIN, 18.f);
    // }

    //Pitch1 和 Yaw 的关联
    // if (armCmd.set_angle_Pitch1 < 15.f) {
    //     armCmd.set_angle_Yaw = std::clamp(armCmd.set_angle_Yaw,
    //         -40.f, 40.f);
    // }

    // 最高优先级的限位
    if (should_limit_yaw) {
        armCmd.set_angle_Yaw = std::clamp(armCmd.set_angle_Yaw,
            -12.0f, 6.0f);
    }
		return APP_OK;
}

/** 
 * @brief 根据关节角度计算大pitch的重补扭矩
 * 
 * @retval null
*/
EAppStatus CModArm::Grav_Compemsation_Pitch1()
{
	float_t pitch1 = deg2rad(armInfo.angle_Pitch1 - 18);
	float_t pitch2 = deg2rad(armInfo.angle_Pitch2 - 11);
	float_t roll = deg2rad(armInfo.angle_Roll);
	float_t end_pitch = deg2rad(armInfo.angle_end_pitch); ///< 获取关节角

	this->comjoint_.g_pitch1 = pitch1;
	this->comjoint_.g_pitch2 = pitch2;
	
	this->comjoint_.Grav_Pitch1_Out = (26.0*cos(pitch1 + 0.34) - 7.8*cos(pitch1 + pitch2 + 0.08) - 0.21*cos(pitch1 + pitch2 + roll - 1.4) - 0.21*cos(pitch1 + pitch2 - roll - 1.5) + 0.19*cos(pitch1 + pitch2 + 0.12)*cos(end_pitch) + 0.19*cos(pitch1 + pitch2 + 0.12)*sin(roll)*sin(end_pitch)) / MG8010_i36V2_Torque_Constant;
	return APP_OK;
}

/** 
 * @brief 根据关节角度计算小pitch的重补扭矩
 * 
 * @retval null
*/	
EAppStatus CModArm::Grav_Compemsation_Pitch2()
{
	float_t pitch1 = deg2rad(armInfo.angle_Pitch1 - 18);
	float_t pitch2 = deg2rad(armInfo.angle_Pitch2 - 11);
	float_t roll = deg2rad(armInfo.angle_Roll);
	float_t end_pitch = deg2rad(armInfo.angle_end_pitch); ///< 获取关节角

	this->comjoint_.Grav_Pitch2_Out = -(7.8*cos(pitch1 + pitch2 + 0.08) + 0.21*cos(pitch1 + pitch2 + roll - 1.4) + 0.21*cos(pitch1 + pitch2 - roll - 1.5) - 0.19*cos(pitch1 + pitch2 + 0.12)*cos(end_pitch) - 0.19*sin(pitch1 + pitch2 + 0.12)*cos(roll)*sin(end_pitch)) / MG6012_i36V3_Torque_Constant / 0.4;
	return APP_OK;
}

/** 
 * @brief 根据关节角度计算roll的重补扭矩
 * 
 * @retval null
*/
EAppStatus CModArm::Grav_Compemsation_Roll()
{
	float_t pitch1 = deg2rad(armInfo.angle_Pitch1 - 18);
	float_t pitch2 = deg2rad(armInfo.angle_Pitch2 - 11);
	float_t roll = deg2rad(armInfo.angle_Roll);
	float_t end_pitch = deg2rad(armInfo.angle_end_pitch); ///< 获取关节角

	this->comRoll_.Grav_Roll_Out = -0.42*sin(pitch1 + pitch2 + 1.7)*sin(roll) - 0.19*sin(roll)*sin(pitch1 + pitch2 + 1.7)*cos(end_pitch + 1.59);
	return APP_OK;
}

} // namespace my_engineer
