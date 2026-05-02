/******************************************************************************
 * @brief        
 * 
 * @file         com_hip.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2025-12-05
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

 #include "mod_chassis.hpp"
 float_t debug_output_L = 0.f;
 float_t debug_output_buffer_L = 0.f;
 float_t debug_actual_speed_LL = 0.f;
 float_t debug_raw_speed_LL = 0.f;

 namespace my_engineer{

 CMemsBase *pmems_hip_test = nullptr;

/**
 * @brief 初始化底盘髋关节组件
 * 
 * @param param 
 * @return EAppStatus 
 */
EAppStatus CModChassis::CComHip::InitComponent(SModInitParam_Base &param){

    	// 检查param是否正确
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	// 类型转换
	auto chassisParam = static_cast<SModInitParam_Chassis &>(param);

	// 保存电机和陀螺仪指针
    mems = MemsIDMap.at(chassisParam.memsDevID);
	motor[LL] = MotorIDMap.at(chassisParam.hipMotorID_L_L);					
    motor[LR] = MotorIDMap.at(chassisParam.hipMotorID_L_R);	                ///<通过对arm类图的索引找到初始注册的电机

	// 初始化PID控制器
	mitCtrl[LL].kp = chassisParam.MIT_L_kp;
	mitCtrl[LL].kd = chassisParam.MIT_L_kd;
	mitCtrl[LL].tau = chassisParam.MIT_L_tau;
    mitCtrl[LR].kp = chassisParam.MIT_R_kp;
	mitCtrl[LR].kd = chassisParam.MIT_R_kd;
	mitCtrl[LR].tau = chassisParam.MIT_R_tau;

    chassisParam.rollCorrectionPidParam.threadNum = 1;
    pidRollCtrl.InitPID(&chassisParam.rollCorrectionPidParam);

	chassisParam.HipPosPidParam_L.threadNum = 1;
    chassisParam.HipPosPidParam_R.threadNum = 1;
    chassisParam.HipSpdPidParam_L.threadNum = 1;
    chassisParam.HipSpdPidParam_R.threadNum = 1;
    HipPosPid[LL].InitPID(&chassisParam.HipPosPidParam_L);
    HipPosPid[LR].InitPID(&chassisParam.HipPosPidParam_R);
    HipSpdPid[LL].InitPID(&chassisParam.HipSpdPidParam_L);
    HipSpdPid[LR].InitPID(&chassisParam.HipSpdPidParam_R);


    // test
    pmems_hip_test = mems;

    mems->StartDevice();

	Component_FSMFlag_ = FSM_RESET;
	componentStatus = APP_OK;

	return APP_OK;
}

// /**
//  * @brief 根据整车roll角度计算出持平需要的腿长
//  * 
//  * @retval roll环pid输出值
//  */
// float Roll_Pid(float target, float measure){}

/**
 * @brief 更新髋关节关节组件
 * 
 * @return EAppStatus 
 */
EAppStatus CModChassis::CComHip::UpdateComponent() {
	// 检查组件状态
	if (componentStatus == APP_RESET) return APP_ERROR;
	constexpr bool kEnableHipMitSend = false; // 仅保留舵轮电机CAN发送

	CDevMtrDM_MIT *pMtr[2];
    pMtr[LL] = static_cast<CDevMtrDM_MIT *>(motor[LL]);
    pMtr[LR] = static_cast<CDevMtrDM_MIT *>(motor[LR]);

	// 更新组件信息
	HipInfo.pos_L_L = rad2deg(pMtr[LL]->motorPhyAngle);
	HipInfo.pos_L_R = rad2deg(pMtr[LR]->motorPhyAngle); 
	
	// HipInfo.pos_L_L = pMtr[LL]->motorData[CDevMtr::DATA_POSIT] * 10;
	// HipInfo.pos_L_R = pMtr[LR]->motorData[CDevMtr::DATA_POSIT] * 10;

    // 缓慢移动控制逻辑
	static float_t next_angle[2] = {0.0f};
	static float_t gradual_kp = 0.01f;
	static float_t gradual_min = 0.04f;

	next_angle[LL] += (HipCmd.L_Set_Angle - next_angle[LL]) * gradual_kp;			///<一阶低通滤波，避免角度突变
	if (fabs(next_angle[LL] - HipCmd.L_Set_Angle) < gradual_min) {
		next_angle[LL] = HipCmd.L_Set_Angle;									///<设定最小的分辨率
	}

    next_angle[LR] += (HipCmd.R_Set_Angle - next_angle[LR]) * gradual_kp;			///<一阶低通滤波，避免角度突变
	if (fabs(next_angle[LR] - HipCmd.R_Set_Angle) < gradual_min) {
		next_angle[LR] = HipCmd.R_Set_Angle;									///<设定最小的分辨率
	}

	/*MIT 模式参数：
  - kp：位置刚度系数（范围 0-500 N/rad）
  - kd：阻尼系数（范围 0-5 N·s/rad）
  - position：目标位置（弧度）
  - velocity：速度给定（这里设为0）
  - torque：力矩前馈（这里设为0）*/

	switch (Component_FSMFlag_) {
		case FSM_RESET: {
			if (kEnableHipMitSend) {
				// pMtr[LL]->Control_MIT(0.0f, 0.0f, deg2rad(0.0f) * L_LIFT_MOTOR_DIR, 0.0f, 0.0f);
            	// pMtr[LR]->Control_MIT(0.0f, 0.0f, deg2rad(0.0f) * R_LIFT_MOTOR_DIR, 0.0f, 0.0f);
			}
			// mtrOutputBuffer.fill(0);
			// HipPosPid[LL].ResetPidController();
			// HipSpdPid[LL].ResetPidController();
			// HipPosPid[LR].ResetPidController();
			// HipSpdPid[LR].ResetPidController();
			// pMtr[LL]->Control_MIT(0.0f, 0.0f, 0.0f, 0.0f, mtrOutputBuffer[LL] * DM8009P_CURRENT_TO_TORQUE_L);
			// pMtr[LR]->Control_MIT(0.0f, 0.0f, 0.0f, 0.0f, mtrOutputBuffer[LR] * DM8009P_CURRENT_TO_TORQUE_R);
			HipCmd = SHipCommand();				///<调用默认构造函数初始化
			return APP_OK;
		}

		case FSM_PREINIT: {
			HipCmd.L_Set_Angle = deg2rad(CHASSIS_HIP_INIT_ECD_L);
            HipCmd.R_Set_Angle = deg2rad(CHASSIS_HIP_INIT_ECD_R); ///< 初始化髋关节
			// HipCmd.L_Set_Angle = CHASSIS_HIP_INIT_ECD_L;
			// HipCmd.R_Set_Angle = CHASSIS_HIP_INIT_ECD_R;			///< 初始化髋关节
            // pidRollCtrl.ResetPidController(); ///< 重置pid控制器
			Component_FSMFlag_ = FSM_INIT;
			return APP_OK;
		}

		case FSM_INIT: {
			if (deg2rad(fabs(HipInfo.pos_L_L - HipCmd.L_Set_Angle)) < 0.5 && deg2rad(fabs(HipInfo.pos_L_R - HipCmd.R_Set_Angle)) < 0.5) {
				Component_FSMFlag_ = FSM_CTRL;
				componentStatus = APP_OK;
			}
			// pMtr[LL]->Control_MIT(mitCtrl[LL].kp, mitCtrl[LL].kd, deg2rad(HipCmd.L_Set_Angle), 0.0f, 0.0f);
			if (kEnableHipMitSend) {
            	// pMtr[LL]->Control_MIT(mitCtrl[LL].kp, mitCtrl[LL].kd, deg2rad(next_angle[LL]), 0.0f, 0.0f);
			// pMtr[LR]->Control_MIT(mitCtrl[LR].kp, mitCtrl[LR].kd, deg2rad(HipCmd.R_Set_Angle), 0.0f, 0.0f);
				// pMtr[LR]->Control_MIT(mitCtrl[LR].kp, mitCtrl[LR].kd, deg2rad(next_angle[LR]), 0.0f, 0.0f);
			}
			// if(fabs(HipInfo.pos_L_L - HipCmd.L_Set_Angle) < 100 && fabs(HipInfo.pos_L_R - HipCmd.R_Set_Angle) < 100){
			// 	Component_FSMFlag_ = FSM_CTRL;
			// 	componentStatus = APP_OK;
			// }
			// _UpdateOutput(HipCmd.L_Set_Angle, HipCmd.R_Set_Angle);
			// pMtr[LL]->Control_MIT(0.0f, 0.0f, 0.0f, 0.0f, mtrOutputBuffer[LL] * DM8009P_CURRENT_TO_TORQUE_L);
			// pMtr[LR]->Control_MIT(0.0f, 0.0f, 0.0f, 0.0f, mtrOutputBuffer[LR] * DM8009P_CURRENT_TO_TORQUE_R);
			return APP_OK;
		}

		case FSM_CTRL: {

			if(parent->reset_hip){		///< 复位时候用滤波后的角度值，防止猛肘限位
				if (kEnableHipMitSend) {
					// pMtr[LL]->Control_MIT(mitCtrl[LL].kp, mitCtrl[LL].kd, deg2rad(next_angle[LL]), 0.0f, 0.0f);
					// pMtr[LR]->Control_MIT(mitCtrl[LR].kp, mitCtrl[LR].kd, deg2rad(next_angle[LR]), 0.0f, 0.0f);
				}
			}
			else{
				if (kEnableHipMitSend) {
					// pMtr[LL]->Control_MIT(mitCtrl[LL].kp, mitCtrl[LL].kd, deg2rad(HipCmd.L_Set_Angle), 0.0f, mitCtrl[LL].tau);
					// pMtr[LR]->Control_MIT(mitCtrl[LR].kp, mitCtrl[LR].kd, deg2rad(HipCmd.R_Set_Angle), 0.0f, mitCtrl[LR].tau);
				}
			}
			return APP_OK;
			// _UpdateOutput(HipCmd.L_Set_Angle, HipCmd.R_Set_Angle);
			// pMtr[LL]->Control_MIT(0.0f, 0.0f, 0.0f, 0.0f, mtrOutputBuffer[LL] * DM8009P_CURRENT_TO_TORQUE_L);
			// pMtr[LR]->Control_MIT(0.0f, 0.0f, 0.0f, 0.0f, mtrOutputBuffer[LR] * DM8009P_CURRENT_TO_TORQUE_R);
			// return APP_OK;
		}

		default: {
			componentStatus = APP_ERROR;
			return APP_ERROR;
		}
	}

	return APP_OK;
}

EAppStatus CModChassis::CComHip::_UpdateOutput(float_t posit_L, float_t posit_R){

	// 用于无符号类型的转化
    static auto uint_to_float = [](uint16_t x_uint, float xmin, float xmax, uint8_t bits) -> float {
        float span = xmax - xmin;
        float data_norm = static_cast<float>(x_uint) / ((1 << bits) - 1);
        return data_norm * span + xmin;
    };

    // 用于有符号类型的转化
    static auto int_to_float = [](int16_t x_int, float xmin, float xmax, uint8_t bits) -> float {
        float span = xmax - xmin;
        // 计算有符号数的最大值：2^(bits-1) - 1 （12位则为2047）
        int32_t int_max = (1 << (bits - 1)) - 1;
        // 有符号数归一化：映射到[-1, 1]区间，再缩放至[xmin, xmax]
        float data_norm = static_cast<float>(x_int) / static_cast<float>(int_max);
        return (data_norm + 1.0f) * 0.5f * span + xmin;
    };

	DataBuffer<float_t> PosMeasure_L = {static_cast<float_t>(motor[LL]->motorData[CDevMtr::DATA_POSIT]) * 10};
	DataBuffer<float_t> PosMeasure_R = {static_cast<float_t>(motor[LR]->motorData[CDevMtr::DATA_POSIT]) * 10};	///< 获取位置测量值

	DataBuffer<float_t> PosTarget_L = {posit_L};
	DataBuffer<float_t> PosTarget_R = {posit_R};	///< 更新目标值

	auto Spd_L = HipPosPid[LL].UpdatePidController(PosTarget_L, PosMeasure_L);
	auto Spd_R = HipPosPid[LR].UpdatePidController(PosTarget_R, PosMeasure_R);

	// 读取髋关节电机的实际速度 (rpm)
    float_t raw_speed_LL = int_to_float(
        motor[LL]->motorData[CDevMtr::DATA_SPEED],
        -45,
        45,
        12
    );

	float_t raw_speed_LR = int_to_float(
		motor[LR]->motorData[CDevMtr::DATA_SPEED],
		-45,
		45,
		12
	);
	debug_raw_speed_LL = int_to_float(
        motor[LL]->motorData[CDevMtr::DATA_SPEED],
        -45,
        45,
        12
    );

    float_t actual_speed_LL = abs(raw_speed_LL - 45) * ((raw_speed_LL - 45) > 0 ? 1 : -1);
	float_t actual_speed_LR = abs(raw_speed_LR - 45) * ((raw_speed_LR - 45) > 0 ? 1 : -1);
	debug_actual_speed_LL = actual_speed_LL;

	DataBuffer<float_t> SpdMeasure_L = {static_cast<float_t>(actual_speed_LL) * 10};
	DataBuffer<float_t> SpdMeasure_R = {static_cast<float_t>(actual_speed_LR) * 10};	///< 获取速度测量值

	auto output_L = HipSpdPid[LL].UpdatePidController(Spd_L, SpdMeasure_L);
	debug_output_L = static_cast<float_t>(output_L[0]);
	auto output_R = HipSpdPid[LR].UpdatePidController(Spd_R, SpdMeasure_R);

	mtrOutputBuffer[LL] = static_cast<float_t>(output_L[0]);
	debug_output_buffer_L = mtrOutputBuffer[LL];
	mtrOutputBuffer[LR] = static_cast<float_t>(output_R[0]); 
	
		
	return APP_OK;
}

} // namespace my_engineer