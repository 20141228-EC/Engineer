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
    mitCtrl[LR].kp = chassisParam.MIT_L_kp;
	mitCtrl[LR].kd = chassisParam.MIT_L_kd;         ///< 左右腿暂用同一套pid 若效果不好待改

    chassisParam.rollCorrectionPidParam.threadNum = 1;
    pidRollCtrl.InitPID(&chassisParam.rollCorrectionPidParam);

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

	CDevMtrDM_MIT *pMtr[2];
    pMtr[LL] = static_cast<CDevMtrDM_MIT *>(motor[LL]);
    pMtr[LR] = static_cast<CDevMtrDM_MIT *>(motor[LR]);

	// 更新组件信息
    HipInfo.pos_L_L = pMtr[LL]->motorData[CDevMtr::DATA_ANGLE];
    HipInfo.pos_L_R = pMtr[LR]->motorData[CDevMtr::DATA_ANGLE];   

    // 缓慢移动控制逻辑
	static float_t next_angle[2] = {0.0f};
	static float_t gradual_kp = 0.005f;
	static float_t gradual_min = 0.03f;

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
			pMtr[LL]->Control_MIT(0.0f, 0.0f, ecd2rad(0.0f) * L_LIFT_MOTOR_DIR, 0.0f, 0.0f);
            pMtr[LR]->Control_MIT(0.0f, 0.0f, ecd2rad(0.0f) * R_LIFT_MOTOR_DIR, 0.0f, 0.0f);
			HipCmd = SHipCommand();																	///<调用默认构造函数初始化
			return APP_OK;
		}

		case FSM_PREINIT: {
			HipCmd.L_Set_Angle = CHASSIS_HIP_INIT_ECD_L;
            HipCmd.R_Set_Angle = CHASSIS_HIP_INIT_ECD_R; ///< 初始化髋关节
            pidRollCtrl.ResetPidController(); ///< 重置pid控制器
			Component_FSMFlag_ = FSM_INIT;
			return APP_OK;
		}

		case FSM_INIT: {
			if (ecd2rad(fabs(HipInfo.pos_L_L - HipCmd.L_Set_Angle)) < 5.0 && ecd2rad(fabs(HipInfo.pos_L_R - HipCmd.R_Set_Angle)) < 5.0) {
				Component_FSMFlag_ = FSM_CTRL;
				componentStatus = APP_OK;
			}
			pMtr[LL]->Control_MIT(mitCtrl[LL].kp, mitCtrl[LL].kd, ecd2rad(HipCmd.L_Set_Angle), 0.0f, 0.0f);
            pMtr[LR]->Control_MIT(mitCtrl[LR].kp, mitCtrl[LR].kd, ecd2rad(HipCmd.R_Set_Angle), 0.0f, 0.0f);
			return APP_OK;
		}

		case FSM_CTRL: {

			pMtr[LL]->Control_MIT(mitCtrl[LL].kp, mitCtrl[LL].kd, ecd2rad(next_angle[0]), 0.0f, 0.0f);
            pMtr[LR]->Control_MIT(mitCtrl[LR].kp, mitCtrl[LR].kd, ecd2rad(next_angle[1]), 0.0f, 0.0f);
			return APP_OK;
		}

		default: {
			componentStatus = APP_ERROR;
			return APP_ERROR;
		}
	}

	return APP_OK;
}

} // namespace my_engineer