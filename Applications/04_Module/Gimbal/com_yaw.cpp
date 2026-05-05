/******************************************************************************
 * @brief        
 * 
 * @file         com_yaw.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2026-03-06
 * 
 * @copyright    Copyright (c) 2026
 * 
 ******************************************************************************/

#include "mod_gimbal.hpp"

namespace my_engineer {

    CMemsBase *pmems_test = nullptr;

/**
 * @brief 初始化云台大yaw组件
 * 
 * @param param 
 * @return EAppStatus 
 */
EAppStatus CModGimbal::CComYaw::InitComponent(SModInitParam_Base &param) {
	// 检查param是否正确
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	// 类型转换
	auto gimbalParam = static_cast<SModInitParam_Gimbal &>(param);

	// 保存电机和传感器指针
	motor = MotorIDMap.at(gimbalParam.yawMotorID);
	mems = MemsIDMap.at(gimbalParam.memsDevID);

	//获取算法指针
    filter = static_cast<CAlgo_IMU_Ave*>(AlgoIDMap.at(gimbalParam.FilterID));
    if(!filter){
        return APP_ERROR;
    }

    // CAN发送节点
    mtrCanTxNode = gimbalParam.MotorTxNode_Yaw;

	// 初始化PID控制器
	gimbalParam.YawPosPidParam_Gyro.threadNum = 1;
    pidPosCtrl_Gyro.InitPID(&gimbalParam.YawPosPidParam_Gyro);
    gimbalParam.YawSpdPidParam_Gyro.threadNum = 1;
    pidSpdCtrl_Gyro.InitPID(&gimbalParam.YawSpdPidParam_Gyro);
	gimbalParam.YawPosPidParam_Mec.threadNum = 1;
    pidPosCtrl_Mec.InitPID(&gimbalParam.YawPosPidParam_Mec);
    gimbalParam.YawSpdPidParam_Mec.threadNum = 1;
    pidSpdCtrl_Mec.InitPID(&gimbalParam.YawSpdPidParam_Mec);

    mtrOutputBuffer = 0;

    pmems_test = mems;
    mems->StartDevice();

	Component_FSMFlag_ = FSM_RESET;
	componentStatus = APP_OK;

	return APP_OK;
}

/**
 * @brief 更新Gimbal关节组件
 * 
 * @return EAppStatus 
 */
EAppStatus CModGimbal::CComYaw::UpdateComponent() {
	// 检查组件状态
	if (componentStatus == APP_RESET) return APP_ERROR;

	// 更新组件信息
	static float_t Init_Encoder_Posit_MACH = 0.f;

	yawInfo.posit = filter->Imu_Ave_Info.imu_ave_yaw;	// 由于陀螺仪Z轴与电机编码器增加的方向是相反的，位置反馈需加负号
	yawInfo.encoder = HalfCycle(motor->motorData[CDevMtr::DATA_ANGLE] - GINBAL_FRONT_MOTOR_ANGLE, static_cast<int32_t>(65535));	// 将encoder归一化在朝前为0
	yawInfo.isPositArrived = (fabs(yawInfo.posit - yawCmd.setPosit) < 1.5f);

	switch (Component_FSMFlag_) {
		case FSM_RESET: {
			mtrOutputBuffer = 0;
			yawCmd = SYawCmd();																	///<调用默认构造函数初始化
			return APP_OK;
		}

		case FSM_PREINIT: {
			yawCmd.setEncoder = 0;
			Component_FSMFlag_ = FSM_INIT;
			return APP_OK;
		}

		case FSM_INIT: {
			if (fabs(yawInfo.encoder - yawCmd.setEncoder) < 100) {		///< 阈值姑且定为500 后续再改
			mtrOutputBuffer = 0;

			pidPosCtrl_Gyro.ResetPidController();
			pidSpdCtrl_Gyro.ResetPidController();
			pidPosCtrl_Mec.ResetPidController();
			pidSpdCtrl_Mec.ResetPidController();

			// Init_Encoder_Posit_MACH = filter->Imu_Ave_Info.imu_ave_yaw;	///< 初始化完成记录当前yaw角度

			yawCmd.setPosit = 0.0f;

				Component_FSMFlag_ = FSM_CTRL;
				componentStatus = APP_OK;	// 用于调试，先把到位条件注释掉，直接进ctrl，等调完pid再搞回来
			}
			else{
				_UpdateOutput_Mec(static_cast<float_t>(yawCmd.setEncoder));
			}
			
			yawCmd.setPosit = filter->Imu_Ave_Info.imu_ave_yaw; // 如果上面那个特殊处理不好用，可以用这个
			return APP_OK;
		}

		case FSM_CTRL: {
			_UpdateOutput_Gyro(yawCmd.setPosit);
			// mtrOutputBuffer = 0; // 取消这行注释以排查旧值残留
			return APP_OK;
		}

		default: {
			componentStatus = APP_ERROR;
			return APP_ERROR;
		}
	}

	return APP_OK;
}

/**
 * @brief 陀螺仪模式更新输出
 * @param 陀螺仪角度目标值 posit
 * @retval EAppStatus
 */
EAppStatus CModGimbal::CComYaw::_UpdateOutput_Gyro(float_t posit){

	float_t err = HalfCycle(posit - yawInfo.posit, 360.0f);

    DataBuffer<float_t> posit_measure = {0.0f};		// 误差为0
    DataBuffer<float_t> posit_target = {err};		// 目标值为得到的误差

	auto spd_Yaw = pidPosCtrl_Gyro.UpdatePidController(posit_target, posit_measure);

	DataBuffer<float_t> spd_measure = {static_cast<float_t>(mems->memsData[CMemsBase::DATA_GYRO_Z]) * 57.2957795f}; // 同样加负号与编码器极性对齐
	// DataBuffer<float_t> spd_measure = {static_cast<float_t>(motor->motorData[CDevMtr::DATA_SPEED])};
	
	auto output = pidSpdCtrl_Gyro.UpdatePidController(spd_Yaw, spd_measure);

	mtrOutputBuffer = output[0] * GIMBAL_YAW_MOTOR_GYRO_DIR;

    return APP_OK;
}

/**
 * @brief 机械模式更新输出
 * @param 编码器目标值 encoder
 * @retval EAppStatus
 * 
 */
EAppStatus CModGimbal::CComYaw::_UpdateOutput_Mec(float_t encoder){

	DataBuffer<float_t> encoder_measure = {static_cast<float_t>(yawInfo.encoder)};	// 测量值
	DataBuffer<float_t> encoder_target = {encoder};	// 目标值

	auto spd_Yaw = pidPosCtrl_Mec.UpdatePidController(encoder_target, encoder_measure);

	DataBuffer<float_t> spd_measure = {static_cast<float_t>(motor->motorData[CDevMtr::DATA_SPEED])};
	
	auto output = pidSpdCtrl_Mec.UpdatePidController(spd_Yaw, spd_measure);

	mtrOutputBuffer = output[0];

    return APP_OK;
}

} // namespace my_engineer