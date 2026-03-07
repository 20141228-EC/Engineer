/**
 * @file mod_gimbal.hpp
 * @author sllllr
 * @brief 云台模块
 * @version 1.0
 * @date 2026-03-06
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef MOD_GIMBAL_HPP
#define MOD_GIMBAL_HPP

#include "mod_common.hpp"
#include "algo_ave_filter.hpp"

#define GIMBAL_YAW_MOTOR_DIR 1		///< 目前给1
#define GINBAL_FRONT_MOTOR_ANGLE 0	///< 目前给0 后续改成朝前时的编码器值
#define GIMBAL_YAW_INIT_ANGLE	GINBAL_FRONT_MOTOR_ANGLE	///< 初始化编码器值（朝前）

#define deg2rad(x) ((x) * 0.017453292519943295769236907684886)
#define rad2deg(x) ((x) * 57.295779513082320876798154814105)

namespace my_engineer {

class CModGimbal final: public CModBase{
public:

	// 云台模块初始化参数
	struct SModInitParam_Gimbal: public SModInitParam_Base{
		EDeviceID memsDevID = EDeviceID::DEV_NULL;	///< 陀螺仪设备
		EAlgoID FilterID = EAlgoID::ALGO_NULL;	///< 滤波算法
		EDeviceID yawMotorID = EDeviceID::DEV_NULL;
		CInfCAN::CCanTxNode *MotorTxNode_Yaw; ///< 云台Yaw电机发送节点
		CAlgoPid::SAlgoInitParam_Pid YawPosPidParam_Gyro;
		CAlgoPid::SAlgoInitParam_Pid YawSpdPidParam_Gyro;	// 陀螺仪模式pid
		CAlgoPid::SAlgoInitParam_Pid YawPosPidParam_Mec;
		CAlgoPid::SAlgoInitParam_Pid YawSpdPidParam_Mec;	// 机械模式pid
	};

	// 云台信息
	struct SGimbalInfo{
		EVarStatus isModuleAvailable = false;
		float_t posit_yaw = 0.0f;	// yaw轴角度
		int32_t encoder_yaw = 0.f;	// yaw轴编码器
		bool isPositArrived = false;
	} gimbalInfo;

	// 云台控制命令
	struct SGimbalCmd{
		EVarStatus isAutoCtrl = false;
		float_t set_posit_yaw = 0.f;	// 角度目标值
		int32_t set_encoder_yaw = 0.f;	// 编码器目标值
	} gimbalCmd;

	CModGimbal() = default;

	explicit CModGimbal(SModInitParam_Gimbal &param) { InitModule(param); }

	// 模块析构函数
	~CModGimbal() final { UnregisterModule_(); }

	// 初始化模块
	EAppStatus InitModule(SModInitParam_Base &param) final;

private:
	// 大yaw组件
	class CComYaw: public CComponentBase{
	public:

		struct SYawInfo{
			float_t posit = 0;		// 陀螺仪角度
			int32_t encoder = 0;	// 编码器值
			bool isPositArrived = false;
		} yawInfo;

		struct SYawCmd{
			int32_t setPosit = 0;	// 陀螺仪目标角度
			int16_t setEncoder = 0;	// 编码器目标值
		} yawCmd;

		CDevMtr* motor = nullptr;
		CAlgoPid pidPosCtrl_Gyro;
		CAlgoPid pidSpdCtrl_Gyro;
		CAlgoPid pidPosCtrl_Mec;
		CAlgoPid pidSpdCtrl_Mec;
		int16_t mtrOutputBuffer = 0;	// 电机输出缓冲区
		CInfCAN::CCanTxNode* mtrCanTxNode = nullptr;

		// 传感器实例指针
        CMemsBase *mems = nullptr;

		// 互补滤波算法实例指针
    	CAlgo_IMU_Ave *filter = nullptr;

		EAppStatus InitComponent(SModInitParam_Base &param) final;
		EAppStatus UpdateComponent() final;
		EAppStatus _UpdateOutput_Mec(float_t encoder);
		EAppStatus _UpdateOutput_Gyro(float_t encoder);

	} comYaw_;


	// 重写基类函数
    void UpdateHandler_() final;
    void HeartbeatHandler_() final;
    EAppStatus CreateModuleTask_() final;

    static void StartGimbalModuleTask(void *argument);

    // 声明控制量限制函数(负责对控制量进行限幅，在上面那个任务中进行调用)
    EAppStatus RestrictGimbalCommand_();

};

} // namespace my_engineer

#endif // MOD_GIMBAL_HPP
