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
#include "algo_other.hpp"

// -------------------- 大yaw组件参数 ---------------------
#define GIMBAL_YAW_MOTOR_GYRO_DIR -1		///< 陀螺仪模式输出方向
#define GIMBAL_YAW_MOTOR_MEC_DIR  -1		///< 机械模式输出方向
#define GINBAL_FRONT_MOTOR_ANGLE 1405	///< 目前给0 后续改成朝前时的编码器值
#define GIMBAL_YAW_INIT_ANGLE	GINBAL_FRONT_MOTOR_ANGLE	///< 初始化编码器值（朝前）

// -------------------- 升降组件参数 ---------------------
#define GIMBAL_LIFT_PHYSICAL_RANGE 100.0f       ///< 升降物理行程 (mm)
#define GIMBAL_LIFT_MOTOR_RANGE 184000          ///< 升降电机编码器范围
#define GIMBAL_LIFT_MOTOR_RATIO (GIMBAL_LIFT_MOTOR_RANGE / GIMBAL_LIFT_PHYSICAL_RANGE)
#define GIMBAL_LIFT_MOTOR_DIR_L 1               ///< 左电机方向
#define GIMBAL_LIFT_MOTOR_DIR_R -1              ///< 右电机方向

// -------------------- 俯仰组件参数 ---------------------
#define GIMBAL_PITCH_PHYSICAL_RANGE 90.0f       ///< 俯仰物理行程 (degree)
#define GIMBAL_PITCH_MOTOR_RANGE 73728          ///< 俯仰电机编码器范围
#define GIMBAL_PITCH_MOTOR_RATIO (GIMBAL_PITCH_MOTOR_RANGE / GIMBAL_PITCH_PHYSICAL_RANGE)
#define GIMBAL_PITCH_MOTOR_DIR 1                ///< 俯仰电机方向

// -------------------- 平动组件参数 ---------------------
#define GIMBAL_HORIZONTAL_PHYSICAL 1.f			///< 平动物理行程 (mm)
#define GIMBAL_HORIZONTAL_MOTOR_RANGE 100		///< 平动电机编码器范围
#define GIMBAL_HORIZONTAL_MOTOR_RATIO (GIMBAL_HORIZONTAL_MOTOR_RANGE / GIMBAL_HORIZONTAL_PHYSICAL)
#define GIMBAL_HORIZONTAL_MOTOR_DIR 1			///< 平动电机方向

// -------------------- 存矿组件参数 ---------------------
#define GIMBAL_STORAGE_PHYSICAL_RANGE 90.0f       ///< 存矿物理行程 (degree)
#define GIMBAL_STORAGE_MOTOR_RANGE 1          ///< 升降电机编码器范围
#define GIMBAL_STORAGE_MOTOR_RATIO (GIMBAL_STORAGE_MOTOR_RANGE / GIMBAL_STORAGE_PHYSICAL_RANGE)
#define GIMBAL_STORAGE_MOTOR_DIR_F 1               ///< 前电机方向
#define GIMBAL_STORAGE_MOTOR_DIR_B -1              ///< 后电机方向

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
		// EDeviceID liftMotorID_L = EDeviceID::DEV_NULL;
		// EDeviceID liftMotorID_R = EDeviceID::DEV_NULL;
		// EDeviceID pitchMotorID = EDeviceID::DEV_NULL;
		// EDeviceID storageMotorID_F = EDeviceID::DEV_NULL;
		// EDeviceID storageMotorID_B = EDeviceID::DEV_NULL;
		// EDeviceID horizontalMotorID = EDeviceID::DEV_GIMBAL_MTR_HORIZONTAL;
		CInfCAN::CCanTxNode *MotorTxNode_Yaw = nullptr; ///< 云台Yaw电机发送节点
		// CInfCAN::CCanTxNode *pitchMotorTxNode = nullptr;
		// CInfCAN::CCanTxNode *liftMotorTxNode_L = nullptr;
		// CInfCAN::CCanTxNode *liftMotorTxNode_R = nullptr;
		// CInfCAN::CCanTxNode *horizontalMotorTxNode = nullptr;
		// CInfCAN::CCanTxNode *storageMotorTxNode_F = nullptr;
		// CInfCAN::CCanTxNode *storageMotorTxNode_B = nullptr;
		CAlgoPid::SAlgoInitParam_Pid YawPosPidParam_Gyro;
		CAlgoPid::SAlgoInitParam_Pid YawSpdPidParam_Gyro;	// 陀螺仪模式pid
		CAlgoPid::SAlgoInitParam_Pid YawPosPidParam_Mec;
		CAlgoPid::SAlgoInitParam_Pid YawSpdPidParam_Mec;	// 机械模式pid
		// CAlgoPid::SAlgoInitParam_Pid liftPosPidParam;
		// CAlgoPid::SAlgoInitParam_Pid liftSpdPidParam;
		// CAlgoPid::SAlgoInitParam_Pid pitchPosPidParam;
		// CAlgoPid::SAlgoInitParam_Pid pitchSpdPidParam;
		// CAlgoPid::SAlgoInitParam_Pid horizontalPosPidParam;
		// CAlgoPid::SAlgoInitParam_Pid horizontalSpdPidParam;
		// CAlgoPid::SAlgoInitParam_Pid storagePosPidParam;
		// CAlgoPid::SAlgoInitParam_Pid storageSpdPidParam;
	};

	// 云台信息
	struct SGimbalInfo{
		EVarStatus isModuleAvailable = false;
		float_t posit_yaw = 0.0f;	// yaw轴角度
		int32_t encoder_yaw = 0.f;	// yaw轴编码器
		bool isPositArrived_Yaw = false;
		// float_t posit_lift = 0.0f;
		// bool isPositArrived_Lift = false;
		// float_t posit_pitch = 0.0f;
		// bool isPositArrived_Pitch = false;
		// float_t posit_horizontal = 0.f;
		// bool isPositArrived_Horizontal = false;
		// float_t posit_storage = 0.f;
		// bool isPositArrived_Storage = false;
	} gimbalInfo;

	// 云台控制命令
	struct SGimbalCmd{
		EVarStatus isAutoCtrl = false;
		float_t set_posit_yaw = 0.f;	// 角度目标值
		int32_t set_encoder_yaw = 0.f;	// 编码器目标值
		// float_t set_posit_lift = 0.0f;	// 抬升目标值
		// float_t set_posit_pitch = 0.0f;	// pitch目标值
		// float_t set_posit_horizontal = 0.f;	// 平动目标值
		// float_t set_posit_storage = 0.f;	// 存矿目标值
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
			float_t setPosit = 0;	// 陀螺仪目标角度
			int16_t setEncoder = 0;	// 编码器目标值
		} yawCmd;

		CDevMtr* motor = nullptr;
		CAlgoPid pidPosCtrl_Gyro;
		CAlgoPid pidSpdCtrl_Gyro;
		CAlgoPid pidPosCtrl_Mec;
		CAlgoPid pidSpdCtrl_Mec;
		// 底盘跟云台pid
		CAlgoPid pidFollowGimbal_Pos;
		CAlgoPid pidFollowGimbal_Spd;
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

	// // 升降组件 (双电机同步)
	// class CComLift: public CComponentBase{
	// public:

	// 	enum { L = 0, R = 1 };

	// 	const int32_t rangeLimit = GIMBAL_LIFT_MOTOR_RANGE;

	// 	struct SLiftInfo{
	// 		int32_t posit = 0;
	// 		bool isPositArrived = false;
	// 	} liftInfo;

	// 	struct SLiftCmd{
	// 		int32_t setPosit = 0;
	// 	} liftCmd;

	// 	std::array<CDevMtr*, 2> motor = {nullptr, nullptr};
	// 	CAlgoPid pidPosCtrl;
	// 	CAlgoPid pidSpdCtrl;
	// 	std::array<int16_t, 2> mtrOutputBuffer = {0, 0};
	// 	std::array<CInfCAN::CCanTxNode*, 2> mtrCanTxNode = {nullptr, nullptr};

	// 	EAppStatus InitComponent(SModInitParam_Base &param) final;
	// 	EAppStatus UpdateComponent() final;
	// 	static int32_t PhyPositToMtrPosit(float_t phyPosit);
	// 	static float_t MtrPositToPhyPosit(int32_t mtrPosit);
	// 	EAppStatus _UpdateOutput(float_t posit);

	// } comLift_;

	// class CComPitch: public CComponentBase{
	// public:

	// 	const int32_t rangeLimit = GIMBAL_PITCH_MOTOR_RANGE;

	// 	struct SPitchInfo{
	// 		int32_t posit = 0;
	// 		bool isPositArrived = false;
	// 	} pitchInfo;

	// 	struct SPitchCmd{
	// 		int32_t setPosit = 0;
	// 	} pitchCmd;

	// 	CDevMtr *motor = nullptr;

	// 	CAlgoPid pidPosCtrl;
	// 	CAlgoPid pidSpdCtrl;

	// 	std::array<int16_t, 1> mtrOutputBuffer = {0};
	// 	CInfCAN::CCanTxNode* mtrCanTxNode = nullptr;

	// 	EAppStatus InitComponent(SModInitParam_Base &param) final;

	// 	EAppStatus UpdateComponent() final;

	// 	static int32_t PhyPositToMtrPosit(float_t phyPosit);

	// 	static float_t MtrPositToPhyPosit(int32_t mtrPosit);

	// 	EAppStatus _UpdateOutput(float_t posit);

	// } comPitch_;

	// class CComHorizontal: public CComponentBase{
	// public:

	// 	const int32_t rangeLimit = GIMBAL_HORIZONTAL_MOTOR_RANGE;

	// 	struct SHorizontalInfo{
	// 		int32_t posit = 0;
	// 		bool isPositArrived = false;
	// 	} horizontalInfo;

	// 	struct SHorizontalCmd{
	// 		int32_t setPosit = 0;
	// 	} horizontalCmd;

	// 	CDevMtr *motor = nullptr;

	// 	CAlgoPid pidPosCtrl;
	// 	CAlgoPid pidSpdCtrl;

	// 	std::array<int16_t, 1> mtrOutputBuffer = {0};
	// 	CInfCAN::CCanTxNode* mtrCanTxNode = nullptr;

	// 	EAppStatus InitComponent(SModInitParam_Base &param) final;

	// 	EAppStatus UpdateComponent() final;

	// 	static int32_t PhyPositToMtrPosit(float_t phyPosit);

	// 	static float_t MtrPositToPhyPosit(int32_t mtrPosit);

	// 	EAppStatus _UpdateOutput(float_t posit);

	// } comHorizontal_;

	// // 存矿组件 (双电机镜像)
	// class CComStorage: public CComponentBase{
	// public:

	// 	enum { F = 0, B = 1 };

	// 	const int32_t rangeLimit = GIMBAL_STORAGE_MOTOR_RANGE;

	// 	struct SStorageInfo{
	// 		int32_t posit_F = 0;
	// 		bool isPositArrived_F = false;	// 前
	// 		int32_t posit_B = 0;
	// 		bool isPositArrived_B = false;	// 后
	// 	} storageInfo;

	// 	struct SStorageCmd{
	// 		int32_t setPosit = 0;
	// 	} storageCmd;

	// 	std::array<CDevMtr*, 2> motor = {nullptr, nullptr};
	// 	CAlgoPid pidPosCtrl;
	// 	CAlgoPid pidSpdCtrl;
	// 	std::array<int16_t, 2> mtrOutputBuffer = {0, 0};
	// 	std::array<CInfCAN::CCanTxNode*, 2> mtrCanTxNode = {nullptr, nullptr};

	// 	EAppStatus InitComponent(SModInitParam_Base &param) final;
	// 	EAppStatus UpdateComponent() final;
	// 	static int32_t PhyPositToMtrPosit(float_t phyPosit);
	// 	static float_t MtrPositToPhyPosit(int32_t mtrPosit);
	// 	EAppStatus _UpdateOutput(float_t posit);

	// } comStorage_;

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
