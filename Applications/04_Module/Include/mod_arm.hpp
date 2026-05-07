/******************************************************************************
 * @brief
 *
 * @file         mod_arm.hpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2026-01-27
 *
 * @copyright    Copyright (c) 2025
 *
 ******************************************************************************/
#ifndef MOD_ARM_HPP
#define MOD_ARM_HPP

/*-------------------------------------物理限位---------------------------------------------------*/
#define ARM_YAW_PHYSICAL_RANGE_MIN -115.3f
#define ARM_YAW_PHYSICAL_RANGE_MAX 222.7f
#define ARM_PITCH1_PHYSICAL_RANGE_MIN 0.0f
#define ARM_PITCH1_PHYSICAL_RANGE_MAX 96.f
#define ARM_PITCH2_PHYSICAL_RANGE_MIN 1.f
#define ARM_PITCH2_PHYSICAL_RANGE_MAX 120.f
#define ARM_PITCH3_PHYSICAL_RANGE_MIN -76.f
#define ARM_PITCH3_PHYSICAL_RANGE_MAX 0.f
#define ARM_ROLL_PHYSICAL_RANGE_MIN -3.0f
#define ARM_ROLL_PHYSICAL_RANGE_MAX 323.0f
#define ARM_END_PITCH_PHYSICAL_RANGE_MIN -90.0f
#define ARM_END_PITCH_PHYSICAL_RANGE_MAX 90.0f
#define ARM_END_GRIP_PHYSICAL_RANGE_MIN 0.0f
#define ARM_END_GRIP_PHYSICAL_RANGE_MAX 47.f
#define ARM_END_ROLL_PHYSICAL_RANGE_MIN  -224.f
#define ARM_END_ROLL_PHYSICAL_RANGE_MAX  90.f
#define ARM_END_GRIP_PHYSICAL_RANGE  47.f

/*-------------------------------------电机限位----------------------------------------------------*/
//原始限位编码器器范围
#define ARM_YAW_MOTOR_RANGE 61551
#define ARM_PITCH1_MOTOR_RANGE 17098
#define ARM_PITCH2_MOTOR_RANGE 65535
#define ARM_END_PITCH_MOTOR_RANGE 325993
#define ARM_END_GRIP_MOTOR_RANGE 1974934     ///(8192*22+10240+11000)

//与物理角度的映射关系
#define ARM_PITCH1_MOTOR_RATIO (ARM_PITCH1_MOTOR_RANGE / (ARM_PITCH1_PHYSICAL_RANGE_MAX - ARM_PITCH1_PHYSICAL_RANGE_MIN))
#define ARM_PITCH2_MOTOR_RATIO (ARM_PITCH2_MOTOR_RANGE / (ARM_PITCH2_PHYSICAL_RANGE_MAX - ARM_PITCH2_PHYSICAL_RANGE_MIN))
#define ARM_YAW_MOTOR_RATIO (ARM_YAW_MOTOR_RANGE / (ARM_YAW_PHYSICAL_RANGE_MAX - ARM_YAW_PHYSICAL_RANGE_MIN))
#define ARM_END_PITCH_MOTOR_RATIO (ARM_END_PITCH_MOTOR_RANGE / (ARM_END_PITCH_PHYSICAL_RANGE_MAX - ARM_END_PITCH_PHYSICAL_RANGE_MIN))
#define ARM_END_ROLL_MOTOR_RATIO 3524.07f
#define ARM_END_GRIP_MOTOR_RATIO (ARM_END_GRIP_MOTOR_RANGE / ARM_END_GRIP_PHYSICAL_RANGE) //编码器与物理距离转换比（单位mm）

/*-------------------------------------零点偏移--------------------------------------------------------*/
// 这里的offset都是物理的零点相对电机的零点的偏移值，电机的零点在物理的最小值
#define ARM_YAW_MOTOR_OFFSET -ARM_YAW_PHYSICAL_RANGE_MIN * ARM_YAW_MOTOR_RATIO
#define ARM_PITCH1_MOTOR_OFFSET -ARM_PITCH1_PHYSICAL_RANGE_MIN * ARM_PITCH1_MOTOR_RATIO
#define ARM_PITCH2_MOTOR_OFFSET -ARM_PITCH2_PHYSICAL_RANGE_MIN * ARM_PITCH2_MOTOR_RATIO
#define ARM_ROLL_MOTOR_OFFSET 0.f // 但这个比较特殊，测量这个就是从电机0位置到物理0位置总共的角度
#define ARM_END_PITCH_MOTOR_OFFSET -ARM_END_PITCH_PHYSICAL_RANGE_MIN * ARM_END_PITCH_MOTOR_RATIO

/*-------------------------------------方向设定---------------------------------------------------------*/
#define ARM_YAW_MOTOR_DIR 1
#define ARM_PITCH1_MOTOR_DIR -1//减小
#define ARM_PITCH2_MOTOR_DIR 1
#define ARM_ROLL_MOTOR_DIR 1
#define ARM_END_PITCH_MOTOR_L_DIR -1
#define ARM_END_PITCH_MOTOR_R_DIR 1
#define ARM_END_ROLL_MOTOR_L_DIR -1
#define ARM_END_ROLL_MOTOR_R_DIR -1
#define ARM_GRIP_MOTOR_DIR -1	// 张开方向与编码器值增大方向一致

//动态限位
#define ARM_P2_MAX_WHEN_P1_MIN 24.6f  ///< P1处于最小角度时，P2的最大可达角度

/*-------------------------------------初始化数据--------------------------------------------------------*/
#define ARM_YAW_INIT_ANGLE 0.0f
#define ARM_PITCH1_INIT_ANGLE 6.f
#define ARM_PITCH2_INIT_ANGLE 20.f
#define ARM_ROLL_INIT_ANGLE 0.0f
#define ARM_END_PITCH_INIT_ANGLE 0.0f
#define ARM_END_ROLL_INIT_ANGLE 0.0f
#define ARM_END_ROLL_STALL_ANGLE 90.0f
#define ARM_GRIP_INIT_LENGTH 0.0f

#define ARM_INIT_SAFE_PITCH1_ANGLE 70.0f
#define ARM_INIT_SAFE_PITCH2_ANGLE 70.0f
#define ARM_INIT_SAFE_PITCH3_ANGLE -11.0f

#define POSIT_JOINT1_YAW_MACH 53902
#define POSIT_JOINT1_YAW_MACH_PHY 0.f
#define ARM_YAW_MOTOR_RANGE_LHK 61551

#define POSIT_JOINT2_PITCH1_MACH 53021
#define POSIT_JOINT2_PITCH1_MACH_PHY 0.f
#define POSIT_JOINT2_PITCH1_INIT_PHY 6.0f

#define POSIT_JOINT3_PITCH2_MACH 50415 //12837
#define POSIT_JOINT3_PITCH2_MACH_PHY 0.f
#define POSIT_JOINT3_PITCH2_INIT_PHY 20.0f

/*-------------------------------------Pitch3 参数 (KT电机)------------------------------------------*/
#define ARM_PITCH3_MOTOR_DIR -1
#define ARM_PITCH3_MOTOR_RANGE 10942
#define ARM_PITCH3_INIT_ANGLE 0.0f
#define POSIT_JOINT4_PITCH3_MACH 10942
#define POSIT_JOINT4_PITCH3_MACH_PHY 0.f
#define POSIT_JOINT4_PITCH3_INIT_PHY 0.0f

#define POSIT_JOINT4_ROLL_OFFSET 0

#define POSIT_JOINT5_PITCH_END_MACH 0
#define POSIT_JONIT6_ROLL_END_MACH 0
#define POSIT_END_INIT 0
#define POSIT_END_PHY 205.0f

#define deg2rad(x) ((x) * 0.017453292519943295769236907684886)
#define rad2deg(x) ((x) * 57.295779513082320876798154814105)
/*------------------------------------- 夹爪的相关参数------------------------------------------*/
/// 自动控制速度常量（电机的转速rpm）
#define GRIP_OPEN_SPEED  12000.0f
#define GRIP_OPEN_SPEED_MIN  4000.0f
#define GRIP_CLOSE_SPEED  12000.0f

#define GRIP_OPEN_Stop_distance  2.0f
#define GRIP_CLOSE_Stop_distance  2.0f
#define GRIP_OPEN_Slow_distance 37.0f//减速的物理范围
#define GRIP_CLOSE_Slow_distance 28.0f//减速的范围

#define gripOpenStopPosit  (ARM_END_GRIP_MOTOR_RANGE - PhyPositToMtrPosit(GRIP_OPEN_Stop_distance))//刹车距离
#define gripOpenSlowPosit  (ARM_END_GRIP_MOTOR_RANGE - PhyPositToMtrPosit(GRIP_OPEN_Slow_distance))
#define gripCloseStopPosit (PhyPositToMtrPosit(GRIP_CLOSE_Stop_distance))
#define gripCloseSlowPosit (PhyPositToMtrPosit(GRIP_CLOSE_Slow_distance))//减速的编码范围
/*-------------------------------------重力补偿数据--------------------------------------------------------*/
#define PITCH1     0
#define PITCH2 	   1
#define END_PITCH  2
#define RECORD_MAX 10

#define MG6012_i36V3_Torque_Constant	0.175
#define MG8010_i36V2_Torque_Constant	0.15
#define DMJ4310_Torque_Constant			0.975	///< 对应电机的扭矩常数

#include "mod_common.hpp"


namespace my_engineer {

/**
 * @brief 机械臂模块类
 *
 */
class CModArm final: public CModBase {
	friend class CSystemCore;  // 允许 Core 层访问电机力矩数据用于力反馈
public:
	// 定义机械臂模块初始化参数结构体
	struct SModInitParam_Arm: public SModInitParam_Base {
		EDeviceID MotorID_Yaw = EDeviceID::DEV_NULL;
		EDeviceID MotorID_Pitch1 = EDeviceID::DEV_NULL;
		EDeviceID MotorID_Pitch2 = EDeviceID::DEV_NULL;
		EDeviceID MotorID_Pitch3 = EDeviceID::DEV_NULL;
		EDeviceID MotorID_Roll = EDeviceID::DEV_NULL;
		EDeviceID MotorID_End_L = EDeviceID::DEV_NULL;
		EDeviceID MotorID_End_R = EDeviceID::DEV_NULL;
		EDeviceID MotorID_Grip = EDeviceID::DEV_NULL;
		CInfCAN::CCanTxNode *MotorTxNode_Yaw; ///< 机械臂关节Yaw电机发送节点
		CInfCAN::CCanTxNode *MotorTxNode_Pitch1; ///< 机械臂关节Pitch电机1发送节点
		CInfCAN::CCanTxNode *MotorTxNode_Pitch2; ///< 机械臂关节Pitch电机2发送节点
		CInfCAN::CCanTxNode *MotorTxNode_Pitch3;
		CInfCAN::CCanTxNode *MotorTxNode_End_L;
		CInfCAN::CCanTxNode *MotorTxNode_End_R;
		CInfCAN::CCanTxNode *MotorTxNode_Grip; ///< 机械臂夹爪电机发送节点
		float_t MIT_Roll_kp = 0.0f; ///< MIT控制器比例系数
		float_t MIT_Roll_kd = 0.0f; ///< MIT控制器微分系数
		CAlgoPid::SAlgoInitParam_Pid YawPosPidParam;
		CAlgoPid::SAlgoInitParam_Pid YawSpdPidParam;
		CAlgoPid::SAlgoInitParam_Pid Pitch1PosPidParam;
		CAlgoPid::SAlgoInitParam_Pid Pitch1SpdPidParam;
		CAlgoPid::SAlgoInitParam_Pid Pitch2PosPidParam;
		CAlgoPid::SAlgoInitParam_Pid Pitch2SpdPidParam;
		CAlgoPid::SAlgoInitParam_Pid Pitch3PosPidParam;
		CAlgoPid::SAlgoInitParam_Pid Pitch3SpdPidParam;
		CAlgoPid::SAlgoInitParam_Pid endPosPidParam;
		CAlgoPid::SAlgoInitParam_Pid endSpdPidParam;
		CAlgoPid::SAlgoInitParam_Pid GripPosPidParam;///< 夹爪位置PID参数
		CAlgoPid::SAlgoInitParam_Pid GripSpdPidParam;

		struct SGripInitParam {
			float_t initSpeedMax_     = 6000.0f;   ///< 初始化最大速度
			float_t initSpeedMin_     = 2300.0f;   ///< 保底最低速度
			float_t initTorqueThresh_ = 1200.0f;   ///< 力矩开始减速的阈值
			float_t initTorqueRange_  = 2000.0f;   ///< 从全速减到最低速的力矩区间
		} GripInitParam;

		struct SGripDetectParam {
			float_t closeTorqueThresh = 1700.0f; ///< 滤波力矩开始减速的阈值
			float_t closeTorqueRange  = 1100.0f; ///< 从全速减到最低速的滤波力矩区间
			float_t closeSpeedMin     = 1000.0f; ///< 闭合时保底最低速度
			float_t detectTorque      = 2800.0f; ///< 滤波力矩超过此值即判定夹取成功
			float_t filterAlpha       = 0.95f;   ///< LowPassFilter滤波系数α
		} GripDetectParam;
	};

	// 定义机械臂信息结构体并实例化
	struct SArmInfo {
		EVarStatus isModuleAvailable = false; ///< 模块是否可用
		float_t angle_Yaw = 0.0f; ///< 机械臂关节Yaw角度
		float_t angle_Pitch1 = 0.0f; ///< 机械臂关节Pitch1角度
		float_t angle_Pitch2 = 0.0f; ///< 机械臂关节Pitch2角度
		float_t angle_Pitch3 = 0.0f; ///< 机械臂关节Pitch3角度
		float_t angle_Roll = 0.0f; ///< 机械臂关节Roll角度
		float_t angle_end_pitch = 0.0f; ///< 机械臂末端Pitch角度
		float_t angle_end_roll = 0.0f; ///< 机械臂末端Roll角度
		float_t length_grip = 0.0f; ///< 机械臂夹爪张开距离
		bool isAngleArrived_Yaw = false; ///< 机械臂关节Yaw角度是否到达
		bool isAngleArrived_Pitch1 = false; ///< 机械臂关节Pitch1角度是否到达
		bool isAngleArrived_Pitch2 = false; ///< 机械臂关节Pitch2角度是否到达
		bool isAngleArrived_Pitch3 = false; ///< 机械臂关节Pitch3角度是否到达
		bool isAngleArrived_Roll = false; ///< 机械臂关节Roll角度是否到达
		bool isAngleArrived_End_Pitch = false; ///< 机械臂末端Pitch角度是否到达
		bool isAngleArrived_End_Roll = false; ///< 机械臂末端Roll角度是否到达
		bool isAngleArrived_Grip = false; ///< 机械臂夹爪角度是否夹取
		bool isGripped = false; ///< 夹爪是否处于堵转夹持状态
		float_t holdLength_grip = 0.0f; ///< 夹取保持位置（物理距离 mm）
		enum class EGripState : uint8_t { RELEASE = 0, HOLD = 1 };
		EGripState gripState = EGripState::RELEASE;
	} armInfo;

	// 定义机械臂控制命令结构体并实例化
	struct SArmCmd {
		bool isAutoCtrl = false; ///< 是否自动控制
		bool isCustomCtrl = false; ///< 是否自定义控制
		bool reGripCmd = false;  ///<二次夹紧
		float_t set_angle_Yaw = 0.0f; ///< 机械臂关节Yaw角度设定
		float_t set_angle_Pitch1 = 0.0f; ///< 机械臂关节Pitch1角度设定
		float_t set_angle_Pitch2 = 0.0f; ///< 机械臂关节Pitch2角度设定
		float_t set_angle_Pitch3 = 0.0f; ///< 机械臂关节Pitch3角度设定
		float_t set_angle_Roll = 0.0f; ///< 机械臂关节Roll角度设定
		float_t set_angle_end_pitch = 0.0f; ///< 机械臂末端Pitch角度设定
		float_t set_angle_end_roll = 0.0f; ///< 机械臂末端Roll角度设定
		float_t set_length_grip = 0.0f; ///< 机械臂夹爪距离设定（自动任务直接设定）
		float_t set_speed_grip = 0.0f; ///< 机械臂夹爪速度设定（速度环模式）
		bool gripClose = false;           ///< 手动闭合标志（Core层设置）
		bool gripOpen = false;            ///< 手动张开标志（Core层设置）
	} armCmd;

	CModArm() = default;

	CAlgoTrajPlayback initTraj_;                     ///< 初始化轨迹规划器实例
	float_t initTrajTime_ = 0.0f;                    ///< 当前轨迹时间 (s)
	bool isInitTrajActive_ = false;                  ///< 轨迹是否正在执行

	// 定义带参数的模块构造函数，创建模块时自动调用初始化函数
	explicit CModArm(SModInitParam_Arm &param) { InitModule(param); }        ///<要求**带参数**的构造函数要用explicit修饰，防止隐式转换

	// 模块析构函数
	~CModArm() final { UnregisterModule_(); }

	// 初始化模块
	EAppStatus InitModule(SModInitParam_Base &param) final;

	uint8_t should_limit_yaw = 0; ///< 是否限制Yaw角度

private:
	// 定义机械臂Yaw关节组件类并实例化
	class CComJoint: public CComponentBase {
	public:
		enum {Y = 0, P1 = 1, P2 = 2, P3 = 3};
		const int32_t rangeLimit_yaw 		= ARM_YAW_MOTOR_RANGE_LHK; ///< Yaw关节电机范围限制
		const int32_t rangeLimit_pitch1 = ARM_PITCH1_MOTOR_RANGE; ///< Pitch1关节电机范围限制
		const int32_t rangeLimit_pitch2 = ARM_PITCH2_MOTOR_RANGE; ///< Pitch2关节电机范围限制
		const int32_t rangeLimit_pitch3 = ARM_PITCH3_MOTOR_RANGE; ///< Pitch3关节电机范围限制

		// 定义Yaw关节信息结构体
		struct SYawInfo {
			int32_t posit_yaw = 0.0f;           ///< Yaw关节当前位置
			int32_t posit_pitch1 = 0.0f;     ///< Pitch1关节当前位置
			int32_t posit_pitch2 = 0.0f;     ///< Pitch2关节当前位置
			int32_t posit_pitch3 = 0.0f;     ///< Pitch3关节当前位置
			bool isPositArrived_yaw = false;    ///< Yaw位置是否到达目标
			bool isPositArrived_pitch1 = false; ///< Pitch1位置是否到达
			bool isPositArrived_pitch2 = false; ///< Pitch2位置是否到达
			bool isPositArrived_pitch3 = false; ///< Pitch3位置是否到达
		} jointInfo;

		// 定义Yaw关节控制命令结构体
		struct SYawCmd {
			int32_t setPosit_yaw = 0.0f;        ///< Yaw关节目标位置
			int32_t setPosit_pitch1 = POSIT_JOINT2_PITCH1_INIT_PHY; ///< Pitch1关节目标位置
			int32_t setPosit_pitch2 = POSIT_JOINT3_PITCH2_INIT_PHY; ///< Pitch2关节目标位置
			int32_t setPosit_pitch3 = POSIT_JOINT4_PITCH3_INIT_PHY; ///< Pitch3关节目标位置
		} jointCmd;

		// PID控制器
		CAlgoPid pidPosCtrl_yaw;
		CAlgoPid pidSpdCtrl_yaw;
		CAlgoPid pidPosCtrl_pitch1;
		CAlgoPid pidSpdCtrl_pitch1;
		CAlgoPid pidPosCtrl_pitch2;
		CAlgoPid pidSpdCtrl_pitch2;
		CAlgoPid pidPosCtrl_pitch3;
		CAlgoPid pidSpdCtrl_pitch3;

		// 电机数据输出缓冲区
		std::array<int16_t, 4> mtrOutputBuffer = {0};

		CAlgoTrajPlayback initTraj_;
		float_t initTrajTime_ = 0.0f;
		bool isInitTrajActive_ = false;

		// 重补输出
		float_t Grav_Pitch1_Out = 0;
		float_t Grav_Pitch2_Out = 0;
		float_t g_pitch1 = 0;
		float_t g_pitch2 = 0;

		// 电机实例指针
		CDevMtr* motor[4] = {nullptr};

		// 父类指针，用于访问其他组件
		CModArm* parentModule = nullptr;

		// 物理位置转换为电机位置
		static int32_t PhyPositToMtrPosit_yaw(float_t phyPosit);
		static int32_t PhyPositToMtrPosit_pitch1(float_t phyPosit);
		static int32_t PhyPositToMtrPosit_pitch2(float_t phyPosit);
		static int32_t PhyPositToMtrPosit_pitch3(float_t phyPosit);

		// 电机位置转换为物理位置
		static float_t MtrPositToPhyPosit_yaw(int32_t mtrPosit);
		static float_t MtrPositToPhyPosit_pitch1(int32_t mtrPosit);
		static float_t MtrPositToPhyPosit_pitch2(int32_t mtrPosit);
		static float_t MtrPositToPhyPosit_pitch3(int32_t mtrPosit);

		// 初始化组件
		EAppStatus InitComponent(SModInitParam_Base &param) final;

		// 更新组件
		EAppStatus UpdateComponent() final;

		// 输出更新函数
		EAppStatus _UpdateOutput(float_t posit_yaw, float_t posit_pitch1, float_t posit_pitch2, float_t posit_pitch3);

		// 单独电机输出更新函数
		EAppStatus _UpdateOutput_Yaw(float_t posit_yaw);
		EAppStatus _UpdateOutput_Pitch1(float_t posit_pitch1);
		EAppStatus _UpdateOutput_Pitch2(float_t posit_pitch2);
		EAppStatus _UpdateOutput_Pitch3(float_t posit_pitch3);

		// 电机can发送节点
		std::array<CInfCAN::CCanTxNode*, 4> mtrCanTxNode;

	} comjoint_;

	// 定义机械臂Roll关节组件类并实例化
	class CComRoll: public CComponentBase {
	public:
		// 定义Roll关节信息结构体
		struct SRollInfo {
			float_t angle = 0.0f;           ///< Roll关节当前角度
			bool isAngleArrived = false;    ///< Roll角度是否到达目标
		} rollInfo;

		// 定义Roll关节控制命令结构体
		struct SRollCmd {
			float_t setAngle = 0.0f;        ///< Roll关节目标角度
		} rollCmd;

		// MIT控制结构体
		struct SMitCtrl {
			float_t kp = 0.0f;
			float_t kd = 0.0f;
			float_t q = 0.0f;
			float_t dq = 0.0f;
			float_t tau = 0.0f;
		} mitCtrl;

		// 电机实例指针
		CDevMtr* motor = nullptr;

		// 重补输出
		float_t Grav_Roll_Out = 0;

		static float_t MtrAngleToPhyAngle(float_t angle) {
			// 将电机角度转换为物理角度
			return angle;
		}

		static float_t PhyAngleToMtrAngle(float_t angle) {
			// 将物理角度转换为电机角度
			return angle;
		}

		// 初始化组件
		EAppStatus InitComponent(SModInitParam_Base &param) final;

		// 更新组件
		EAppStatus UpdateComponent() final;
	} comRoll_;

	// 定义机械臂末端组件类并实例化
	class CComEnd: public CComponentBase {
	public:

		enum { L = 0, R = 1 };
		enum class EEndInitState : uint8_t {
			PITCH = 0,
			ROLL = 1,
			DONE = 2
		};

		const int32_t rangeLimit_Pitch = ARM_END_PITCH_MOTOR_RANGE; ///< 电机位置范围限制

		// 定义机械臂末端信息结构体并实例化~
		struct SEndInfo {
			int32_t posit_Pitch = 0;    ///< End Posit Pitch
			int32_t posit_Roll = 0;    ///< End Posit Roll
			bool isPositArrived_Pitch = false; ///< End Posit Arrived Pitch
			bool isPositArrived_Roll = false; ///< End Posit Arrived Roll
		} endInfo;

		// 定义机械臂末端控制命令结构体并实例化
		struct SEndCmd {
			int32_t setPosit_Pitch = 0;    ///< End Posit Set Pitch
			int32_t setPosit_Roll = 0;    ///< End Posit Set Roll
		} endCmd;

		// 电机实例指针数组
		std::array<CDevMtr*, 2> motor = {nullptr};

		// 定义机械臂末端PID控制器
		CAlgoPid pidPosCtrl;
		CAlgoPid pidSpdCtrl;

		int32_t rollZeroOffset = 0; ///< Roll零点偏移（初始化时标定）
		bool rollCalibrated_ = false; ///< Roll是否已完成标定
		bool pitchCalibrated_ = false; ///< Pitch是否已完成标定
		EEndInitState initState_ = EEndInitState::PITCH;
		uint16_t initStateTick_ = 0;
		// 电机数据输出缓冲区
		std::array<int16_t, 2> mtrOutputBuffer = {0};

		// 初始化组件
		EAppStatus InitComponent(SModInitParam_Base &param) final;

		// 更新组件
		EAppStatus UpdateComponent() final;

		// 物理位置转换为电机位置: Pitch
		static int32_t PhyPositToMtrPosit_Pitch(float_t phyPosit);

		// 电机位置转换为物理位置: Pitch
		static float_t MtrPositToPhyPosit_Pitch(int32_t mtrPosit);

		// 物理位置转换为电机位置: Roll
		int32_t PhyPositToMtrPosit_Roll(float_t phyPosit);

		// 电机位置转换为物理位置: Roll
		float_t MtrPositToPhyPosit_Roll(int32_t mtrPosit);

		// 输出更新函数
		EAppStatus _UpdateOutput(float_t posit_Pitch, float_t posit_Roll);

		// 电机can发送节点
		std::array<CInfCAN::CCanTxNode*, 2> mtrCanTxNode;


	} comEnd_;

	class CComGrip: public CComponentBase {
	public:

		const int32_t rangeLimit_Grip = ARM_END_GRIP_MOTOR_RANGE; ///< 夹爪电机位置范围限制
		// 定义夹爪信息结构体
		struct SGripInfo {
			enum class EGripState : uint8_t { RELEASE = 0, HOLD = 1 };
			EGripState state = EGripState::RELEASE;	///< 夹爪控制子状态
			int32_t posit_grip = 0;           	///< 夹爪当前位置
			int32_t holdPosit_Grip = 0;			///< 记忆夹持位置
			bool isGripped = false; 			///< 是否夹住
		} gripInfo;

		// 定义夹爪控制命令结构体
		struct SGripCmd {
			int32_t setPosit_grip = 0;        ///< 夹爪目标位置（编码器），位置环模式使用
			float_t setSpeed_grip = 0.0f;     ///< 夹爪目标速度（正=张开，负=闭合），速度环模式使用
			bool cmdReGrip = false;           ///< 二次夹紧
			bool cmdClose = false;            ///< 手动闭合标志（Core层设置）
			bool cmdOpen = false;             ///< 手动张开标志（Core层设置）
		} gripCmd;

		struct SGripinitParam {
			float_t initSpeedMax_     = 6000.0f;   ///< 初始化最大速度
			float_t initSpeedMin_     = 2300.0f;   ///< 保底最低速度
			float_t initTorqueThresh_ = 1000.0f;   ///< 力矩开始减速的阈值
			float_t initTorqueRange_  = 2000.0f;   ///< 从全速减到最低速的力矩区间
		} GripinitParam_;

		struct SGripCmdDetect {
			bool edgeReady          = false;    ///< 边沿检测就绪标志
			float_t filteredTorque  = 0.0f;     ///< IIR滤波后的力矩值
			float_t closeTorqueThresh = 1700.0f;///< 滤波力矩开始减速的阈值
			float_t closeTorqueRange  = 1100.0f;///< 从全速减到最低速的滤波力矩区间
			float_t closeSpeedMin     = 5000.0f;///< 闭合时保底最低速度
			float_t detectTorque      = 2800.0f;///< 滤波力矩超过此值即判定夹取成功
			float_t filterAlpha       = 0.95f;  ///< LowPassFilter滤波系数α
		} gripDetect_;

		// PID控制器
		CAlgoPid pidPosCtrl;
		CAlgoPid pidSpdCtrl;

		// 电机实例指针
		CDevMtr* motor = nullptr;

		// 电机数据输出缓冲区
		int16_t mtrOutputBuffer = 0;
		float_t speedTargetFiltered_ = 0.0f;
		float_t speedMeasuredFiltered_ = 0.0f;
		uint16_t initTick_ = 0;

		static float_t MtrPositToPhyPosit(float_t mtrPosit);

		static int32_t PhyPositToMtrPosit(float_t phyPosit);

		// 初始化组件
		EAppStatus InitComponent(SModInitParam_Base &param) final;


		float_t CalcGripSlowSpeed(float_t maxSpeed, float_t minSpeed, int32_t remainToStop, int32_t slowBand);

		// 更新组件
		EAppStatus UpdateComponent() final;

		// 输出更新函数（位置环）
		EAppStatus _UpdateOutput(float_t gripTarget);

		// 输出更新函数（速度环，复用内环pidSpdCtrl）
		EAppStatus _UpdateOutputSpd(float_t speedTarget);

		// 电机can发送节点
		CInfCAN::CCanTxNode* mtrCanTxNode;

	} comGrip_;

	// 重写基类函数
	void UpdateHandler_() final;
	void HeartbeatHandler_() final;
	EAppStatus CreateModuleTask_() final;

	// 声明机械臂模块任务函数
	static void StartArmModuleTask(void *argument);

	// 控制量限制函数
	EAppStatus RestrictArmCommand_();

	// 重补输出更新函数
	EAppStatus Grav_Compemsation_Pitch1();
	EAppStatus Grav_Compemsation_Pitch2();
	EAppStatus Grav_Compemsation_Roll();

};

///< 全局变量
extern bool Need_Grav_Compensation; ///< 是否启用重力补偿
extern bool Is_Recording_ArmTorque; ///<是否正在记录数据
extern DataBuffer<float_t> arm_Info[3][10]; ///<用于记录臂的力矩，三个关节，10个数据点
extern uint16_t index; ///< 数组索引
extern bool is_record; ///< 是否要记录数据

} // namespace my_engineer

#endif // MOD_ARM_HPP
