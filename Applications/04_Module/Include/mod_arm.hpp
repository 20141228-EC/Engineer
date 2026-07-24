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
#define ARM_YAW_PHYSICAL_RANGE_MIN -128.f
#define ARM_YAW_PHYSICAL_RANGE_MAX 180.f
#define ARM_PITCH1_PHYSICAL_RANGE_MIN 0.0f
#define ARM_PITCH1_PHYSICAL_RANGE_MAX 91.f
#define ARM_PITCH2_PHYSICAL_RANGE_MIN 1.f
#define ARM_PITCH2_PHYSICAL_RANGE_MAX 141.f
#define ARM_ROLL_PHYSICAL_RANGE_MIN -148.0f
#define ARM_ROLL_PHYSICAL_RANGE_MAX 152.0f
#define ARM_END_PITCH_PHYSICAL_RANGE_MIN -91.0f
#define ARM_END_PITCH_PHYSICAL_RANGE_MAX 90.0f
#define ARM_END_GRIP_PHYSICAL_RANGE_MIN 0.0f
#define ARM_END_GRIP_PHYSICAL_RANGE_MAX 30.f
#define ARM_END_ROLL_PHYSICAL_RANGE_MIN  -103.f
#define ARM_END_ROLL_PHYSICAL_RANGE_MAX  221.f
#define ARM_END_GRIP_PHYSICAL_RANGE  30.f

/*-------------------------------------电机限位----------------------------------------------------*/
//原始限位编码器器范围
#define ARM_YAW_MOTOR_RANGE 62469
#define ARM_PITCH1_MOTOR_RANGE 15347
#define ARM_PITCH2_MOTOR_RANGE 38637
#define ARM_END_GRIP_MOTOR_RANGE 300000

//与物理角度的映射关系
#define ARM_PITCH1_MOTOR_RATIO (ARM_PITCH1_MOTOR_RANGE / (ARM_PITCH1_PHYSICAL_RANGE_MAX - ARM_PITCH1_PHYSICAL_RANGE_MIN))
#define ARM_PITCH2_MOTOR_RATIO (ARM_PITCH2_MOTOR_RANGE / (ARM_PITCH2_PHYSICAL_RANGE_MAX - ARM_PITCH2_PHYSICAL_RANGE_MIN))
#define ARM_YAW_MOTOR_RATIO (ARM_YAW_MOTOR_RANGE / (ARM_YAW_PHYSICAL_RANGE_MAX - ARM_YAW_PHYSICAL_RANGE_MIN))
#define ARM_END_GRIP_MOTOR_RATIO (ARM_END_GRIP_MOTOR_RANGE / ARM_END_GRIP_PHYSICAL_RANGE) //编码器与物理距离转换比（单位mm）

/*-------------------------------------零点偏移--------------------------------------------------------*/
// 这里的offset都是物理的零点相对电机的零点的偏移值，电机的零点在物理的最小值
#define ARM_YAW_MOTOR_OFFSET -ARM_YAW_PHYSICAL_RANGE_MIN * ARM_YAW_MOTOR_RATIO
#define ARM_PITCH1_MOTOR_OFFSET -ARM_PITCH1_PHYSICAL_RANGE_MIN * ARM_PITCH1_MOTOR_RATIO
#define ARM_PITCH2_MOTOR_OFFSET -ARM_PITCH2_PHYSICAL_RANGE_MIN * ARM_PITCH2_MOTOR_RATIO
#define ARM_ROLL_MOTOR_OFFSET 0.f // 但这个比较特殊，测量这个就是从电机0位置到物理0位置总共的角度
#define ARM_END_PITCH_MOTOR_OFFSET -ARM_END_PITCH_PHYSICAL_RANGE_MIN * ARM_END_PITCH_MOTOR_RATIO

#define ARM_PITCH2_MOTOR_REDUCTION_RATIO 1 //同步带减速比，无减速带
/*-------------------------------------方向设定---------------------------------------------------------*/
#define ARM_YAW_MOTOR_DIR -1
#define ARM_PITCH1_MOTOR_DIR 1
#define ARM_PITCH2_MOTOR_DIR 1
#define ARM_ROLL_MOTOR_DIR 1
#define ARM_END_PITCH_MOTOR_DIR 1
#define ARM_END_ROLL_MOTOR_DIR  -1	
#define ARM_GRIP_POS_DIR  -1   // 编码器位置
#define ARM_GRIP_SPD_DIR  -1   // 编码器速度

//动态限位
#define ARM_P2_MAX_WHEN_P1_MIN 45.f  ///< P1处于最小角度时，P2的最大可达角度

/*-------------------------------------初始化数据--------------------------------------------------------*/
#define ARM_YAW_INIT_ANGLE 0.0f
#define ARM_PITCH1_INIT_ANGLE 6.f
#define ARM_PITCH2_INIT_ANGLE 20.f
#define ARM_ROLL_INIT_ANGLE 0.0f
#define ARM_END_PITCH_INIT_ANGLE 0.0f
#define ARM_END_ROLL_INIT_ANGLE 0.0f
#define ARM_END_ROLL_STALL_ANGLE 90.0f
#define ARM_GRIP_INIT_LENGTH 0.0f


#define ARM_INIT_SAFE_YAW_ANGLE 0.0f
#define ARM_INIT_SAFE_PITCH1_ANGLE 70.0f
#define ARM_INIT_SAFE_PITCH2_ANGLE 70.0f

#define POSIT_JOINT1_YAW_MACH 23899
#define POSIT_JOINT1_YAW_MACH_PHY 0.f
#define ARM_YAW_MOTOR_RANGE_LHK 61551

#define POSIT_JOINT2_PITCH1_MACH 55063
#define POSIT_JOINT2_PITCH1_MACH_PHY 0.f
#define POSIT_JOINT2_PITCH1_INIT_PHY 6.0f

#define POSIT_JOINT3_PITCH2_MACH 11764	 //data = 12678 p1为0度时p2最大角度编码器值和0度编码器值的差值。但是如果是跳变了65535则要将跳变之前的减掉
#define POSIT_JOINT3_PITCH2_MACH_PHY 0.f
#define POSIT_JOINT3_PITCH2_INIT_PHY 20.0f

#define POSIT_JOINT4_ROLL_OFFSET 0

#define POSIT_JOINT5_PITCH_END_MACH 0
#define POSIT_JONIT6_ROLL_END_MACH 0
#define POSIT_END_INIT 0
#define POSIT_END_PHY 205.0f

#define deg2rad(x) ((x) * 0.017453292519943295769236907684886)
#define rad2deg(x) ((x) * 57.295779513082320876798154814105)
/*------------------------------------- 夹爪的相关参数------------------------------------------*/
#define GRIP_OUTPUT_LIMIT 1000
#define GRIP_SLOW_MACH 65535 //开始减速的编码器值
#define GRIP_OPEN_SPEED   15000.0f   ///< 夹爪张开速度
#define GRIP_CLOSE_SPEED  15000.0f   ///< 夹爪闭合速度，电机的上限速度
#define GRIP_INIT_SPEED   6000.0f   ///< 标定速度，恒定低速直到堵转
#define GRIP_MIN_MOVE_SPEED 6000.0f ///< 力矩减速后最小运动速度，低于此值直接归零避免蠕动
#define GRIP_INIT_STALL_CURRENT 50.0f  ///< 标定堵转电流阈值
#define GRIP_INIT_STALL_COUNT   150.f       ///< 标定堵转确认计数
#define GRIP_CLOSE_START_GRACE  180.f        ///< 闭合启动计数器
#define GRIP_GRIP_HOLD_COUNT    20.f        ///< 夹取确认计数
#define GRIP_SPEED_RAMP_STEP    500.0f      ///< 速度斜坡步长
#define GRIP_STALL_SPEED_THRESH 3000.0f     ///< 堵转判定绝对速度阈值

#define GRIP_OPEN_Stop_distance  0.5f
#define GRIP_CLOSE_Stop_distance  0.5f

#define gripOpenStopPosit  (ARM_END_GRIP_MOTOR_RANGE - PhyPositToMtrPosit(GRIP_OPEN_Stop_distance))
#define gripCloseStopPosit (PhyPositToMtrPosit(GRIP_CLOSE_Stop_distance))

/*------------------------------------- 超时判断检测数据------------------------------------------*/
#define TIME_OUT_TICK 8000//ms

/*-------------------------------------重力补偿数据--------------------------------------------------------*/
#define PITCH1     0
#define PITCH2 	   1
#define END_PITCH  2
#define RECORD_MAX 10

#define MG6012_i36V3_Torque_Constant	0.175
#define MG8010_i36V2_Torque_Constant	0.15
#define DMJ4310_Torque_Constant			0.975	///< 对应电机的扭矩常数

#include "mod_common.hpp"
#include "algo_grav_comp.hpp"
#include "algo_other.hpp"

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
		EDeviceID MotorID_Roll = EDeviceID::DEV_NULL;
		EDeviceID MotorID_End_Pitch = EDeviceID::DEV_NULL; ///< 末端Pitch电机(DM4310 MIT)
		EDeviceID MotorID_End_Roll = EDeviceID::DEV_NULL;  ///< 末端Roll电机(DM4310 MIT)
		EDeviceID MotorID_Grip = EDeviceID::DEV_NULL;
		CInfCAN::CCanTxNode *MotorTxNode_Yaw; ///< 机械臂关节Yaw电机发送节点
		CInfCAN::CCanTxNode *MotorTxNode_Pitch1; ///< 机械臂关节Pitch电机1发送节点
		CInfCAN::CCanTxNode *MotorTxNode_Pitch2; ///< 机械臂关节Pitch电机2发送节点
		CInfCAN::CCanTxNode *MotorTxNode_Grip; ///< 机械臂夹爪电机发送节点
		float_t MIT_Roll_kp = 0.0f; ///< MIT控制器Roll比例系数
		float_t MIT_Roll_kd = 0.0f; ///< MIT控制器Roll微分系数
		float_t MIT_End_Pitch_kp = 0.0f; ///< MIT控制器End Pitch比例系数
		float_t MIT_End_Pitch_kd = 0.0f; ///< MIT控制器End Pitch微分系数
		float_t MIT_End_Roll_kp = 0.0f;  ///< MIT控制器End Roll比例系数
		float_t MIT_End_Roll_kd = 0.0f;  ///< MIT控制器End Roll微分系数
		CAlgoPid::SAlgoInitParam_Pid YawPosPidParam;
		CAlgoPid::SAlgoInitParam_Pid YawSpdPidParam;
		CAlgoPid::SAlgoInitParam_Pid Pitch1PosPidParam;
		CAlgoPid::SAlgoInitParam_Pid Pitch1SpdPidParam;
		CAlgoPid::SAlgoInitParam_Pid Pitch2PosPidParam;
		CAlgoPid::SAlgoInitParam_Pid Pitch2SpdPidParam;
		CAlgoPid::SAlgoInitParam_Pid GripPosPidParam;///< 夹爪位置PID参数
		CAlgoPid::SAlgoInitParam_Pid GripSpdPidParam;

		struct SGripInitParam {
			float_t initSpeedMax_     = 6000.0f;   ///< 初始化最大速度
			float_t initSpeedMin_     = 2300.0f;   ///< 保底最低速度
			float_t initTorqueThresh_ = 1200.0f;   ///< 力矩开始减速的阈值
			float_t initTorqueRange_  = 2000.0f;   ///< 从全速减到最低速的力矩区间
		} GripInitParam;

		struct SGripDetectParam {
			float_t closeTorqueThresh = 80.0f;  ///< 滤波电流接触阈值
			float_t closeTorqueRange  = 30.0f;  ///< 接触到夹紧的电流区间
			float_t detectTorque      = 100.0f; ///< 边沿检测夹取成功阈值
			float_t filterAlpha       = 0.95f;   ///< LowPassFilter滤波系数α
		} GripDetectParam;

		SGravParam gravParam;
	};

	// 定义机械臂信息结构体并实例化
	struct SArmInfo {
		EVarStatus isModuleAvailable = false; ///< 模块是否可用
		float_t angle_Yaw = 0.0f; ///< 机械臂关节Yaw角度
		float_t angle_Pitch1 = 0.0f; ///< 机械臂关节Pitch1角度
		float_t angle_Pitch2 = 0.0f; ///< 机械臂关节Pitch2角度
		float_t angle_Roll = 0.0f; ///< 机械臂关节Roll角度
		float_t angle_end_pitch = 0.0f; ///< 机械臂末端Pitch角度
		float_t angle_end_roll = 0.0f; ///< 机械臂末端Roll角度
		float_t length_grip = 0.0f; ///< 机械臂夹爪张开距离
		bool isInitTimeout = false;          ///< 初始化是否超时，在init状态中检测之后，若是超时则切换为自定义控制器模式
		bool isAngleArrived_Yaw = false; ///< 机械臂关节Yaw角度是否到达
		bool isAngleArrived_Pitch1 = false; ///< 机械臂关节Pitch1角度是否到达
		bool isAngleArrived_Pitch2 = false; ///< 机械臂关节Pitch2角度是否到达
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
		float_t set_angle_Roll = 0.0f; ///< 机械臂关节Roll角度设定
		float_t set_angle_end_pitch = 0.0f; ///< 机械臂末端Pitch角度设定
		float_t set_angle_end_roll = 0.0f; ///< 机械臂末端Roll角度设定
		float_t set_length_grip = 0.0f; ///< 机械臂夹爪距离设定（自动任务直接设定）
		float_t set_speed_grip = 0.0f; ///< 夹爪速度设定
		bool gripClose = false;           ///< 手动闭合标志（Core层设置）
		bool gripOpen = false;            ///< 手动张开标志（Core层设置）
		bool resetbyControl = false; ///<通过自定义控制器进行重试
		bool enableGravOnly = false; ///< 纯重力补偿模式: true=PID仅输出重力前馈, false=正常模式
	} armCmd;

	CModArm() = default;

	CAlgoTrajPlayback initTraj_;                     ///< 初始化轨迹规划器实例
	float_t initTrajTime_ = 0.0f;                    ///< 当前轨迹时间 (s)
	bool isInitTrajActive_ = false;                  ///< 轨迹是否正在执行
	uint32_t initStartTick_ = 0;                     ///< FSM_INIT开始时刻，供超时检测使用

	// 定义带参数的模块构造函数，创建模块时自动调用初始化函数
	explicit CModArm(SModInitParam_Arm &param) { InitModule(param); }        ///<要求**带参数**的构造函数要用explicit修饰，防止隐式转换

	// 模块析构函数
	~CModArm() final { UnregisterModule_(); }

	// 初始化模块
	EAppStatus InitModule(SModInitParam_Base &param) final;
	void SetGravityCompMode(CAlgoGravityComp::CGravityCompMode mode);  ///< 含 GRAVITY_ONLY 时自动开关位置环
	CAlgoGravityComp::CGravityCompMode GetGravityCompMode() const { return gravComp_.GetMode(); }
	uint8_t should_limit_yaw = 0; ///< 是否限制Yaw角度

private:
	// 定义机械臂Yaw关节组件类并实例化
	class CComJoint: public CComponentBase {
	public:
		enum {Y = 0, P1 = 1, P2 = 2};
		const int32_t rangeLimit_yaw 		= ARM_YAW_MOTOR_RANGE_LHK; ///< Yaw关节电机范围限制
		const int32_t rangeLimit_pitch1 = ARM_PITCH1_MOTOR_RANGE; ///< Pitch1关节电机范围限制
		const int32_t rangeLimit_pitch2 = ARM_PITCH2_MOTOR_RANGE; ///< Pitch2关节电机范围限制

		// 定义Yaw关节信息结构体
		struct SYawInfo {
			int32_t posit_yaw = 0.0f;           ///< Yaw关节当前位置
			int32_t posit_pitch1 = 0.0f;     ///< Pitch1关节当前位置
			int32_t posit_pitch2 = 0.0f;     ///< Pitch2关节当前位置
			bool isPositArrived_yaw = false;    ///< Yaw位置是否到达目标
			bool isPositArrived_pitch1 = false; ///< Pitch1位置是否到达
			bool isPositArrived_pitch2 = false; ///< Pitch2位置是否到达
			bool isPositArrived_fail= false; 	///<关节有没有初始化到的
		} jointInfo;

		// 定义Yaw关节控制命令结构体
		struct SYawCmd {
			int32_t setPosit_yaw = 0.0f;        ///< Yaw关节目标位置
			int32_t setPosit_pitch1 = POSIT_JOINT2_PITCH1_INIT_PHY; ///< Pitch1关节目标位置
			int32_t setPosit_pitch2 = POSIT_JOINT3_PITCH2_INIT_PHY; ///< Pitch2关节目标位置
		} jointCmd;

		// PID控制器
		CAlgoPid pidPosCtrl_yaw;
		CAlgoPid pidSpdCtrl_yaw;
		CAlgoPid pidPosCtrl_pitch1;
		CAlgoPid pidSpdCtrl_pitch1;
		CAlgoPid pidPosCtrl_pitch2;
		CAlgoPid pidSpdCtrl_pitch2;

		// 电机数据输出缓冲区
		std::array<int16_t, 3> mtrOutputBuffer = {0};

		CAlgoTrajPlayback initTraj_;
		float_t initTrajTime_ = 0.0f;
		bool isInitTrajActive_ = false;

		float_t grav_ff_pitch1 = 0.0f;
		float_t grav_ff_pitch2 = 0.0f;

		bool onlyGravity_ = false;  ///< 纯重力模式
		void SetOnlyGravity(bool enable) { onlyGravity_ = enable; }

		// 到位阈值
		int32_t arrivalThresh_pitch1 = 700;
		int32_t arrivalThresh_pitch2 = 700;

		// 电机实例指针
		CDevMtr* motor[3] = {nullptr};

		// 父类指针，用于访问其他组件
		CModArm* parentModule = nullptr;

		// 物理位置转换为电机位置
		static int32_t PhyPositToMtrPosit_yaw(float_t phyPosit);
		static int32_t PhyPositToMtrPosit_pitch1(float_t phyPosit);
		static int32_t PhyPositToMtrPosit_pitch2(float_t phyPosit);

		// 电机位置转换为物理位置
		static float_t MtrPositToPhyPosit_yaw(int32_t mtrPosit);
		static float_t MtrPositToPhyPosit_pitch1(int32_t mtrPosit);
		static float_t MtrPositToPhyPosit_pitch2(int32_t mtrPosit);

		// 初始化组件
		EAppStatus InitComponent(SModInitParam_Base &param) final;

		// 更新组件
		EAppStatus UpdateComponent() final;

		// 输出更新函数
		EAppStatus _UpdateOutput(float_t posit_yaw, float_t posit_pitch1, float_t posit_pitch2);

		// 单独电机输出更新函数
		EAppStatus _UpdateOutput_Yaw(float_t posit_yaw);
		EAppStatus _UpdateOutput_Pitch1(float_t posit_pitch1);
		EAppStatus _UpdateOutput_Pitch2(float_t posit_pitch2);

		// 电机can发送节点
		std::array<CInfCAN::CCanTxNode*, 3> mtrCanTxNode;

	} comjoint_;

	// 定义机械臂Roll关节组件类并实例化
	class CComRoll: public CComponentBase {
	public:
		// 定义Roll关节信息结构体
		struct SRollInfo {
			float_t angle = 0.0f;           ///< Roll关节当前角度
			float_t torque = 0.0f;
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
		// 计算出来的重补前馈
		float_t grav_ff_roll = 0.0f;

		bool onlyGravity_ = false;  ///< 纯重力补偿
		void SetOnlyGravity(bool enable) { onlyGravity_ = enable; }

		// 到位阈值
		float_t arrivalThresh_roll = 3.0f;
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

	// 定义末端Pitch关节组件类并实例化（DM4310 MIT直驱）
	class CComEndPitch: public CComponentBase {
	public:
		struct SEndPitchInfo {
			float_t angle = 0.0f;           ///< 当前Pitch角度(deg)
			float_t torque = 0.0f;          ///< 当前Pitch力矩(N·m)
			bool isAngleArrived = false;    ///< 角度是否到达目标
		} endPitchInfo;

		struct SEndPitchCmd {
			float_t setAngle = 0.0f;        ///< 目标Pitch角度(deg)
		} endPitchCmd;

		struct SMitCtrl {
			float_t kp = 0.0f;
			float_t kd = 0.0f;
		} mitCtrl;

		CDevMtr* motor = nullptr;

		// 重补输出
		float_t Grav_End_Pitch_Out = 0;

		bool onlyGravity_ = false;  ///< 纯重力补偿
		void SetOnlyGravity(bool enable) { onlyGravity_ = enable; }

		// 初始化组件
		EAppStatus InitComponent(SModInitParam_Base &param) final;

		// 更新组件
		EAppStatus UpdateComponent() final;
	} comEndPitch_;

	// 定义末端Roll关节组件类并实例化（DM4310 MIT直驱）
	class CComEndRoll: public CComponentBase {
	public:
		struct SEndRollInfo {
			float_t angle = 0.0f;           ///< 当前Roll角度(deg)
			float_t torque = 0.0f;          ///< 当前Roll力矩(N·m)
			bool isAngleArrived = false;    ///< 角度是否到达目标
		} endRollInfo;

		struct SEndRollCmd {
			float_t setAngle = 0.0f;        ///< 目标Roll角度(deg)
		} endRollCmd;

		struct SMitCtrl {
			float_t kp = 0.0f;
			float_t kd = 0.0f;
		} mitCtrl;

		CDevMtr* motor = nullptr;

		// 重补输出
		float_t Grav_End_Roll_Out = 0;

		bool onlyGravity_ = false;  ///< 纯重力补偿
		void SetOnlyGravity(bool enable) { onlyGravity_ = enable; }

		// 初始化组件
		EAppStatus InitComponent(SModInitParam_Base &param) final;

		// 更新组件
		EAppStatus UpdateComponent() final;
	} comEndRoll_;

	class CComGrip: public CComponentBase {
	public:

		const int32_t rangeLimit_Grip = ARM_END_GRIP_MOTOR_RANGE; ///< 夹爪电机位置范围限制

		// 夹爪张开方向
		enum class EGripMoveDir : int8_t {
			IDLE  = 0,
			CLOSE = -1,
			OPEN  = 1,
		};

		//速度环输出
		struct SGripCtrlOutput {
			enum class EGripCtrlMode : uint8_t {STOP = 0, SPEED = 1};
			EGripCtrlMode mode = EGripCtrlMode::STOP;
			float_t targetSpeed = 0.0f;  ///< 目标速度（正：张开方向，负：闭合方向）
		};

		// 定义夹爪信息结构体
		struct SGripInfo {
			enum class EGripState : uint8_t { RELEASE = 0, HOLD = 1 };//松开、夹持
			EGripState state = EGripState::RELEASE;
			int32_t posit_grip = 0;
			int32_t holdPosit_Grip = 0;
			bool isGripped = false;
		} gripInfo;

		// 定义夹爪控制命令结构体
		struct SGripCmd {
			int32_t setPosit_grip = 0;        ///< 夹爪目标位置（编码器）
			float_t setSpeed_grip = 0.0f;     ///< 速度设定（拨轮/自动任务用）
			int32_t outTime_tick = 0;         ///< 二次夹紧脉冲起始 tick
			int32_t regripStableCnt = 0;      ///< 二次夹紧稳定计数
			bool regripPulse = false;         ///< 二次夹紧脉冲进行中
			bool cmdReGrip = false;           ///< 二次夹紧请求
			bool cmdClose = false;            ///< 手动闭合标志
			bool cmdOpen = false;             ///< 手动张开标志
		} gripCmd;

		// 定义夹爪力矩检测结构体
		struct SGripCmdDetect {
			bool edgeReady          = false;    ///< 边沿检测就绪标志
			float_t filteredTorque  = 0.0f;     ///< 滤波后的电流值
			float_t closeTorqueThresh = 100.0f;  ///< 滤波电流接触阈值
			float_t closeTorqueRange  = 50.0f;  ///< 接触到夹紧的电流区间
			float_t detectTorque      = 80.0f; ///< 边沿检测夹取成功阈值
			float_t filterAlpha       = 0.95f;  ///< 低通滤波滤波系数
			int32_t gripHoldCnt = 0;      ///< 高电流持续计数
		} gripDetect_;

		CAlgoRamp rampSpd_; ///< 通用速度斜坡，所有速度指令统一经过
		// PID控制器
		CAlgoPid pidPosCtrl;
		CAlgoPid pidSpdCtrl;

		// 电机实例指针
		CDevMtr* motor = nullptr;

		// 电机数据输出缓冲区
		int16_t mtrOutputBuffer = 0;
		SGripCtrlOutput gripCtrlOutput_;
		SGripCtrlOutput::EGripCtrlMode lastCtrlMode_ = SGripCtrlOutput::EGripCtrlMode::STOP;

		// 标定堵转计数器（组件层堵转判断）
		int32_t initStallCnt_ = 0;
		int32_t closeStartCnt_ = 0;
		bool torqueLimitActive_ = false; ///< 力矩减速触发

		static float_t MtrPositToPhyPosit(float_t mtrPosit);
		static int32_t PhyPositToMtrPosit(float_t phyPosit);

		// 判断夹爪的运动方向
		EGripMoveDir DeriveMoveDir() const;


		// 初始化组件
		EAppStatus InitComponent(SModInitParam_Base &param) final;
		// 更新组件
		EAppStatus UpdateComponent() final;

		// 输出更新函数（位置环）
		EAppStatus _UpdateOutput(float_t gripTarget);    ///< 位置环输出
		EAppStatus _UpdateOutputSpd(float_t speedTarget); ///< 速度环输出更新函数
		float_t ApplyTorqueSpeedLimit(float_t spd); ///< 闭合方向力矩线性减速
		// 输出模式的选择
		EAppStatus ApplyGripOutput(const SGripCtrlOutput& output);

		void HandleStateRelease(EGripMoveDir moveDir, SGripCtrlOutput& out);///<两种控制状态
		void HandleStateHold(EGripMoveDir moveDir, SGripCtrlOutput& out);

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

	//超时判断检测函数
	void initTimeoutdect();

	CAlgoGravityComp gravComp_;
	SGravState gravState_; ///< 重力补偿输入状态
	SGravOutput gravOut_;  ///< 重力补偿输出结果

};


} // namespace my_engineer

#endif // MOD_ARM_HPP
