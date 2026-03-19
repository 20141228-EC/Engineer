/******************************************************************************
 * @brief
 *
 * @file         mod_controller.hpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-03-30
 * @LastEditors  Ciallo(1002046597@qq.com)
 * @LastEditTime 2026-01-22
 *
 * @copyright    Copyright (c) 2025
 *
 ******************************************************************************/

#ifndef MOD_CONTROLLER_HPP
#define MOD_CONTROLLER_HPP

#include "mod_common.hpp"
#include "algo_gravity_comp.hpp"

// 直接把机器人的物理位置给自定义控制器，懒得转换了
#define CONTROLLER_YAW_PHYSICAL_RANGE 195.0f
#define CONTROLLER_YAW_MOTOR_RANGE 4437.33f
#define CONTROLLER_YAW_PHYSICAL_RANGE_MIN -97.5f
#define CONTROLLER_YAW_PHYSICAL_RANGE_MAX 97.5f
#define CONTROLLER_YAW_MOTOR_MACH 4800
/*------------------------------------------------------------------------------------------*/
#define CONTROLLER_PITCH1_PHYSICAL_RANGE_MIN 0.0f
#define CONTROLLER_PITCH1_PHYSICAL_RANGE_MAX 90.0f
#define CONTROLLER_PITCH1_PHYSICAL_RANGE 328.6f
#define CONTROLLER_PITCH1_MOTOR_RANGE 392000
#define CONTROLLER_PITCH1_MOTOR_OFFSET 0
/*----------------------------------零点标定偏移(deg)-------------------------------------------*/
// 上电时关节未精确停在物理0°导致的偏差，正值表示电机0rad对应的物理角度
// 例如：偏移8.0表示电机报告0rad时，控制器关节实际在物理8°位置
#define CONTROLLER_PITCH1_ZERO_OFFSET  0.0f   // P1零点偏移(deg)，根据实测调节
#define CONTROLLER_PITCH2_ZERO_OFFSET  8.0f   // P2零点偏移(deg)，根据实测调节
/*----------------------------------Pitch2限幅范围-----------------------------------------------*/
#define CONTROLLER_PITCH2_PHYSICAL_RANGE_MIN 0.0f
#define CONTROLLER_PITCH2_PHYSICAL_RANGE_MAX 180.0f
/*----------------------------------roll限幅范围------------------------------------------*/
#define CONTROLLER_ROLL_PHYSICAL_RANGE_MIN -163.0f     ///< 对应机器人 Roll 上限 163°（映射取反）
#define CONTROLLER_ROLL_PHYSICAL_RANGE_MAX 175.0f      ///< 对应机器人 Roll 下限 -175°（映射取反）
/*----------------------------------Pitch_End-----------------------------------------------*/
#define CONTROLLER_PITCH_END_PHYSICAL_RANGE_MAX 145.0f
#define CONTROLLER_PITCH_END_PHYSICAL_RANGE_MIN -60.0f
#define CONTROLLER_PITCH_END_MOTOR_RANGE 4201
#define CONTROLLER_PITCH_END_MOTOR_RATIO (CONTROLLER_PITCH_END_MOTOR_RANGE / (CONTROLLER_PITCH_END_PHYSICAL_RANGE_MAX - CONTROLLER_PITCH_END_PHYSICAL_RANGE_MIN))
#define CONTROLLER_PITCH_END_MOTOR_OFFSET 3345
/*----------------------------------重力补偿安装偏移(deg)------------------------------------*/
// DH角度 = (物理角度 - OFFSET) * DEG2RAD
// K3_end符号修正后：零力矩跳变点在P2≈50°，需移至90°，P2偏移+40°
#define CONTROLLER_GRAV_COMP_PITCH1_OFFSET    188.0f    // P1: 最小值在90°
#define CONTROLLER_GRAV_COMP_PITCH2_OFFSET    170.0f    // P2: 最小值在90°
#define CONTROLLER_GRAV_COMP_ROLL_OFFSET      10.0f     // Roll: DH零点偏移（无关点-80°和+100°的中点）
#define CONTROLLER_GRAV_COMP_PITCHEND_OFFSET  20.0f     // PitchEnd: 42+20，补偿PITCH2_OFFSET变化的影响
/*----------------------------------重力补偿力矩限幅(N·m)------------------------------------*/
#define CONTROLLER_GRAV_COMP_TAU_LIMIT_DM4310  2.5f    // Pitch1/2 (DM4310) 额定3N·m，留余量
#define CONTROLLER_GRAV_COMP_TAU_LIMIT_DM3510  0.5f    // Roll/PitchEnd (DM3510) 峰值力矩测试
/*----------------------------------电机减速比------------------------------------------------*/
#define CONTROLLER_GEAR_RATIO_DM4310  10.0f   // Pitch1/2 (DM4310) 减速比 10:1
/*----------------------------------各轴效率/补偿缩放(欠补偿时增大，过补偿时减小)---------------*/
#define CONTROLLER_PITCH1_EFFICIENCY_COMP   1.5f    // Pitch1 (DM4310) 减速器效率补偿
#define CONTROLLER_PITCH2_EFFICIENCY_COMP   1.5f    // Pitch2 (DM4310) 减速器效率补偿
#define CONTROLLER_PITCHEND_EFFICIENCY_COMP 1.0f     // PitchEnd (DM3510) K5_1已校准，无需额外缩放
/*------------------------------------------------------------------------------------------*/
#define CONTROLLER_PITCH1_MOTOR_RATIO (CONTROLLER_PITCH1_MOTOR_RANGE / CONTROLLER_PITCH1_PHYSICAL_RANGE)
#define CONTROLLER_YAW_MOTOR_RATIO (CONTROLLER_YAW_MOTOR_RANGE / (CONTROLLER_YAW_PHYSICAL_RANGE_MAX - CONTROLLER_YAW_PHYSICAL_RANGE_MIN))
#define CONTROLLER_YAW_MOTOR_OFFSET -CONTROLLER_YAW_MOTOR_RATIO * CONTROLLER_YAW_PHYSICAL_RANGE_MIN


// 当物理位置从0增大时，电机位置的变化方向
#define CONTROLLER_YAW_MOTOR_DIR 1
#define CONTROLLER_PITCH1_MOTOR_DIR -1
#define CONTROLLER_PITCH2_MOTOR_DIR -1
#define CONTROLLER_ROLL_MOTOR_DIR -1
#define CONTROLLER_PITCH_END_MOTOR_DIR -1     // PitchEnd: MotortruePositToOffsetPosit含取反，与P2同理

// 摇杆校准参数
#define CONTROLLER_ROCKER_DEAD_ZONE 2000   // 摇杆死区

#define CONTROLLER_ROCKER_KEY_LONG_PRESS_DURATION 2000
#define CONTROLLER_ROLL_SPEED_MAX 200.0f // 大Roll轴电机最大有效速度


// 前伸横移辅助移动
#define CONTROLLER_ASSIST_ENABLE 1 // 是否启用辅助移动

namespace my_engineer {

// 前向声明
class CModController;

// 左右臂控制器实例（定义在conf_module.cpp，调试器可直接查看）
extern CModController controllerModuleLeft;
extern CModController controllerModuleRight;

/**
 * @brief 控制器模块类
 *
 */
class CModController final: public CModBase{
public:

	// 定义控制器模块初始化参数结构体
	struct SModInitParam_Controller: public SModInitParam_Base{
		EDeviceID rocker_id 		= EDeviceID::DEV_NULL; ///< 摇杆设备ID
		EDeviceID button_id 		= EDeviceID::DEV_NULL; ///< 按键设备ID
		EDeviceID buzzer_id 		= EDeviceID::DEV_NULL; ///< 蜂鸣器设备ID
		/*--------------------------摇杆X轴校准参数--------------------------------------*/
		int32_t rocker_x_center    = 34900;  ///< X轴中心值 (ADC原始值)
		int32_t rocker_x_range_pos = 26300;  ///< X轴正向范围 (raw > center 方向)
		int32_t rocker_x_range_neg = 26300;  ///< X轴负向范围 (raw < center 方向)
		int8_t  rocker_x_dir       = -1;     ///< X轴方向系数 (-1 或 +1)
		/*--------------------------摇杆Y轴校准参数--------------------------------------*/
		int32_t rocker_y_center    = 46880;  ///< Y轴中心值 (ADC原始值)
		int32_t rocker_y_range_pos = 26080;  ///< Y轴正向范围 (raw > center 方向)
		int32_t rocker_y_range_neg = 24960;  ///< Y轴负向范围 (raw < center 方向)
		int8_t  rocker_y_dir       = 1;      ///< Y轴方向系数 (-1 或 +1)
		EDeviceID yaw_id 				= EDeviceID::DEV_NULL; ///< yaw电机设备ID
		EDeviceID pitch1_id 		= EDeviceID::DEV_NULL; ///< 大pitch电机设备ID
		EDeviceID pitch2_id 		= EDeviceID::DEV_NULL; ///<小pitch电机设备ID
		EDeviceID roll_id 			= EDeviceID::DEV_NULL; ///< Roll轴电机设备ID (MIT模式)
		EDeviceID pitch_end_id 	= EDeviceID::DEV_NULL; ///< 末端pitch电机设备ID (MIT模式)
		/*--------------------------Set Can----------------------------------------------*/
		CInfCAN::CCanTxNode *yawTxNode;
		CInfCAN::CCanTxNode *pitch1TxNode;
		CInfCAN::CCanTxNode *pitch2TxNode;
		CInfCAN::CCanTxNode *rollTxNode;
		CInfCAN::CCanTxNode *pitchEndTxNode;
		/*--------------------------Set Pid----------------------------------------------*/
		CAlgoPid::SAlgoInitParam_Pid yawPosPidParam;
		CAlgoPid::SAlgoInitParam_Pid yawSpdPidParam;
	};

	enum KEY_STATUS  {RELEASE = 0, PRESS = 1, LONG_PRESS = 2,};
	/*define the */
	enum  EMotorParam: int{
    POSIT = 0, 			///< Position
    SPEED = 1,     	///< Speed
    KP,        		 	///< Proportional Gain
    KD,        			///< Derivative Gain
    TF,        			///< Feedforward Torque
    COUNT_     			///< Count of Motor Parameters
  };

	// 定义控制器信息结构体并实例化
	struct SControllerInfo{
		EVarStatus isModuleAvailable = false; ///< 模块是否可用
		EVarStatus isReturnSuccess = false; ///< 归位是否成功
		bool isRest = false; ///< 是否归位
		bool isLevel4 = false; ///< 是否处于四级状态
		bool isLevel3 = false; ///< 是否处于三级状态
		bool isSelf = false; ///<
		int8_t rocker_X = 0; ///< 摇杆X轴值 -100 - 100
		int8_t rocker_Y = 0; ///< 摇杆Y轴值 -100 - 100
		KEY_STATUS rocker_Key = KEY_STATUS::RELEASE; ///< 摇杆按键状态
		float_t posit_yaw = 0; ///< yaw电机位置
		float_t posit_pitch1 = 0; ///< pitch1电机位置
		float_t posit_pitch2 = 0; ///< pitch2电机位置
		float_t posit_roll = 0; ///< Roll轴电机位置 (MIT模式)
		float_t posit_pitch_end = 0; ///< 末端pitch电机位置 (MIT模式)
	} ControllerInfo = { };

	// 定义控制器命令结构体并实例化
	struct SControllerCmd{
		EVarStatus StartControl = false; ///< 控制器开始控制信号
		EVarStatus isFree = false; ///< 控制器是否可自由控制
		float_t cmd_yaw = 0; ///< 横移电机命令
		float_t cmd_pitch1 = 0; ///< 大pitch电机命令
		float_t cmd_pitch2 = 0; ///< 小pitch电机命令
		float_t cmd_roll = 0; ///< Roll轴电机命令 (MIT模式)
		float_t cmd_pitch_end = 0; ///< 末端pitch电机命令 (MIT模式)
	} ControllerCmd = { };

	CModController() = default;

	// 定义带参数的模块构造函数，创建模块时自动调用初始化函数
	explicit CModController(SModInitParam_Controller &param) { InitModule(param); }

	// 模块析构函数
	~CModController() final { UnregisterModule_(); }

	// 模块初始化
	EAppStatus InitModule(SModInitParam_Base &param) final;

	int16_t get_rocker_x() {return comRocker_.rockerInfo.X;};
	int16_t get_rocker_y() {return comRocker_.rockerInfo.Y;};

	// 重力补偿控制接口
	void SetGravityCompEnabled(bool enabled) { gravityCompEnabled_ = enabled; }
	bool IsGravityCompEnabled() const { return gravityCompEnabled_; }
	void SetGravityCompScale(float scale) { gravityComp_.SetScale(scale); }
	float GetGravityCompScale() const { return gravityComp_.GetScale(); }

private:

	// 定义yaw轴电机组件类并实例化
	class CComYaw: public CComponentBase{
	public:

		const int32_t rangeLimit = static_cast<int32_t>(CONTROLLER_YAW_MOTOR_RANGE); ///< 电机位置范围限制

		// 定义Yaw轴信息结构体并实例化
		struct SYawInfo {
			int32_t posit = 0;    ///< Yaw Position
			bool isPositArrived = false; ///< Yaw Position Arrived
		} yawInfo;

		// 定义Yaw轴控制命令结构体并实例化
		struct SYawCmd {
			bool isFree = false;	 ///< Yaw Free
			int32_t setPosit = 0;    ///< Yaw Position Set
		} yawCmd;

		// 电机实例指针
		CDevMtr *motor[1] = {nullptr};

		// 定义Yaw轴PID控制器
		CAlgoPid pidPosCtrl;
		CAlgoPid pidSpdCtrl;

		// 电机数据输出缓冲区
		std::array<int16_t, 1> mtrOutputBuffer = {0};

		// 初始化组件
		EAppStatus InitComponent(SModInitParam_Base &param) final;

		// 更新组件
		EAppStatus UpdateComponent() final;

		// 物理位置转换为电机位置
		static int32_t PhyPositToMtrPosit(float_t phyPosit);

		// 电机位置转换为物理位置
		static float_t MtrPositToPhyPosit(int32_t mtrPosit);

		// 输出更新函数
		EAppStatus _UpdateOutput(float_t posit);

		// 电机can发送节点
		std::array<CInfCAN::CCanTxNode*, 1> mtrCanTxNode_;

	} comYaw_;

	// 定义Pitch1轴组件类并实例化
	class CComPitch1: public CComponentBase{
	public:

		const int32_t rangeLimit = static_cast<int32_t>(CONTROLLER_PITCH1_MOTOR_RANGE); ///< 电机位置范围限制

		// 定义Pitch1轴信息结构体并实例化
		struct SPitch1Info {
			float_t posit = 0;    ///< Pitch1 Position
			bool isPositArrived = false; ///< Pitch1 Position Arrived
		} pitch1Info;

		// 定义Pitch1轴控制命令结构体并实例化
		struct SPitch1Cmd {
			bool isFree = false;	 ///< Pitch1 Free
			float_t setParam[static_cast<int>(EMotorParam::COUNT_)] = {0};
		} pitch1Cmd;

		// 电机实例指针
		CDevMtrDM *motor[1] = {nullptr};

		// 初始化组件
		EAppStatus InitComponent(SModInitParam_Base &param) final;

		// 更新组件
		EAppStatus UpdateComponent() final;

		EAppStatus _UpdateOutput(float_t* setParam);

    static float_t OffsetPositToMotortruePosit_test(float_t OffsetPosit);
    static float_t MotortruePositToOffsetPosit_test(float_t MotortruePosit);

		// 电机can发送节点
		std::array<CInfCAN::CCanTxNode*, 1> mtrCanTxNode_;

	} comPitch1_;

	// 定义Pitch2轴组件类并实例化
	class CComPitch2: public CComponentBase{
	public:

		// 定义Pitch2轴信息结构体并实例化
		struct SPitch2Info {
			float_t posit = 0;    ///< Pitch2 Position
			bool isPositArrived = false; ///< Pitch2 Position Arrived
		} pitch2Info;

		// 定义Pitch2轴控制命令结构体并实例化
		struct SPitch2Cmd {
			bool isFree = false;	 ///< Pitch2 Free
			float_t setParam[static_cast<int>(EMotorParam::COUNT_)] = {0};
		} pitch2Cmd;

		// 电机实例指针
		CDevMtrDM *motor[1] = {nullptr};

		// 初始化组件
		EAppStatus InitComponent(SModInitParam_Base &param) final;

		// 更新组件
		EAppStatus UpdateComponent() final;

		EAppStatus _UpdateOutput(float_t* setParam);

    static float_t OffsetPositToMotortruePosit_test(float_t OffsetPosit);
    static float_t MotortruePositToOffsetPosit_test(float_t MotortruePosit);

		// 电机can发送节点
		std::array<CInfCAN::CCanTxNode*, 1> mtrCanTxNode_;

	} comPitch2_;

	/*----------- Roll轴组件类（MIT模式，DM3510） -----------*/
	/**
	 * @note Roll（第4轴）使用电机编码值映射，跟随控制器编码器位置
	 */
	class CComRoll: public CComponentBase{
	public:

		// 定义Roll轴信息结构体并实例化
		struct SRollInfo {
			float_t posit = 0;    ///< Roll Position (MIT模式使用角度)
			bool isPositArrived = false;
		} rollInfo;

		// 定义Roll轴控制命令结构体并实例化（MIT模式参数）
		struct SRollCmd {
			bool isFree = false;	 ///< Roll Free (保留字段，当前未使用)
			float_t setParam[static_cast<int>(EMotorParam::COUNT_)] = {0};
		} rollCmd;

		// 电机实例指针（MIT模式使用DM电机）
		CDevMtrDM *motor[1] = {nullptr};

		// 初始化组件
		EAppStatus InitComponent(SModInitParam_Base &param) final;

		// 更新组件
		EAppStatus UpdateComponent() final;

		// 输出更新函数
		EAppStatus _UpdateOutput(float_t* setParam);

		// 物理位置与电机位置转换（MIT模式角度转换）
		static float_t OffsetPositToMotortruePosit(float_t offsetPosit);
		static float_t MotortruePositToOffsetPosit(float_t motortruePosit);

		// 电机can发送节点
		std::array<CInfCAN::CCanTxNode*, 1> mtrCanTxNode_;

	} comRoll_;

	/*----------- 末端Pitch轴组件类（MIT模式，DM3510） -----------*/
	class CComPitchEnd: public CComponentBase{
	public:

		// 定义末端Pitch轴信息结构体并实例化
		struct SPitchEndInfo {
			float_t posit = 0;    ///< Pitch End Position (MIT模式使用角度)
			bool isPositArrived = false; ///< Pitch End Position Arrived
		} pitchEndInfo;

		// 定义末端Pitch轴控制命令结构体并实例化（MIT模式参数）
		struct SPitchEndCmd {
			bool isFree = false;	 ///< Pitch End Free
			float_t setParam[static_cast<int>(EMotorParam::COUNT_)] = {0};
		} pitchEndCmd;

		// 电机实例指针（MIT模式使用DM电机）
		CDevMtrDM *motor[1] = {nullptr};

		// 初始化组件
		EAppStatus InitComponent(SModInitParam_Base &param) final;

		// 更新组件
		EAppStatus UpdateComponent() final;

		// 输出更新函数
		EAppStatus _UpdateOutput(float_t* setParam);

		// 物理位置与电机位置转换（MIT模式角度转换）
		static float_t OffsetPositToMotortruePosit(float_t offsetPosit);
		static float_t MotortruePositToOffsetPosit(float_t motortruePosit);

		// 电机can发送节点
		std::array<CInfCAN::CCanTxNode*, 1> mtrCanTxNode_;

	} comPitchEnd_;

	// 定义摇杆组件类并实例化
	class CComRocker: public CComponentBase{
	public:

		CDevRocker *rocker = nullptr; ///< 摇杆设备指针

		using KEY_STATUS = CModController::KEY_STATUS;

		// X轴校准参数（每个摇杆实例独立）
		int32_t x_center    = 34900;  ///< X轴中心值
		int32_t x_range_pos = 26300;  ///< X轴正向归一化范围
		int32_t x_range_neg = 26300;  ///< X轴负向归一化范围
		int8_t  x_dir       = -1;     ///< X轴方向系数

		// Y轴校准参数（每个摇杆实例独立）
		int32_t y_center    = 46880;  ///< Y轴中心值
		int32_t y_range_pos = 26080;  ///< Y轴正向归一化范围
		int32_t y_range_neg = 24960;  ///< Y轴负向归一化范围
		int8_t  y_dir       = 1;      ///< Y轴方向系数

		// 定义摇杆信息结构体并实例化
		struct SRockerInfo {
			int32_t X = 0; ///< X轴值
			int32_t Y = 0; ///< Y轴值
			KEY_STATUS Key_status = KEY_STATUS::RELEASE; ///< 按键状态
		} rockerInfo;

		uint32_t last_time_stamp = 0; ///< 上次时间戳
		uint32_t key_duration = 0; ///< 按键持续时间

		// 初始化组件
		EAppStatus InitComponent(SModInitParam_Base &param) final;

		// 更新组件
		EAppStatus UpdateComponent() final;

	} comRocker_;

	// 定义蜂鸣器组件类并实例化
	class CComBuzzer: public CComponentBase{
	public:

		using MusicType = CDevBuzzer::MusicType; ///< 音乐类型

		CDevBuzzer *buzzer = nullptr; ///< 蜂鸣器设备指针

		MusicType current_music = MusicType::NONE; ///< 当前音乐类型

		// 定义蜂鸣器信息结构体并实例化
		struct SBuzzerInfo {
			bool play_ready = false; ///< 可播放
		} buzzerInfo;

		// 定义蜂鸣器控制命令结构体并实例化
		struct SBuzzerCmd {
			MusicType musicType = MusicType::NONE; ///< 音乐类型
		} buzzerCmd;

		// 初始化组件
		EAppStatus InitComponent(SModInitParam_Base &param) final;

		// 更新组件
		EAppStatus UpdateComponent() final;

	} comBuzzer_;


	// 重写基类的方法
	void UpdateHandler_() final;
	void HeartbeatHandler_() final;
	EAppStatus CreateModuleTask_() final;

	// 声明模块任务函数
	static void StartControllerModuleTask(void *argument);

	// 控制量限制函数
	EAppStatus RestrictControllerCommand_();

	// 重力补偿相关
	CAlgoGravityComp gravityComp_;             ///< 重力补偿算法实例
	bool gravityCompEnabled_ = true;           ///< 重力补偿使能标志
	void UpdateGravityComp_();                  ///< 计算并应用重力补偿

};

} // namespace my_engineer

#endif // MOD_CONTROLLER_HPP
		