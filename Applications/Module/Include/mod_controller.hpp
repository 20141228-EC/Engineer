/******************************************************************************
 * @brief        
 * 
 * @file         mod_controller.hpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-03-30
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#ifndef MOD_CONTROLLER_HPP
#define MOD_CONTROLLER_HPP

#include "mod_common.hpp"

// 直接把机器人的物理位置给自定义控制器，懒得转换了
#define CONTROLLER_YAW_PHYSICAL_RANGE 195.0f
#define CONTROLLER_YAW_MOTOR_RANGE 4437.33f
#define CONTROLLER_YAW_PHYSICAL_RANGE_MIN -97.5f
#define CONTROLLER_YAW_PHYSICAL_RANGE_MAX 97.5f
#define CONTROLLER_YAW_MOTOR_MACH 4800
/*------------------------------------------------------------------------------------------*/
#define CONTROLLER_PITCH1_PHYSICAL_RANGE 328.6f
#define CONTROLLER_PITCH1_MOTOR_RANGE 392000
#define CONTROLLER_PITCH1_MOTOR_OFFSET 0
/*------------------------------------------------------------------------------------------*/
#define CONTROLLER_PITCH2_PHYSICAL_RANGE_MIN 0.0f     // TODO: 标定后修改
#define CONTROLLER_PITCH2_PHYSICAL_RANGE_MAX 180.0f   // TODO: 标定后修改
/*------------------------------------------------------------------------------------------*/
#define CONTROLLER_ROLL_PHYSICAL_RANGE_MIN -90.0f     // TODO: 标定后修改
#define CONTROLLER_ROLL_PHYSICAL_RANGE_MAX 90.0f      // TODO: 标定后修改
/*----------------------------------Pitch_End-----------------------------------------------*/
#define CONTROLLER_PITCH_END_PHYSICAL_RANGE_MAX 145.0f
#define CONTROLLER_PITCH_END_PHYSICAL_RANGE_MIN -60.0f
#define CONTROLLER_PITCH_END_MOTOR_RANGE 4201
#define CONTROLLER_PITCH_END_MOTOR_RATIO (CONTROLLER_PITCH_END_MOTOR_RANGE / (CONTROLLER_PITCH_END_PHYSICAL_RANGE_MAX - CONTROLLER_PITCH_END_PHYSICAL_RANGE_MIN))
#define CONTROLLER_PITCH_END_MOTOR_OFFSET 3345
/*------------------------------------------------------------------------------------------*/
#define CONTROLLER_PITCH1_MOTOR_RATIO (CONTROLLER_PITCH1_MOTOR_RANGE / CONTROLLER_PITCH1_PHYSICAL_RANGE)
#define CONTROLLER_YAW_MOTOR_RATIO (CONTROLLER_YAW_MOTOR_RANGE / (CONTROLLER_YAW_PHYSICAL_RANGE_MAX - CONTROLLER_YAW_PHYSICAL_RANGE_MIN))
#define CONTROLLER_YAW_MOTOR_OFFSET -CONTROLLER_YAW_MOTOR_RATIO * CONTROLLER_YAW_PHYSICAL_RANGE_MIN


// 当物理位置从0增大时，电机位置的变化方向
#define CONTROLLER_YAW_MOTOR_DIR 1
#define CONTROLLER_PITCH1_MOTOR_DIR -1
#define CONTROLLER_PITCH2_MOTOR_DIR 1
#define CONTROLLER_ROLL_MOTOR_DIR -1
#define CONTROLLER_PITCH_END_MOTOR_DIR -1

// 与自定义控制器模块相关的宏定义
#define CONTROLLER_ROCKER_DEAD_ZONE 10000 // 摇杆死区
#define CONTROLLER_ROCKER_RANGE 65500 // 摇杆范围
#define CONTROLLER_ROCKER_LOW 5000
#define CONTROLLER_ROCKER_HIGH 65530
#define CONTROLLER_ROCKER_KEY_LONG_PRESS_DURATION 2000
#define CONTROLLER_ROLL_SPEED_MAX 200.0f // 大Roll轴电机最大有效速度


// 前伸横移辅助移动
#define CONTROLLER_ASSIST_ENABLE 1 // 是否启用辅助移动

namespace my_engineer {

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
		// int8_t rocker_X = 0; ///< 摇杆X轴值 -100 - 100
		// int8_t rocker_Y = 0; ///< 摇杆Y轴值 -100 - 100
		// KEY_STATUS rocker_Key = KEY_STATUS::RELEASE; ///< 摇杆按键状态
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

private:

	// 定义yaw轴电机组件类并实例化
	class CComYaw: public CComponentBase{
	public:

		const int32_t rangeLimit = CONTROLLER_YAW_MOTOR_RANGE; ///< 电机位置范围限制

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

		const int32_t rangeLimit = CONTROLLER_PITCH1_MOTOR_RANGE; ///< 电机位置范围限制

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
			bool isFree = false;	 ///< Pitch1 Free
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
	class CComRoll: public CComponentBase{
	public:

		// 定义Roll轴信息结构体并实例化
		struct SRollInfo {
			float_t posit = 0;    ///< Roll Position (MIT模式使用角度)
			bool isPositArrived = false;
		} rollInfo;

		// 定义Roll轴控制命令结构体并实例化（MIT模式参数）
		struct SRollCmd {
			bool isFree = false;	 ///< Roll Free
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

};

} // namespace my_engineer

#endif // MOD_CONTROLLER_HPP
		