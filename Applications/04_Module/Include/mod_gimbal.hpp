/**
 * @file mod_gimbal.hpp
 * @author Ciallo
 * @brief 云台模块:升降 (双M2006同步)、俯仰 (舵机/电机可切换)
 * @version 2.1
 * @date 2025-01-19
 *
 * @note 使用 USE_PITCH_SERVO 宏来切换俯仰控制方式
 *       定义该宏则使用舵机，否则使用M2006电机
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef MOD_GIMBAL_HPP
#define MOD_GIMBAL_HPP

#include "mod_common.hpp"

// ----------------- 俯仰控制方式选择 -----------------
// 定义此宏使用舵机控制俯仰，注释掉则使用M2006电机
#define USE_PITCH_SERVO


// 声明舵机设备类
#ifdef USE_PITCH_SERVO
class CDevServo;
#endif

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


namespace my_engineer {

class CModGimbal final: public CModBase{
public:

	// 云台模块初始化参数
	struct SModInitParam_Gimbal: public SModInitParam_Base{
		// 升降组件 (双电机同步)
		EDeviceID liftMotorID_L = EDeviceID::DEV_NULL;
		EDeviceID liftMotorID_R = EDeviceID::DEV_NULL;
		CInfCAN::CCanTxNode *liftMotorTxNode_L = nullptr;
		CInfCAN::CCanTxNode *liftMotorTxNode_R = nullptr;
		CAlgoPid::SAlgoInitParam_Pid liftPosPidParam;
		CAlgoPid::SAlgoInitParam_Pid liftSpdPidParam;

#ifdef USE_PITCH_SERVO
		// 俯仰组件 (舵机版本)
		EDeviceID pitchServoID = EDeviceID::DEV_NULL;
		float_t servoAngleMin = 0.0f;      ///< 舵机最小角度
		float_t servoAngleMax = 180.0f;    ///< 舵机最大角度
		float_t servoAngleOffset = 45.0f;  ///< 舵机角度偏移（物理0度对应的舵机角度）
#else
		// 俯仰组件 (单电机)
		EDeviceID pitchMotorID = EDeviceID::DEV_NULL;
		CInfCAN::CCanTxNode *pitchMotorTxNode = nullptr;
		CAlgoPid::SAlgoInitParam_Pid pitchPosPidParam;
		CAlgoPid::SAlgoInitParam_Pid pitchSpdPidParam;
#endif
	};

	// 云台信息
	struct SGimbalInfo{
		EVarStatus isModuleAvailable = false;
		float_t posit_lift = 0.0f;
		bool isPositArrived_Lift = false;
		float_t posit_pitch = 0.0f;
		bool isPositArrived_Pitch = false;
	} gimbalInfo;

	// 云台控制命令
	struct SGimbalCmd{
		EVarStatus isAutoCtrl = false;
		float_t set_posit_lift = 0.0f;
		float_t set_posit_pitch = 0.0f;
	} gimbalCmd;

	CModGimbal() = default;

	explicit CModGimbal(SModInitParam_Gimbal &param) { InitModule(param); }

	// 模块析构函数
	~CModGimbal() final { UnregisterModule_(); }

	// 初始化模块
	EAppStatus InitModule(SModInitParam_Base &param) final;

private:
	// 升降组件 (双电机同步)
	class CComLift: public CComponentBase{
	public:

		enum { L = 0, R = 1 };

		const int32_t rangeLimit = GIMBAL_LIFT_MOTOR_RANGE;

		struct SLiftInfo{
			int32_t posit = 0;
			bool isPositArrived = false;
		} liftInfo;

		struct SLiftCmd{
			int32_t setPosit = 0;
		} liftCmd;

		std::array<CDevMtr*, 2> motor = {nullptr, nullptr};
		CAlgoPid pidPosCtrl;
		CAlgoPid pidSpdCtrl;
		std::array<int16_t, 2> mtrOutputBuffer = {0, 0};
		std::array<CInfCAN::CCanTxNode*, 2> mtrCanTxNode = {nullptr, nullptr};

		EAppStatus InitComponent(SModInitParam_Base &param) final;
		EAppStatus UpdateComponent() final;
		static int32_t PhyPositToMtrPosit(float_t phyPosit);
		static float_t MtrPositToPhyPosit(int32_t mtrPosit);
		EAppStatus _UpdateOutput(float_t posit);

	} comLift_;

#ifdef USE_PITCH_SERVO
	// 俯仰组件 (舵机版本)
	class CComPitchServo: public CComponentBase{
	public:

		struct SPitchInfo{
			float_t posit = 0;      ///< 当前角度（度）
			bool isPositArrived = false;
		} pitchInfo;

		struct SPitchCmd{
			float_t setPosit = 0;   ///< 目标角度（度）
		} pitchCmd;

		CDevServo *servo = nullptr;

		// 舵机角度参数
		float_t servoAngleMin = 0.0f;
		float_t servoAngleMax = 180.0f;
		float_t servoAngleOffset = 0.0f;

		EAppStatus InitComponent(SModInitParam_Base &param) final;

		EAppStatus UpdateComponent() final;

		static float_t PhyPositToSetPosit(float_t phyPosit);

		static float_t SetPositToPhyPosit(float_t setPosit);

		EAppStatus _UpdateServoOutput(float_t posit);

	} comPitchServo_;

#else
	// 俯仰组件 (单电机)
	class CComPitch: public CComponentBase{
	public:

		const int32_t rangeLimit = GIMBAL_PITCH_MOTOR_RANGE;

		struct SPitchInfo{
			int32_t posit = 0;
			bool isPositArrived = false;
		} pitchInfo;

		struct SPitchCmd{
			int32_t setPosit = 0;
		} pitchCmd;

		CDevMtr *motor = nullptr;

		CAlgoPid pidPosCtrl;
		CAlgoPid pidSpdCtrl;

		std::array<int16_t, 1> mtrOutputBuffer = {0};
		CInfCAN::CCanTxNode* mtrCanTxNode = nullptr;

		EAppStatus InitComponent(SModInitParam_Base &param) final;

		EAppStatus UpdateComponent() final;

		static int32_t PhyPositToMtrPosit(float_t phyPosit);

		static float_t MtrPositToPhyPosit(int32_t mtrPosit);

		EAppStatus _UpdateOutput(float_t posit);

	} comPitch_;
#endif

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
