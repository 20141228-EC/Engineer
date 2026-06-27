/**
 * @file mod_gimbal.hpp
 * @author ciallo
 * @brief 云台模块
 * @version 3.1
 * @date 2026-06-08
 *
 * @copyright Copyright (c) 2026
 *
 * @details 管理图传yaw轴 (DM_MIT电机) 与 Pitch轴 (DJI M2006电机)
 */

#ifndef MOD_GIMBAL_HPP
#define MOD_GIMBAL_HPP

#include "mod_common.hpp"

#define GIMBAL_VISUAL_MOTOR_MOTOR_DIR -1     ///< 图传yaw电机方向 (1=正向, -1=反向)
#define GIMBAL_VISUAL_MOTOR_INIT_ANGLE 1    ///< 图传yaw初始角度
#define GIMBAL_PITCH_INIT_ANGLE 0

// Pitch轴宏定义
#define GIMBAL_PITCH_MOTOR_DIR        -1     ///< Pitch电机方向
#define GIMBAL_PITCH_PHYSICAL_RANGE       110.0f  ///< Pitch物理角度范围 
#define GIMBAL_PITCH_MOTOR_RANGE    66980  ///< 编码器范围 
#define GIMBAL_PITCH_MEC    33490  //偏移
#define GIMBAL_PITCH_MOTOR_RATIO      (GIMBAL_PITCH_MOTOR_RANGE / GIMBAL_PITCH_PHYSICAL_RANGE)
#define GIMBAL_PITCH_GRAV_FF  200       ///< 重力前馈
#define deg2rad(x) ((x) * 0.017453292519943295769236907684886)
#define rad2deg(x) ((x) * 57.295779513082320876798154814105)

namespace my_engineer {

/**
 * @brief 云台模块
 *
 * @details 仅管理图传yaw轴 (DM_MIT电机)
 */
class CModGimbal final: public CModBase{
public:

	/// 云台模块初始化参数
	struct SModInitParam_Gimbal: public SModInitParam_Base{
		// Yaw轴 (DM_MIT)
		EDeviceID yawVisualMotorID = EDeviceID::DEV_NULL;   ///< 图传yaw电机设备ID
		CInfCAN::CCanTxNode *yawVisualMotorTxNode = nullptr;///< CAN发送节点 (DM_MIT自发, 此处未使用)
		float_t MIT_YAW_kp = 0.0f; ///< MIT控制器位置刚度系数 (0-500 N/rad)
		float_t MIT_YAW_kd = 0.0f; ///< MIT控制器阻尼系数 (0-5 N·s/rad)

		// Pitch轴 (DJI M2006)
		EDeviceID pitchMotorID = EDeviceID::DEV_NULL;       ///< Pitch电机设备ID
		CInfCAN::CCanTxNode *pitchMotorTxNode = nullptr;    ///< Pitch CAN发送节点
		CAlgoPid::SAlgoInitParam_Pid PitchPosPidParam;      ///< Pitch位置PID参数
		CAlgoPid::SAlgoInitParam_Pid PitchSpdPidParam;      ///< Pitch速度PID参数
	};

	/// 云台状态信息
	struct SGimbalInfo{
		EVarStatus isModuleAvailable = false;       ///< 模块是否可用
		EVarStatus isIntoControll = false;       ///< 是否进入了自定义控制器控制
		bool isPositArrived_Visualyaw = false;      ///< 图传yaw是否到达目标角度
		float_t angle_visualyaw = 0.f;              ///< 图传yaw当前角度
		bool isPositArrived_Pitch = false;          ///< Pitch是否到达目标角度
		float_t angle_pitch = 0.f;                  ///< Pitch当前角度 (度)
	} gimbalInfo;

	/// 云台控制命令
	struct SGimbalCmd{
		EVarStatus isAutoCtrl = false;              ///< 是否处于自动控制模式
		float_t set_visualyaw = 0.f;                ///< 图传yaw目标角度
		float_t set_pitch = 0.f;                      ///< Pitch目标角度
	} gimbalCmd;

	CModGimbal() = default;

	explicit CModGimbal(SModInitParam_Gimbal &param) { InitModule(param); }

	~CModGimbal() final { UnregisterModule_(); }

	EAppStatus InitModule(SModInitParam_Base &param) final;

private:

	/**
	 * @brief 图传yaw轴组件
	 */
	class CComVisualyaw:public CComponentBase{
	public:

		/// 图传yaw轴状态信息
		struct SVisuallyawInfo
		{
			float_t angle = 0.0f;           ///< 当前角度
			bool isAngleArrived = false;     ///< 是否到达目标
		}VisuallyawInfo;

		/// 图传yaw轴控制命令
		struct SVisuallyawCmd {
			float_t setAngle = 0.0f;        ///< 目标角度
		}VisuallyawCmd;

		/// MIT控制参数
		struct SMitCtrl {
			float_t kp = 0.0f;     ///< 位置刚度系数
			float_t kd = 0.0f;     ///< 阻尼系数
			float_t q = 0.0f;      ///< 目标位置 (弧度)
			float_t dq = 0.0f;     ///< 目标速度 (rad/s)
			float_t tau = 0.0f;    ///< 力矩前馈 (N·m)
		} mitCtrl;

		CDevMtr* motor = nullptr;   ///< 电机实例指针 (实际类型 CDevMtrDM_MIT*)

		static float_t MtrAngleToPhyAngle(float_t angle) {
			return angle;
		}

		static float_t PhyAngleToMtrAngle(float_t angle) {
			return angle;
		}

		EAppStatus InitComponent(SModInitParam_Base &param) final;

		EAppStatus UpdateComponent() final;

	}comVisualyaw_;

	/**
	 * @brief 云台Pitch轴组件
	 */
	class CComGimbalPitch : public CComponentBase {
	public:

		/// Pitch轴状态信息
		struct SPitchInfo {
			int32_t posit = 0;              ///< 当前位置
			bool isPositArrived = false;    ///< 是否到达目标
		} pitchInfo;

		/// Pitch轴控制命令
		struct SPitchCmd {
			int32_t setPosit = 0;           ///< 目标位置 
		} pitchCmd;

		const int32_t rangeLimit = GIMBAL_PITCH_MOTOR_RANGE;
		CDevMtr *motor = nullptr;           ///< 电机实例指针
		CInfCAN::CCanTxNode *mtrCanTxNode = nullptr; ///< CAN发送节点
		CAlgoPid pidPosCtrl;               ///< 位置PID控制器
		CAlgoPid pidSpdCtrl;               ///< 速度PID控制器
		int16_t mtrOutputBuffer = 0;        ///< 电机输出缓冲区

		/// 编码器位置转物理角度
		static float_t MtrPositToPhyPosit(int32_t posit);

		/// 物理角度转编码器位置 
		static int32_t PhyPositToMtrPosit(float_t angle);

		EAppStatus InitComponent(SModInitParam_Base &param) final;
		EAppStatus UpdateComponent() final;

	private:
		EAppStatus _UpdateOutput(float_t targetPosit);

	} comGimbalPitch_;

    void UpdateHandler_() final;        ///< 主循环更新
    void HeartbeatHandler_() final;     ///< 心跳检测
    EAppStatus CreateModuleTask_() final;///< 创建模块

    static void StartGimbalModuleTask(void *argument); ///< 模块任务入口

    EAppStatus RestrictGimbalCommand_();///< 限制控制命令范围

};

} // namespace my_engineer

#endif // MOD_GIMBAL_HPP
