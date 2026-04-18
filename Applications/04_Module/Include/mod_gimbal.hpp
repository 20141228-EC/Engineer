/**
 * @file mod_gimbal.hpp
 * @author ciallo
 * @brief 云台模块
 * @version 2.0
 * @date 2026-03-06
 *
 * @copyright Copyright (c) 2026
 *
 * @details
 */

#ifndef MOD_GIMBAL_HPP
#define MOD_GIMBAL_HPP

#include "mod_common.hpp"

#define GIMBAL_VISUAL_MOTOR_MOTOR_DIR 1     ///< 图传yaw电机方向 (1=正向, -1=反向)
#define GIMBAL_VISUAL_MOTOR_INIT_ANGLE 0    ///< 图传yaw初始角度 (度)

#define deg2rad(x) ((x) * 0.017453292519943295769236907684886)
#define rad2deg(x) ((x) * 57.295779513082320876798154814105)

namespace my_engineer {

/**
 * @brief 云台模块
 *
 * @details 仅管理图传yaw轴 (DM_MIT电机)
 *          电机通过MIT位置控制模式驱动，带低通滤波平滑输出
 */
class CModGimbal final: public CModBase{
public:

	/// 云台模块初始化参数
	struct SModInitParam_Gimbal: public SModInitParam_Base{
		EDeviceID yawVisualMotorID = EDeviceID::DEV_NULL;   ///< 图传yaw电机设备ID
		CInfCAN::CCanTxNode *yawVisualMotorTxNode = nullptr;///< CAN发送节点 (DM_MIT自发, 此处未使用)
		float_t MIT_YAW_kp = 0.0f; ///< MIT控制器位置刚度系数 (0-500 N/rad)
		float_t MIT_YAW_kd = 0.0f; ///< MIT控制器阻尼系数 (0-5 N·s/rad)
	};

	/// 云台状态信息 
	struct SGimbalInfo{
		EVarStatus isModuleAvailable = false;       ///< 模块是否可用
		EVarStatus isIntoControll = false;       ///< 是否进入了自定义控制器控制
		bool isPositArrived_Visualyaw = false;      ///< 图传yaw是否到达目标角度
		float_t angle_visualyaw = 0.f;              ///< 图传yaw当前角度 (度)
	} gimbalInfo;

	/// 云台控制命令 
	struct SGimbalCmd{
		EVarStatus isAutoCtrl = false;              ///< 是否处于自动控制模式
		float_t set_visualyaw = 0.f;                ///< 图传yaw目标角度 (度, 范围0-360)
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
			bool isAngleArrived = false;     ///< 是否到达目标 (误差 < 2度)
		}VisuallyawInfo;

		/// 图传yaw轴控制命令
		struct SVisuallyawCmd {
			float_t setAngle = 0.0f;        ///< 目标角度 (度)
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

    void UpdateHandler_() final;        ///< 主循环更新 (1000Hz, 由 StartUpdateTask 调用)
    void HeartbeatHandler_() final;     ///< 心跳检测 (100Hz)
    EAppStatus CreateModuleTask_() final;///< 创建模块FreeRTOS任务

    static void StartGimbalModuleTask(void *argument); ///< 模块任务入口 (FSM状态机)

    EAppStatus RestrictGimbalCommand_();///< 限制控制命令范围

};

} // namespace my_engineer

#endif // MOD_GIMBAL_HPP
