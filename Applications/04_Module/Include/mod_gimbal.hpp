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
#define GIMBAL_VISUAL_MOTOR_INIT_ANGLE 1    ///< 图传yaw初始角度 (度)

#define STORAGE_L_MOTOR_DIR 1
#define STORAGE_R_MOTOR_DIR 1
/*--------------------------存矿石电机初始化角度及转换系数---------------------------------*/

#define STORAGE_L_MOTOR_SCALE  182.04f    ///< KT电机编码器分辨率/360 = 65535/360 (encoder/degree)
#define STORAGE_R_MOTOR_SCALE  182.04f    ///< KT电机编码器分辨率/360 = 65535/360 (encoder/degree)
#define STORAGE_L_MOTOR_RANGE 0
#define STORAGE_R_MOTOR_RANGE 0
#define STORAGE_L_MOTOR_INIT_POSIT 0
#define STORAGE_R_MOTOR_INIT_POSIT 0

/*--------------------------机械装配偏差---------------------------------*/
#define POSIT_STORAGE_L_MACH 0
#define POSIT_STORAGE_R_MACH 0
#define POSIT_STORAGE_L_MACH_PHY 0
#define POSIT_STORAGE_R_MACH_PHY 0

#define STORAGE_MOTOR_ARRIVE_ERR 700

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
		EDeviceID storageMotorID_L = EDeviceID::DEV_NULL;   ///< 左存矿电机设备ID
		EDeviceID storageMotorID_R = EDeviceID::DEV_NULL;   ///< 右存矿电机设备ID
		CInfCAN::CCanTxNode *yawVisualMotorTxNode = nullptr;///< CAN发送节点 (DM_MIT自发, 此处未使用)
		CInfCAN::CCanTxNode *storageMotorTxNode_L = nullptr; ///< 左存矿电机CAN发送节点
		CInfCAN::CCanTxNode *storageMotorTxNode_R = nullptr; ///< 右存矿电机CAN发送节点
		float_t MIT_YAW_kp = 0.0f; ///< MIT控制器位置刚度系数 (0-500 N/rad)
		float_t MIT_YAW_kd = 0.0f; ///< MIT控制器阻尼系数 (0-5 N·s/rad)
		CAlgoPid::SAlgoInitParam_Pid storagePosPidParam_L;
		CAlgoPid::SAlgoInitParam_Pid storageSpdPidParam_L;
		CAlgoPid::SAlgoInitParam_Pid storagePosPidParam_R;
		CAlgoPid::SAlgoInitParam_Pid storageSpdPidParam_R;
	};

	/// 云台状态信息 
	struct SGimbalInfo{
		EVarStatus isModuleAvailable = false;       ///< 模块是否可用
		EVarStatus isIntoControll = false;       ///< 是否进入了自定义控制器控制
		bool isPositArrived_Visualyaw = false;      ///< 图传yaw是否到达目标角度
		bool isPositArrived_Storage_L = false;      ///< 左存矿电机是否到达目标角度
		bool isPositArrived_Storage_R = false;      ///< 右存矿电机是否到达目标角度
		bool isStorageAvailable = false;            ///< 存矿组件是否配置了电机
		float_t angle_visualyaw = 0.f;              ///< 图传yaw当前角度 (度)
		int32_t posit_storage_L = 0;                ///< 左存矿电机当前位置
		int32_t posit_storage_R = 0;                ///< 右存矿电机当前位置
	} gimbalInfo;

	/// 云台控制命令 
	struct SGimbalCmd{
		EVarStatus isAutoCtrl = false;              ///< 是否处于自动控制模式
		float_t set_visualyaw = 0.f;                ///< 图传yaw目标角度 (度, 范围0-360)
		int32_t set_posit_storage_L = STORAGE_L_MOTOR_INIT_POSIT; ///< 左存矿目标位置
		int32_t set_posit_storage_R = STORAGE_R_MOTOR_INIT_POSIT; ///< 右存矿目标位置
	} gimbalCmd;
    
	enum class EStorageSlot : uint8_t { //用作模块层暴露接口
        UP = 1,
        L_UP = 2,
        L_DOWN = 3,
        DOWN = 4,
        R_DOWN = 5,
        R_UP = 6,
    };
	CModGimbal() = default;

	explicit CModGimbal(SModInitParam_Gimbal &param) { InitModule(param); }

	~CModGimbal() final { UnregisterModule_(); }

	EAppStatus InitModule(SModInitParam_Base &param) final;

	void ChooseStoreOre(EStorageSlot ore);

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

	class CComStorage:public CComponentBase{
	public:
		enum {L = 0, R = 1};
		enum class Eore_station{UP = 1, L_UP = 2,L_DOWN = 3,DOWN = 4,R_DOWN = 5,R_UP = 6}ore_station_ = Eore_station::UP;

		//矿石角度
		static constexpr float_t OreStationAngle[] = {
			0.0f,    ///< [0] 未使用
			0.0f,    ///< [1] UP
			60.0f,   ///< [2] L_UP
			120.0f,  ///< [3] L_DOWN
			180.0f,  ///< [4] DOWN
			240.0f,  ///< [5] R_DOWN
			300.0f   ///< [6] R_UP
		};

		struct SStorageInfo {
			int32_t posit_L_storage = 0;           ///< 左存矿当前位置
			int32_t posit_R_storage = 0;           ///< 右存矿当前位置
			bool isPositArrived_L_storage = false; ///< 左存矿位置是否到达目标
			bool isPositArrived_R_storage = false; ///< 右存矿位置是否到达目标
			bool isAvailable = false;
		} storageInfo;

		struct SStorageCmd {
			int32_t setPosit_L_storage = STORAGE_L_MOTOR_INIT_POSIT; ///< 左存矿目标位置
			int32_t setPosit_R_storage = STORAGE_R_MOTOR_INIT_POSIT; ///< 右存矿目标位置
		} storageCmd;

		CAlgoPid pidPosCtrl_L_storage;
		CAlgoPid pidSpdCtrl_L_storage;
		CAlgoPid pidPosCtrl_R_storage;
		CAlgoPid pidSpdCtrl_R_storage;

		std::array<int16_t, 2> mtrOutputBuffer = {0};
		std::array<CDevMtr*, 2> motor = {nullptr, nullptr};
		std::array<CInfCAN::CCanTxNode*, 2> mtrCanTxNode = {nullptr, nullptr};

		static int32_t PhyPositToMtrPosit_L(float_t phyPosit);
		static int32_t PhyPositToMtrPosit_R(float_t phyPosit);
		static float_t MtrPositToPhyPosit_L(int32_t mtrPosit);
		static float_t MtrPositToPhyPosit_R(int32_t mtrPosit);

		// 初始化组件
		EAppStatus InitComponent(SModInitParam_Base &param) final;

		// 更新组件
		EAppStatus UpdateComponent() final;

		EAppStatus _UpdateOutput(float_t posit_L, float_t posit_R);
		EAppStatus _UpdateOutput_L(float_t posit_L);
		EAppStatus _UpdateOutput_R(float_t posit_R);

		void ChooseStoreOre(Eore_station ore_station_);

	} comstorage_;

    void UpdateHandler_() final;        ///< 主循环更新 (1000Hz, 由 StartUpdateTask 调用)
    void HeartbeatHandler_() final;     ///< 心跳检测 (100Hz)
    EAppStatus CreateModuleTask_() final;///< 创建模块FreeRTOS任务

    static void StartGimbalModuleTask(void *argument); ///< 模块任务入口 (FSM状态机)

    EAppStatus RestrictGimbalCommand_();///< 限制控制命令范围

};

} // namespace my_engineer

#endif // MOD_GIMBAL_HPP
