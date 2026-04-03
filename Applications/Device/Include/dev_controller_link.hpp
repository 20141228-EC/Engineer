/******************************************************************************
 * @brief
 *
 * @file         dev_controller_link.hpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V2.0
 * @date         2025-04-05
 * @LastEditors  Ciallo(1002046597@qq.com)
 * @LastEditTime 2026-01-17
 *
 * @copyright    Copyright (c) 2025
 *
 ******************************************************************************/

#ifndef DEV_CONTROLLER_LINK_HPP
#define DEV_CONTROLLER_LINK_HPP

#include "dev_common.hpp"

#include "inf_uart.hpp"
#include "algo_crc.hpp"

namespace my_engineer {

/*------------------------------ 状态标志位定义 ------------------------------*/
// ControllerData (控制器 -> 机器人)
#define STATUS_CONTROLLER_OK      (1 << 0)  // bit0: 控制器状态OK
#define STATUS_RETURN_SUCCESS     (1 << 1)  // bit1: 归位成功标志
#define STATUS_TOGGLE_MASK        (0x03 << 2)  // bit2-3: 拨杆档位
#define STATUS_TOGGLE_SHIFT       2
#define STATUS_GRIPPER            (1 << 4)  // bit4: 夹爪闭合
#define STATUS_REGRIP             (1 << 5)  // bit5: 夹爪二次夹紧请求（脉冲）

// RobotData (机器人 -> 控制器)
#define STATUS_ASK_RESET          (1 << 0)  // bit0: 要求复位
#define STATUS_CONTROLLED         (1 << 1)  // bit1: 被控制器控制中
#define STATUS_ROBOT_INIT_OK      (1 << 4)  // bit4: 机器人初始化完成

/**
 * @brief 臂部角度结构体（6轴，浮点直传，用于 ControllerDataPkg）
 * 总大小: 6 × 4 = 24 bytes
 */
struct SArmAnglesPkg {
	float yaw = 0.f;        ///< Yaw角度 (deg)
	float pitch1 = 0.f;     ///< Pitch1角度 (deg)
	float pitch2 = 0.f;     ///< Pitch2角度 (deg)
	float pitch3 = 0.f;	 	///< Pitch3角度 (deg)
	float roll = 0.f;       ///< Roll角度 (deg)
	float pitch_end = 0.f;  ///< PitchEnd角度 (deg)
} __packed;

/**
 * @brief 压缩角度结构体（6轴，用于 RobotDataPkg）
 * int16存储，精度0.01°，范围±327.67°
 * 总大小: 6 × 2 = 12 bytes
 */
struct SArmAnglesCompressed {
	int16_t yaw = 0;        ///< Yaw角度 (×100)
	int16_t pitch1 = 0;     ///< Pitch1角度 (×100)
	int16_t pitch2 = 0;     ///< Pitch2角度 (×100)
	int16_t pitch3 = 0;     ///< Pitch3角度 (×100)
	int16_t roll = 0;       ///< Roll角度 (×100)
	int16_t pitch_end = 0;  ///< PitchEnd角度 (×100)
} __packed;

/**
 * @brief 力矩/电流反馈结构体（6轴，用于 RobotDataPkg）
 * int16存储，直接使用电机反馈原始值
 * 总大小: 6 × 2 = 12 bytes
 */
struct SArmTorqueCompressed {
	int16_t yaw = 0;        ///< Yaw电流 (原始值)
	int16_t pitch1 = 0;     ///< Pitch1力矩 (原始值)
	int16_t pitch2 = 0;     ///< Pitch2力矩 (原始值)
	int16_t pitch3 = 0;     ///< Pitch3力矩 (原始值)
	int16_t roll = 0;       ///< Roll电流 (原始值)
	int16_t pitch_end = 0;  ///< PitchEnd电流 (原始值)
} __packed;

/**
 * @brief 控制器通信设备类
 *
 */
class CDevControllerLink final: public CDevBase {
public:

	// 定义控制器通信设备初始化参数结构体
	struct SDevInitParam_ControllerLink: public SDevInitParam_Base{
		EInterfaceID interfaceID = EInterfaceID::INF_NULL; ///< 串口ID
	};

	/**
	 * @brief 通信数据包头
	 * @note  与裁判系统协议兼容
	 */
	struct SPkgHeader {
		uint8_t SOF = 0xA5;       ///< 帧头
		uint16_t pkgLen = 0;      ///< 数据长度
		uint8_t seq = 0;          ///< 包序号
		uint8_t CRC8 = 0x00;      ///< CRC8校验
		uint16_t cmd_Id = 0x0000; ///< 命令ID
	} __packed;  // 7字节

	enum EPackageID: uint8_t {
		ID_NULL = 0,
		ID_CONTROLLER_DATA,      ///< 0x0302 控制器-->机器人数据
		ID_ROBOT_DATA,           ///< 0x0309 机器人-->控制器数据
		ID_CHOSELEVEL_DATA,      ///< 0x0306 自定义控制器-->选手端 (8字节, 30Hz) 
		ID_ROBOT_TO_CLIENT_DATA, ///< 0x0310 机器人-->自定义客户端数据 (150字节, 50Hz)
	};

	/**
	 * @brief 控制器数据包 (Controller -> Robot)
	 * 数据段大小: 30 bytes (满足30字节限制)
	 * 完整包大小: 7(header) + 30(data) + 2(CRC16) = 39 bytes
	 */
	struct SControllerDataPkg {
		SPkgHeader header;
		uint8_t status_flags = 0;           ///< 状态标志位 (bit-packed)      1B
		SArmAnglesPkg arm;                  ///< 单臂6轴角度 (float)          24B
		int8_t rocker_X = 0;                ///< 摇杆X (-100~100)             1B
		int8_t rocker_Y = 0;                ///< 摇杆Y (-100~100)             1B
		uint8_t reserved[3] = {0};          ///< 保留字段                      3B
		uint16_t CRC16 = 0x0000;            ///< CRC16校验
	} __packed controllerData_info_pkg = { };

	/**
	 * @brief 机器人数据包 (Robot -> Controller)
	 * 数据段大小: 30 bytes (满足30字节限制)
	 * 完整包大小: 7(header) + 30(data) + 2(CRC16) = 39 bytes
	 */
	struct SRobotDataPkg {
		SPkgHeader header;
		uint8_t status_flags = 0;              ///< 状态标志位           1B
		SArmAnglesCompressed arm;              ///< 臂部角度 (int16)    12B
		SArmTorqueCompressed torque;           ///< 臂部力矩/电流       12B
		int8_t reserved[5] = {0};              ///< 保留字段             5B
		uint16_t CRC16 = 0x0000;               ///< CRC16校验
	} __packed robotData_info_pkg = { };

	/*for chose level*/
	struct SChoseLevelDataPkg {///<用来自定义控制器来模拟鼠标
		SPkgHeader header;
		uint8_t Key_value1;
		uint8_t Key_value2;
		uint16_t x_position:12;
    	uint16_t mouse_left:4;
    	uint16_t y_position:12;
    	uint16_t mouse_right:4;
		int8_t reserved[1] = {0}; 
		uint16_t CRC16 = 0x0000;
	}__packed choseLevelData_info_pkg = { };

	struct SClientDataPkg {   ///<机器人发给自定义客户端的数据
        SPkgHeader header;             ///< 包头 (cmd_Id=0x0310)
        uint8_t data[150] = {0};       ///< 自定义数据 (最大150字节)
        uint16_t CRC16 = 0x0000;       ///< CRC16校验
    }__packed clientData_info_pkg = { };   
	

	enum class EControllerLinkStatus {
		RESET,
		OFFLINE,
		ONLINE,
	} controllerLinkStatus = EControllerLinkStatus::RESET;

	/*Please increase or decrease difficulty based on the season  */
	enum class EExchangeLevel {
		NONE,
		FIRST,   ///< 一级兑换
		SECOND,  ///< 二级兑换
		THIRD,   ///< 三级兑换
		FOURTH,  ///< 四级兑换
	} exchangeLevel = EExchangeLevel::NONE;

	CDevControllerLink() { deviceType = EDevType::DEV_CONTROLLER_LINK; }

	EAppStatus InitDevice(const SDevInitParam_Base *pStructInitParam) override;

	EAppStatus SendPackage(EPackageID packageID, SPkgHeader &packageHeader);

	/*------------- 角度压缩/解压（仅用于 RobotDataPkg 的 int16 角度） -------------*/
	static inline int16_t CompressAngle(float angle) {
		return static_cast<int16_t>(angle * 100.0f);
	}
	static inline float DecompressAngle(int16_t compressed) {
		return static_cast<float>(compressed) / 100.0f;
	}

private:

	CInfUART *uartInterface_ = nullptr; ///< 串口设备指针

	std::array<uint8_t, 512> rxBuffer_ = {0};

	uint32_t rxTimestamp_ = 0; ///< 接收时间戳

	void UpdateHandler_() override;

	void HeartbeatHandler_() override;

	EAppStatus ResolveRxPackage_();
};

/*--------------------------------Key-Value Mapping Table---------------------------------------------*/
#define A_KEY_VALUE 65
#define B_KEY_VALUE 66
#define C_KEY_VALUE 67
#define D_KEY_VALUE 68
#define E_KEY_VALUE 69
#define F_KEY_VALUE 70
#define G_KEY_VALUE 71
#define H_KEY_VALUE 72
#define I_KEY_VALUE 73
#define J_KEY_VALUE 74
#define K_KEY_VALUE 75
#define L_KEY_VALUE 76
#define M_KEY_VALUE 77
#define N_KEY_VALUE 78
#define O_KEY_VALUE 79
#define P_KEY_VALUE 80
#define Q_KEY_VALUE 81
#define R_KEY_VALUE 82
#define S_KEY_VALUE 83
#define T_KEY_VALUE 84
#define U_KEY_VALUE 85
#define V_KEY_VALUE 86
#define W_KEY_VALUE 87
#define X_KEY_VALUE 88
#define Y_KEY_VALUE 89
#define Z_KEY_VALUE 90

} // namespace my_engineer

#endif // DEV_CONTROLLER_LINK_HPP
