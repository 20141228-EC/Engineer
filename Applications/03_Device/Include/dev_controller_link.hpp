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

// /*------------------------------ 状态标志位定义 ------------------------------*/
// // ControllerData (控制器 -> 机器人)
// #define STATUS_CONTROLLER_OK      (1 << 0)  // bit0: 控制器状态OK
// #define STATUS_RETURN_SUCCESS     (1 << 1)  // bit1: 归位成功标志
// #define STATUS_TOGGLE_MASK        (0x03 << 2)  // bit2-3: 拨杆档位 (0-3)
// #define STATUS_TOGGLE_SHIFT       2
// #define STATUS_GRIPPER            (1 << 4)  // bit4: 夹爪闭合
// #define STATUS_REGRIP             (1 << 5)  // bit5: 夹爪二次夹紧请求（脉冲）

// // RobotData (机器人 -> 控制器)
// #define STATUS_ASK_RESET          (1 << 0)  // bit0: 要求复位
// #define STATUS_CONTROLLED         (1 << 1)  // bit1: 被控制器控制中
// #define STATUS_ROBOT_INIT_OK      (1 << 4)  // bit4: 机器人初始化完成

/**
 * @brief 臂部角度结构体（5轴，浮点直传，用于 ControllerDataPkg）
 * 总大小: 5 × 4 = 24 bytes
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
 * @brief 压缩角度结构体（5轴，用于 RobotDataPkg）
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
class CDevControllerLink final: public CDevBase{
public:

	// 定义控制器通信设备初始化参数结构体
	struct SDevInitParam_ControllerLink: public SDevInitParam_Base{
		EInterfaceID interfaceID = EInterfaceID::INF_NULL; ///< 串口ID
	};

	enum EPackageID: uint8_t {
		ID_NULL = 0,
		ID_CONTROLLER_DATA,
		ID_ROBOT_DATA,
	};

	struct SPkgHeader {
		uint8_t SOF = 0xA5; ///< 包头
		uint16_t pkgLen = 0; ///< 包长度
		uint8_t seq = 0; ///< 包序号
		uint8_t CRC8 = 0x00; ///< CRC8校验
		uint16_t cmd_Id = 0x0000; ///< 命令ID
	} __packed; //禁止编译器的内存对齐优化

	/**
	 * @brief 机器人状态控制包
	 * 数据段大小: 1 bytes 
	 */
	struct SRobotStatusFlags {
		uint8_t ask_reset : 1;  ///< bit0 - 要求复位
		uint8_t control_by_controller : 1; ///< bit1 - 被控制器控制中
		uint8_t robot_init_ok : 1;    ///< bit2-3 - 拨杆档位
		uint8_t p3_lock : 1;           ///<p3锁定标志
		uint8_t reserve : 4;
	} __packed robotStatusFlags_pkt = {};

	/**
	 * @brief 控制器状态控制包
	 * 数据段大小: 1 bytes 
	 */
	struct SControllerStatusFlags {
		uint8_t ask_reset : 1;  			///< bit0 - 要求复位
		uint8_t return_sucess : 1; 			///< bit1 - 归位成功
		uint8_t controller_init_ok : 1;    		///< bit2 - 控制器初始化完成
		uint8_t grip : 1;    				///< bit3 - 夹爪夹住
		uint8_t regrip : 1;    				///< bit4 - 夹爪二次夹紧请求
		uint8_t reserve : 3;
	} __packed ControllerStatusFlags_pkt = {};

	/**
	 * @brief 机器人数据包 (Robot -> Controller)
	 * 数据段大小: 30 bytes (满足30字节限制)
	 * 完整包大小: 7(header) + 30(data) + 2(CRC16) = 39 bytes
	 */
	struct SRobotDataPkg {
		SPkgHeader header;
		SRobotStatusFlags status_flags;        ///< 状态标志位           1B
		SArmAnglesCompressed arm;              ///< 臂部角度 (int16)    12B
		SArmTorqueCompressed torque;           ///< 臂部力矩/电流       12B
		int8_t reserved[5] = {0};              ///< 保留字段             5B
		uint16_t CRC16 = 0x0000;               ///< CRC16校验
	} __packed robotData_info_pkg = { };

	/**
	 * @brief 控制器数据包 (Controller -> Robot)
	 * 数据段大小: 30 bytes (满足30字节限制)
	 * 完整包大小: 7(header) + 30(data) + 2(CRC16) = 39 bytes
	 */
	struct SControllerDataPkg {
		SPkgHeader header;
		SControllerStatusFlags status_flags ;           ///< 状态标志位 (bit-packed)      1B
		SArmAnglesPkg arm;                  			///< 单臂5轴角度 (float)          24B
		int8_t rocker_X = 0;               				///< 摇杆X (-100~100)             1B
		int8_t rocker_Y = 0;                			///< 摇杆Y (-100~100)             1B
		uint8_t reserved[3] = {0};          			///< 保留字段                      7B
		uint16_t CRC16 = 0x0000;            			///< CRC16校验
	} __packed controllerData_info_pkg = {};

	enum class EControllerLinkStatus {
		RESET,
		OFFLINE,
		ONLINE,
	} controllerLinkStatus = EControllerLinkStatus::RESET;

	CDevControllerLink() {deviceType = EDevType::DEV_CONTROLLER_LINK; }

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

	/**
	 * @brief 双缓冲区：消除 UART ISR 与主循环之间的数据竞态
	 *
	 * rxBuffers_[rxWriteIdx_]  —— ISR 写入端
	 * rxBuffers_[1-rxWriteIdx_] —— 主循环读取端
	 * 切换时仅交换索引（__disable_irq 保护），耗时 < 1μs
	 */
	std::array<uint8_t, 512> rxBuffers_[2] = {};
	volatile uint8_t rxWriteIdx_ = 0;    ///< ISR 当前写入的缓冲区索引, 0或1
	volatile bool rxNewData_ = false;     ///< ISR 置位，主循环清除

	uint32_t rxTimestamp_ = 0; ///< 接收时间戳

	void UpdateHandler_() override;

	void HeartbeatHandler_() override;

	EAppStatus ResolveRxPackage_(std::array<uint8_t, 512> &buffer);
};

} // namespace my_engineer

#endif // DEV_CONTROLLER_LINK_HPP