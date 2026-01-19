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
#define STATUS_CONTROLLER_OK      (1 << 0)  // bit0: 控制器状态OK
#define STATUS_RETURN_SUCCESS     (1 << 1)  // bit1: 归位成功标志
#define STATUS_TOGGLE_MASK        (0x03 << 2)  // bit2-3: 拨杆档位 (0-3)
#define STATUS_TOGGLE_SHIFT       2
#define STATUS_GRIPPER_LEFT       (1 << 4)  // bit4: 左夹爪闭合
#define STATUS_GRIPPER_RIGHT      (1 << 5)  // bit5: 右夹爪闭合

/**
 * @brief 压缩角度结构体（5轴）
 * 使用int16存储，精度0.01°，范围±327.67°
 * 总大小: 5 × 2 = 10 bytes
 */
struct SArmAnglesCompressed {
	int16_t yaw = 0;        ///< Yaw角度 (×100)
	int16_t pitch1 = 0;     ///< Pitch1角度 (×100)
	int16_t pitch2 = 0;     ///< Pitch2角度 (×100)
	int16_t roll = 0;       ///< Roll角度 (×100)
	int16_t pitch_end = 0;  ///< PitchEnd角度 (×100)
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
	 * @brief 控制器数据包 (Controller -> Robot)
	 * 数据段大小: 24 bytes
	 * 完整包大小: 7(header) + 24(data) + 2(CRC16) = 33 bytes
	 */
	struct SControllerDataPkg {
		SPkgHeader header;
		uint8_t status_flags = 0;           ///< 状态标志位 (bit-packed)
		SArmAnglesCompressed left_arm;      ///< 左臂5轴角度 (10 bytes)
		SArmAnglesCompressed right_arm;     ///< 右臂5轴角度 (10 bytes)
		int8_t rocker_LX = 0;               ///< 左臂roll_end增量 (-100~100)，控制第6轴
		int8_t rocker_RX = 0;               ///< 右臂roll_end增量 (-100~100)，控制第6轴
		int8_t rocker_RY = 0;               ///< 底盘前进 (-100~100，仅底盘模式有效)
		uint16_t CRC16 = 0x0000;            ///< CRC16校验
	} __packed controllerData_info_pkg = { };

	/**
	 * @brief 机器人数据包 (Robot -> Controller)
	 * 用于同步机器人当前位置到控制器
	 */
	struct SRobotDataPkg {
		SPkgHeader header;
		uint8_t status_flags = 0;           ///< 状态标志位 (bit-packed)
		SArmAnglesCompressed left_arm;      ///< 左臂5轴角度 (10 bytes)
		SArmAnglesCompressed right_arm;     ///< 右臂5轴角度 (10 bytes)
		int8_t reserved[3] = {0};           ///< 保留字段
		uint16_t CRC16 = 0x0000;            ///< CRC16校验
	} __packed robotData_info_pkg = { };

	enum class EControllerLinkStatus {
		RESET,
		OFFLINE,
		ONLINE,
	} controllerLinkStatus = EControllerLinkStatus::RESET;

	CDevControllerLink() {deviceType = EDevType::DEV_CONTROLLER_LINK; }

	EAppStatus InitDevice(const SDevInitParam_Base *pStructInitParam) override;

	EAppStatus SendPackage(EPackageID packageID, SPkgHeader &packageHeader);

	/*------------------------------ 角度压缩/解压工具函数 ------------------------------*/
	/**
	 * @brief 压缩角度 (float -> int16)
	 * @param angle 角度值 (单位: 度)
	 * @return int16_t 压缩后的值 (×100)
	 */
	static inline int16_t CompressAngle(float_t angle) {
		return static_cast<int16_t>(angle * 100.0f);
	}

	/**
	 * @brief 解压角度 (int16 -> float)
	 * @param compressed 压缩后的值 (×100)
	 * @return float_t 角度值 (单位: 度)
	 */
	static inline float_t DecompressAngle(int16_t compressed) {
		return static_cast<float_t>(compressed) / 100.0f;
	}

private:

	CInfUART *uartInterface_ = nullptr; ///< 串口设备指针

	std::array<uint8_t, 512> rxBuffer_ = {0};

	uint32_t rxTimestamp_ = 0; ///< 接收时间戳

	void UpdateHandler_() override;

	void HeartbeatHandler_() override;

	EAppStatus ResolveRxPackage_();
};

} // namespace my_engineer

#endif // DEV_CONTROLLER_LINK_HPP
