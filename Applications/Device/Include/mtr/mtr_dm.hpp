/**
 * @file mtr_dm.hpp
 * @author Sassinak
 * @brief 定义基于电机设备基类的DM电机设备类
 * @version 1.0
 * @date 2025-05-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef MTR_DM_HPP
#define MTR_DM_HPP

#include "mtr_common.hpp"
#include "inf_can.hpp"

#define KP_MAX 500.0f
#define KP_MIN 0.0f
#define KD_MAX 5.0f
#define KD_MIN 0.0f

namespace my_engineer {

/**
 * @brief DM电机设备类
 *
 */
class CDevMtrDM : public CDevMtr {
protected:

  // 对应的can指针
  CInfCAN *pInfCAN_ = nullptr;

  // 定义接收节点
  CInfCAN::CCanRxNode canRxNode_;

  // 定义发送节点（MIT模式专用，电机自管理）
  CInfCAN::CCanTxNode canTxNode_mit_;

  // 是否使用角度转位置
  EVarStatus useAngleToPosit_ = false;

  // 编码器分辨率
  uint32_t encoderResolution_ = 8192;

  // 上一次的角度
  int32_t lastAngle_ = 0;

  // 上一次使能时间（用于节流）
  uint32_t lastEnableTime_ = 0;

  // MIT模式使能状态（防止心跳覆盖主动失能）
  bool motorEnabled_ = false;

  void UpdateHandler_() override;

  void HeartbeatHandler_() override;

  // 获取当前位置,经过减速比的处理
  int32_t getPosition_();

public:

  /**
   * @brief MIT模式参数范围（每个电机实例可独立配置）
   */
  struct SMitLimitParam {
    float_t Q_MAX   = 12.5f;   ///< 位置范围 ±Q_MAX (rad)
    float_t DQ_MAX  = 30.0f;   ///< 速度范围 ±DQ_MAX (rad/s)
    float_t TAU_MAX = 10.0f;   ///< 力矩范围 ±TAU_MAX (N·m)
  };

  /// MIT 参数范围（初始化时赋值）
  SMitLimitParam mitLimit_;

  /**
   * @brief 定义DM电机ID枚举类型
   * 
   */
  enum class EDmMtrID {
    ID_NULL = 0,
    ID_1,
    ID_2,
    ID_3,
    ID_4,
    ID_5,
    ID_6,
    ID_7,
    ID_8,
    ID_MIT,
  };

  /**
   * @brief DM电机设备初始化参数
   * 
   */
  struct SMtrInitParam_DM : public SMtrInitParam_Base {
    EDmMtrID dmMtrID = EDmMtrID::ID_NULL;
    EMotorControlMode dmMtrMode = EMotorControlMode::MODE_UNDEF;
    EVarStatus useAngleToPosit = false;
    /*only design for MIT mode*/
    float_t Kp = 0.0f;  // Proportional gain for MIT mode
    float_t Kd = 0.0f;  // Derivative gain for MIT
    uint32_t MIT_RxCANID = 0x000;
    uint32_t MIT_TxCANID = 0x000;
    // MIT 参数范围（不同型号电机可配置不同值）
    float_t Q_MAX   = 12.5f;   // 位置范围 ±Q_MAX (rad)   DM4310=12.5, DM3510=12.5
    float_t DQ_MAX  = 30.0f;   // 速度范围 ±DQ_MAX (rad/s) DM4310=30,   DM3510=280
    float_t TAU_MAX = 10.0f;   // 力矩范围 ±TAU_MAX (N·m)  DM4310=10,   DM3510=1
  };

  DataBuffer<uint8_t> DM_EnableBuffer = {
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFC,
  };

   DataBuffer<uint8_t> DM_DisableBuffer = {
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFD,
  };


  static int float_to_uint(float_t x_float, float_t x_min, float_t x_max, int bits);
  static float_t uint_to_float(int x_int, float_t x_min, float_t x_max, int bits);

  /**
   * @brief MIT模式控制（填充+发送一体）
   * @param kp 位置刚度 (0-500)
   * @param kd 阻尼系数 (0-5)
   * @param pos 目标位置 (rad)
   * @param spd 速度给定 (rad/s)
   * @param torq 力矩前馈 (N·m)
   */
  void Control_MIT(float_t kp, float_t kd, float_t pos, float_t spd, float_t torq);

  /**
   * @brief 使能电机（MIT模式，使用内部TxNode）
   */
  void EnableMotor();

  /**
   * @brief 失能电机（MIT模式，使用内部TxNode）
   */
  void DisableMotor();

  /**
   * @brief 保存当前位置为零点（MIT模式）
   */
  void SetZero();

  /**
   * @brief 清除电机错误（MIT模式）
   */
  void ClearError();

  /**
   * @brief 判断是否为MIT模式
   */
  bool IsMitMode() const { return dmMtrMode == EMotorControlMode::MODE_MIT; }

  EDmMtrID dmMotorID = EDmMtrID::ID_NULL; ///< DM电机ID
  EMotorControlMode dmMtrMode = EMotorControlMode::MODE_UNDEF;
  float_t Kp = 0.0f;  // Proportional gain for MIT mode
  float_t Kd = 0.0f;  // Derivative gain for MIT

  /**
   * @brief 初始化电机设备
   * 
   */
  EAppStatus InitDevice(const SDevInitParam_Base *pStructInitParam) override;

  /**
   * @brief 根据电机ID,用目标电流值填充发送数据帧的指定位置
   * 
   * @param buffer - 待填充的数据帧
   * @param current - 目标电流值
   */
  EAppStatus FillCanTxBuffer(DataBuffer<uint8_t> &buffer, const int16_t current);
  EAppStatus FillCanTxBuffer(uint8_t *buffer, const int16_t current);
  static EAppStatus FillCanTxBuffer(CDevMtr *mtr, DataBuffer<uint8_t> &buffer, const int16_t current);
  static EAppStatus FillCanTxBuffer(CDevMtr *mtr, uint8_t *buffer, const int16_t current);

  EAppStatus FillCanTxBuffer_MIT(DataBuffer<uint8_t> &buffer,const float_t pos,const float_t speed, const float_t torq,const float_t Kp,const float_t Kd);
  EAppStatus FillCanTxBuffer_MIT(uint8_t *buffer,const float_t pos,const float_t speed, const float_t torq,const float_t Kp,const float_t Kd);
  static EAppStatus FillCanTxBuffer_MIT(CDevMtr *mtr, DataBuffer<uint8_t> &buffer,const float_t pos,const float_t speed, const float_t torq,const float_t Kp,const float_t Kd);
  static EAppStatus FillCanTxBuffer_MIT(CDevMtr *mtr, uint8_t *buffer,const float_t pos,const float_t speed, const float_t torq,const float_t Kp,const float_t Kd);
	
};

} // namespace my_engineer

#endif // MTR_DM_HPP
