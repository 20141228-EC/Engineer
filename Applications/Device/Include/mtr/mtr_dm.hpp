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

#define Pos_MAX  3.141592f
#define Pos_MIN -3.141592f
#define Spd_MAX  30.0f
#define Spd_MIN -30.0f
#define KP_MAX 500.0f
#define KP_MIN 0.0f
#define KD_MAX 5.0f
#define KD_MIN 0.0f
#define Tf_MAX  10.0f
#define Tf_MIN -10.0f 

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

  // 是否使用角度转位置
  EVarStatus useAngleToPosit_ = false;

  // 编码器分辨率
  uint32_t encoderResolution_ = 8192;                                 

  // 上一次的角度
  int32_t lastAngle_ = 0;

  void UpdateHandler_() override;

  void HeartbeatHandler_() override;

  // 获取当前位置,经过减速比的处理
  int32_t getPosition_();

public:

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
  EAppStatus EnableMotor(CDevMtr *mtr);
  EAppStatus DisableMotor(CDevMtr *mtr);

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
