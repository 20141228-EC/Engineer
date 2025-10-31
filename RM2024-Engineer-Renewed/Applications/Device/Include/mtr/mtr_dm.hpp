/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-04-17
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-04-17   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */
#ifndef RP_MTR_DM_HPP
#define RP_MTR_DM_HPP

#include "mtr_common.hpp"
#include "inf_can.hpp"

namespace robotpilots {

class CMtrDm : public CMtrInstance {
protected:

  CCanInterface *canInterface_ = nullptr;

  CCanInterface::CCanRxNode canRxNode_;

  EFuncStatus useAngleToPosit_ = false;

  uint32_t encoderResolution_ = 8192;

  int32_t lastAngle_ = 0;

  void UpdateHandler_() override;

  void HeartbeatHandler_() override;

  int32_t getPosition_();

public:

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
  };

  struct SMtrDmInitParam : public SMtrInitParam {
    EDmMtrID dmMtrID = EDmMtrID::ID_NULL;
    EFuncStatus useAngleToPosit = false;
  };

  EDmMtrID dmMotorID = EDmMtrID::ID_NULL;

  ERpStatus InitDevice(const SDevInitParam *pStruct) override;

  ERpStatus FillCanTxBuffer(DataBuffer<uint8_t> &buffer, const int16_t current);

  ERpStatus FillCanTxBuffer(uint8_t *buffer, const int16_t current);

  static ERpStatus FillCanTxBuffer(CMtrInstance *mtr, DataBuffer<uint8_t> &buffer, const int16_t current);

  static ERpStatus FillCanTxBuffer(CMtrInstance *mtr, uint8_t *buffer, const int16_t current);

};

} // namespace robotpilots

#endif // RP_MTR_DM_HPP
