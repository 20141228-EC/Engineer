/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-03-19
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-03-19   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#ifndef RP_MTR_DJI_HPP
#define RP_MTR_DJI_HPP

#include "mtr_common.hpp"
#include "inf_can.hpp"

namespace robotpilots {

class CMtrDji : public CMtrInstance {
protected:
  /**
   * @brief
   */
  CCanInterface *canInterface_ = nullptr;

  /**
   * @brief
   */
  CCanInterface::CCanRxNode canRxNode_;

  /**
   * @brief
   */
  EFuncStatus useStallMonit_ = false;

  /**
   * @brief
   */
  EMotorDataType stallMonitDataSrc_ = DATA_CURRENT;

  /**
   * @brief
   */
  uint32_t stallThreshold_ = 2000;

  /**
   * @brief
   */
  uint32_t stallTime_ = 200;

  /**
   * @brief
   */
  uint32_t stallCount_ = 0;

  /**
   * @brief
   */
  EFuncStatus useAngleToPosit_ = false;

  /**
   * @brief
   */
  uint32_t encoderResolution_ = 8192;

  /**
   * @brief
   */
  int32_t lastAngle_ = 0;

  /**
   * @brief
   */
  void HeartbeatHandler_() override;

  /**
   * @brief
   * @return
   */
  int32_t getPosition_();

public:

  /**
   * @brief
   */
  enum class EDjiMtrID {
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

  /**
   * @brief
   */
  struct SDjiMtrInitParam : public SMtrInitParam {
    EDjiMtrID djiMtrID = EDjiMtrID::ID_NULL;
    EFuncStatus useStallMonit = false;
    EMotorDataType stallMonitDataSrc = DATA_CURRENT;
    uint32_t stallThreshold = 2000;
    uint32_t stallTime = 250;
    EFuncStatus useAngleToPosit = false;
  };

  EDjiMtrID djiMtrID = EDjiMtrID::ID_NULL;

  /**
   * @brief
   * @param pStruct
   * @return
   */
  ERpStatus InitDevice(const SDevInitParam *pStruct) override;

  ERpStatus FillCanTxBuffer(DataBuffer<uint8_t> &buffer, const int16_t current);

  ERpStatus FillCanTxBuffer(uint8_t *buffer, const int16_t current);

  static ERpStatus FillCanTxBuffer(CMtrInstance *mtr, DataBuffer<uint8_t> &buffer, const int16_t current);

  static ERpStatus FillCanTxBuffer(CMtrInstance *mtr, uint8_t *buffer, const int16_t current);

};

} // namespace robotpilots

#include "mtr_m2006.hpp"
#include "mtr_m3508.hpp"

#endif // RP_MTR_DJI_HPP
