/**
 * @file        sys_vision.hpp
 * @version     1.0
 * @date        2024-05-06
 * @author      Morthine Xiang
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-06   <td>1.0         <td>Morthine Xiang  <td>First Create.
 * </table>
 */

#ifndef RP_SYS_VISION_HPP
#define RP_SYS_VISION_HPP

#include "sys_common.hpp"

namespace robotpilots {

class CSysVision final : public CSysInstance {
public:

  struct SSysVisionInitParam : public SSystemInitParam {
    EDeviceID visionDevID = EDeviceID::DEV_NULL;
  };

  struct SOreTankInfo {
    bool isFoundOreTank = false;
    float_t atti_YAW = 0.0f;
    float_t atti_PITCH = 0.0f;
    float_t atti_ROLL = 0.0f;
    float_t posit_X = 0.0f;
    float_t posit_Y = 0.0f;
    float_t posit_Z = 0.0f;
  };

  struct SUiPointInfo {
    bool isFoundOreTank = false;
    float_t pointPosit_X[5] = {0};
    float_t pointPosit_Y[5] = {0};
  };

  struct SSysVisionInfo{
    SOreTankInfo oreTank;
    SUiPointInfo uiPoint;
  } visionInfo;

  ERpStatus InitSystem(SSystemInitParam *pStruct) final;

private:

  CDevVision *vision_;

  void UpdateHandler_() final;

  void HeartbeatHandler_() final;

  ERpStatus UpdateOreTankInfo_();

  ERpStatus UpdateUiPointInfo_();

  static void StartSysVisionTask(void *arg);

};

extern CSysVision SysVision;

} // namespace robotpilots

#endif // RP_SYS_VISION_HPP
