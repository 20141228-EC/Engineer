/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-11
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-11   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#ifndef RP_MOD_GIMBAL_HPP
#define RP_MOD_GIMBAL_HPP

#include "mod_common.hpp"

namespace robotpilots {

class CModGimbal final : public CModInstance {
public:

  struct SModGimbalInitParam : public SModInitParam {
    TIM_HandleTypeDef *gimbalPitchHalTimHandle = nullptr;
    uint32_t gimbalPitchTimChannel = 0;
    uint32_t gimbalMtrCanStdID = 0x000;
    EInterfaceID gimbalMtrCanInfID = EInterfaceID::INF_NULL;
    EDeviceID liftMotorID_L = EDeviceID::DEV_NULL;
    EDeviceID liftMotorID_R = EDeviceID::DEV_NULL;
    CPidController::SPidControllerInitParam liftPosPidParam;
    CPidController::SPidControllerInitParam liftSpdPidParam;
  };

  struct SGimbalInfo {
    EFuncStatus isModuleAvailable = false;    ///< Is Gimbal Module Available
    float_t posit_Lift            = 0.0f;     ///< Gimbal Lift Posit (Range: 0mm ~ 280mm)
    uint32_t step_pitch           = 0;        ///< Gimbal Pitch Step (0 - MID, 1 - UP, 2 - DOWN, 3 - VISION)
    bool isPositArrived_Lift      = false;    ///< Is Gimbal Lift Posit Arrived
  } gimbalInfo;

  struct SGimbalCmd {
    bool isAutoCtrl       = false;         ///< Is Auto Control
    float_t setPosit_Lift = 150.0f;        ///< Set Gimbal Lift Posit (Range: 0mm ~ 280mm)
    uint32_t setStep_Pitch = 0;            ///< Set Gimbal Pitch Step (0 - MID, 1 - UP, 2 - DOWN)
  } gimbalCmd;

  CModGimbal() = default;

  explicit CModGimbal(SModInitParam &param) { InitModule(param); }

  ~CModGimbal() final { UnregisterModule_(); }

  ERpStatus InitModule(SModInitParam &param) final;

private:

  /**
   * @brief Gimbal Lift Component Class
   */
  class CComLift final : public CComponent {
  public:

    enum { L = 0, R = 1 };

    const int32_t rangeLimit = 880000;

    struct SLiftInfo{
      int32_t posit = 0;
      bool isPositArrived = false;
    } liftInfo;

    struct SLiftCmd{
      int32_t setPosit = 0;
    } liftCmd;

    std::array<CMtrInstance *, 2> motor = {nullptr};

    CPidController pidPosCtrl;

    CPidController pidSpdCtrl;

    std::array<int16_t, 2> mtrOutputBuffer = {0};

    ERpStatus InitComponent(SModInitParam &param) final;

    ERpStatus UpdateComponent() final;

    static int32_t PhyPositToMtrPosit(float_t phyPosit);

    static float_t MtrPositToPhyPosit(int32_t mtrPosit);

    ERpStatus _UpdateOutput(float_t posit);

  } comLift_;

  class CComPitch final : public CComponent {
  public:

    struct SPitchCmd {
      uint32_t setStep = 0;
    } pitchCmd;

    TIM_HandleTypeDef *halTimHandle = nullptr;

    uint32_t timChannel = 0;

    ERpStatus InitComponent(SModInitParam &param) final;

    ERpStatus UpdateComponent() final;

  } comPitch_;

  CCanInterface::CCanTxNode gimbalMtrCanTxNode_;

  void UpdateHandler_() final;

  void HeartbeatHandler_() final;

  ERpStatus CreateModuleTask_() final;

  static void StartGimbalModuleTask(void *arg);

  ERpStatus RestrictGimbalCommand_();

};

} // namespace robotpilots

#endif // RP_MOD_GIMBAL_HPP
