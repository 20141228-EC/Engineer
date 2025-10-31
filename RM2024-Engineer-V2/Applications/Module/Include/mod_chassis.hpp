/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-10
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-10   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#ifndef RP_MOD_CHASSIS_HPP
#define RP_MOD_CHASSIS_HPP

#include "mod_common.hpp"

namespace  robotpilots {

/**
 * @brief Chassis Module Class
 */
class CModChassis final : public CModInstance {
public:

  struct SModChassisInitParam : public SModInitParam {
    EDeviceID memsDevID = EDeviceID::DEV_NULL;
    uint32_t chassisMtrCanStdID = 0x000;
    EInterfaceID chassisMtrCanInfID = EInterfaceID::INF_NULL;
    EDeviceID wheelsetMotorID_LF = EDeviceID::DEV_NULL;
    EDeviceID wheelsetMotorID_RF = EDeviceID::DEV_NULL;
    EDeviceID wheelsetMotorID_LB = EDeviceID::DEV_NULL;
    EDeviceID wheelsetMotorID_RB = EDeviceID::DEV_NULL;
    CPidController::SPidControllerInitParam wheelsetSpdPidParam;
    CPidController::SPidControllerInitParam yawCorrectionPidParam;
  };

  struct SChassisInfo {
    EFuncStatus isModuleAvailable = false;    ///< Is Chassis Module Available
    float_t speed_X = 0;                      ///< Chassis Speed X (Range: -100% ~ 100%)
    float_t speed_Y = 0;                      ///< Chassis Speed Y (Range: -100% ~ 100%)
    float_t speed_W = 0;                      ///< Chassis Speed W (Range: -100% ~ 100%)
  } chassisInfo;

  struct SChassisCmd {
    float_t speed_X = 0;    ///< Chassis Speed X (Range: -100% ~ 100%)
    float_t speed_Y = 0;    ///< Chassis Speed Y (Range: -100% ~ 100%)
    float_t speed_W = 0;    ///< Chassis Speed W (Range: -100% ~ 100%)
  } chassisCmd;

  CModChassis() = default;

  explicit CModChassis(SModInitParam &param) { InitModule(param); }

  ~CModChassis() final { UnregisterModule_(); }

  ERpStatus InitModule(SModInitParam &param) final;

private:

  /**
   * @brief Chassis Wheelset Component Class
   */
  class CComWheelset final : public CComponent {
  public:

    enum {LF = 0, RF = 1, LB = 2, RB = 3};

    struct SWheelsetInfo {
      float_t speed_LF = 0.0f;    ///< Chassis Speed LF (Unit: rpm)
      float_t speed_RF = 0.0f;    ///< Chassis Speed RF (Unit: rpm)
      float_t speed_LB = 0.0f;    ///< Chassis Speed LB (Unit: rpm)
      float_t speed_RB = 0.0f;    ///< Chassis Speed RB (Unit: rpm)
    } wheelsetInfo;

    struct SWheelsetCommand {
      float_t speed_X = 0.0f;    ///< Chassis Speed X (Range: -100% ~ 100%)
      float_t speed_Y = 0.0f;    ///< Chassis Speed Y (Range: -100% ~ 100%)
      float_t speed_W = 0.0f;    ///< Chassis Speed W (Range: -100% ~ 100%)
    } wheelsetCmd;

    CMemsInstance *mems = nullptr;

    CMtrInstance *motor[4] = {nullptr };

    CPidController pidYawCtrl;

    CPidController pidSpdCtrl;

    std::array<int16_t, 4> mtrOutputBuffer = {0};

    ERpStatus InitComponent(SModInitParam &param) final;

    ERpStatus UpdateComponent() final;

    ERpStatus _UpdateOutput(float_t speed_X, float_t speed_Y, float_t speed_W);

  } comWheelset_;

  CCanInterface::CCanTxNode chassisMtrCanTxNode_;

  void UpdateHandler_() final;

  void HeartbeatHandler_() final;

  ERpStatus CreateModuleTask_() final;

  static void StartChassisModuleTask(void *arg);

  ERpStatus RestrictChassisCommand_();

};

} // namespace robotpilots

#endif // RP_MOD_CHASSIS_HPP
