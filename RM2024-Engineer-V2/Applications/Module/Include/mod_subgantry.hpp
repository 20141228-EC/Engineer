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

#ifndef RP_MOD_SUBGANTRY_HPP
#define RP_MOD_SUBGANTRY_HPP

#include "mod_common.hpp"

namespace robotpilots {

class CModSubGantry final : public CModInstance {
public:

  struct SModSubGantryInitParam : public SModInitParam {
    uint32_t subGantryMtrCanStdID = 0x000;
    EInterfaceID subGantryMtrCanInfID = EInterfaceID::INF_NULL;
    EDeviceID liftMotorID_L = EDeviceID::DEV_NULL;
    EDeviceID liftMotorID_R = EDeviceID::DEV_NULL;
    CPidController::SPidControllerInitParam liftPosPidParam;
    CPidController::SPidControllerInitParam liftSpdPidParam;
    EDeviceID stretchMotorID_L = EDeviceID::DEV_NULL;
    EDeviceID stretchMotorID_R = EDeviceID::DEV_NULL;
    CPidController::SPidControllerInitParam stretchPosPidParam;
    CPidController::SPidControllerInitParam stretchSpdPidParam;
  };

  struct SSubGantryInfo {
    EFuncStatus isModuleAvailable = false;      ///< Is SubGantry Module Available
    float_t posit_Lift_L = 0.0f;                ///< Sub-Gantry Lift Posit Left (Range: 0mm ~ 295mm)
    float_t posit_Lift_R = 0.0f;                ///< Sub-Gantry Lift Posit Right (Range: 0mm ~ 295mm)
    float_t posit_Stretch_L = 0.0f;             ///< Sub-Gantry Stretch Posit Left (Range: 0mm ~ 360mm)
    float_t posit_Stretch_R = 0.0f;             ///< Sub-Gantry Stretch Posit Right (Range: 0mm ~ 360mm)
    bool isPositArrived_Lift_L = false;         ///< Is Sub-Gantry Lift Posit Left Arrived
    bool isPositArrived_Lift_R = false;         ///< Is Sub-Gantry Lift Posit Right Arrived
    bool isPositArrived_Stretch_L = false;      ///< Is Sub-Gantry Stretch Posit Left Arrived
    bool isPositArrived_Stretch_R = false;      ///< Is Sub-Gantry Stretch Posit Right Arrived
  } subGantryInfo;

  struct SSubGantryCmd {
    bool isAutoCtrl = false;
    float_t setPosit_Lift_L = 0.0f;             ///< Set Sub-Gantry Lift Posit Left (Range: 0mm ~ 295mm)
    float_t setPosit_Lift_R = 0.0f;             ///< Set Sub-Gantry Lift Posit Right (Range: 0mm ~ 295mm)
    float_t setPosit_Stretch_L = 0.0f;          ///< Set Sub-Gantry Stretch Posit Left (Range: 0mm ~ 360mm)
    float_t setPosit_Stretch_R = 0.0f;          ///< Set Sub-Gantry Stretch Posit Right (Range: 0mm ~ 360mm)
  } subGantryCmd;

  CModSubGantry() = default;

  explicit CModSubGantry(SModSubGantryInitParam &param) { InitModule(param); }

  ~CModSubGantry() final { UnregisterModule_(); }

  ERpStatus InitModule(SModInitParam &param) final;

private:

  /**
   * @brief
   */
  class CComLift final : public CComponent {
  public:

    enum { L = 0, R = 1 };

    const int32_t rangeLimit = 1070000;

    struct SLiftInfo {
      int32_t posit_L = 0;
      int32_t posit_R = 0;
      bool isPositArrived_L = false;
      bool isPositArrived_R = false;
    } liftInfo;

    struct SLiftCmd {
      int32_t setPosit_L = 0;
      int32_t setPosit_R = 0;
    } liftCmd;

    std::array<CMtrInstance *, 2> motor = {nullptr};

    CPidController pidPosCtrl;

    CPidController pidSpdCtrl;

    std::array<int16_t, 2> mtrOutputBuffer = {0};

    ERpStatus InitComponent(SModInitParam &param) final;

    ERpStatus UpdateComponent() final;

    static int32_t PhyPositToMtrPosit(float_t phyPosit);

    static float_t MtrPositToPhyPosit(int32_t mtrPosit);

    ERpStatus _UpdateOutput(float_t posit_L, float_t posit_R);

  } comLift_;

  /**
   * @brief
   */
  class CComStretch final : public CComponent {
  public:

    enum { L = 0, R = 1 };

    const int32_t rangeLimit = 1314000;

    struct SStretchInfo {
      int32_t posit_L = 0;
      int32_t posit_R = 0;
      bool isPositArrived_L = false;
      bool isPositArrived_R = false;
    } stretchInfo;

    struct SStretchCmd {
      int32_t setPosit_L = 0;
      int32_t setPosit_R = 0;
    } stretchCmd;

    std::array<CMtrInstance *, 2> motor = {nullptr};

    CPidController pidPosCtrl;

    CPidController pidSpdCtrl;

    std::array<int16_t, 2> mtrOutputBuffer = {0};

    ERpStatus InitComponent(SModInitParam &param) final;

    ERpStatus UpdateComponent() final;

    static int32_t PhyPositToMtrPosit(float_t phyPosit);

    static float_t MtrPositToPhyPosit(int32_t mtrPosit);

    ERpStatus _UpdateOutput(float_t posit_L, float_t posit_R);

  } comStretch_;

  CCanInterface::CCanTxNode subGantryMtrCanTxNode_;

  void UpdateHandler_() final;

  void HeartbeatHandler_() final;

  ERpStatus CreateModuleTask_() final;

  static void StartSubGantryModuleTask(void *arg);

  ERpStatus RestrictGantryCommand_();

};

} // namespace robotpilots

#endif // RP_MOD_SUBGANTRY_HPP
