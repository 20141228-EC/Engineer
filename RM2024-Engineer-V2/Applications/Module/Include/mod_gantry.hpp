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

#ifndef RP_MOD_GANTRY_HPP
#define RP_MOD_GANTRY_HPP

#include "mod_common.hpp"

namespace robotpilots {

/**
 * @brief Gantry Module Class
 */
class CModGantry final : public CModInstance {
public:

  struct SModGantryInitParam : public SModInitParam {
    GPIO_TypeDef *pumpGpioPort_L = nullptr;
    uint16_t pumpGpioPin_L = 0;
    GPIO_TypeDef *pumpGpioPort_C = nullptr;
    uint16_t pumpGpioPin_C = 0;
    GPIO_TypeDef *pumpGpioPort_R = nullptr;
    uint16_t pumpGpioPin_R = 0;
    uint32_t gantryMtrCanStdID = 0x000;
    EInterfaceID gantryMtrCanInfID = EInterfaceID::INF_NULL;
    uint32_t gantryLiftMtrCanStdID = 0x000;
    EInterfaceID gantryLiftMtrCanInfID = EInterfaceID::INF_NULL;
    uint32_t gantryJointMtrCanStdID = 0x000;
    EInterfaceID gantryJointMtrCanInfID = EInterfaceID::INF_NULL;
    EDeviceID liftMotorID_L = EDeviceID::DEV_NULL;
    EDeviceID liftMotorID_R = EDeviceID::DEV_NULL;
    CPidController::SPidControllerInitParam liftPosPidParam;
    CPidController::SPidControllerInitParam liftSpdPidParam;
    EDeviceID stretchMotorID = EDeviceID::DEV_NULL;
    CPidController::SPidControllerInitParam stretchPosPidParam;
    CPidController::SPidControllerInitParam stretchSpdPidParam;
    EDeviceID traverseMotorID = EDeviceID::DEV_NULL;
    CPidController::SPidControllerInitParam traversePosPidParam;
    CPidController::SPidControllerInitParam traverseSpdPidParam;
    EDeviceID jointMotorID_Yaw = EDeviceID::DEV_NULL;
    EDeviceID jointMotorID_Roll = EDeviceID::DEV_NULL;
    CPidController::SPidControllerInitParam jointPosPidParam;
    CPidController::SPidControllerInitParam jointSpdPidParam;
    EDeviceID endMotorID_L = EDeviceID::DEV_NULL;
    EDeviceID endMotorID_R = EDeviceID::DEV_NULL;
    CPidController::SPidControllerInitParam endPosPidParam;
    CPidController::SPidControllerInitParam endSpdPidParam;
  };

  struct SGantryInfo {
    EFuncStatus isModuleAvailable  = false;        ///< Is Gantry Module Available
    bool isPumpOn_L                = false;        ///< Is Left Pump On
    bool isPumpOn_C                = false;        ///< Is Center Pump On
    bool isPumpOn_R                = false;        ///< Is Right Pump On
    float_t posit_Lift             = 0.0f;         ///< Gantry Lift Position (Range: 0mm ~ 660mm)
    float_t posit_Stretch          = 0.0f;         ///< Gantry Stretch Position (Range: 0mm ~ 410mm)
    float_t posit_Traverse         = 0.0f;         ///< Gantry Traverse Position (Range: 0mm ~ 190mm)
    float_t angle_Joint_Yaw        = 0.0f;         ///< Gantry Joint Yaw Angle (Range: -90° ~ 90°)
    float_t angle_Joint_Roll       = 0.0f;         ///< Gantry Joint Roll Angle (Range: -180° ~ 150°)
    float_t angle_End_Pitch        = 0.0f;         ///< Gantry End Pitch Angle (Range: -135° ~ 80°)
    float_t angle_End_Roll         = 0.0f;         ///< Gantry End Roll Angle (Unit: degree)
    bool isPositArrived_Lift       = false;        ///< Is Gantry Lift Position Arrived
    bool isPositArrived_Stretch    = false;        ///< Is Gantry Stretch Position Arrived
    bool isPositArrived_Traverse   = false;        ///< Is Gantry Traverse Position Arrived
    bool isPositArrived_Joint_Yaw  = false;        ///< Is Gantry Joint Yaw Angle Arrived
    bool isPositArrived_Joint_Roll = false;        ///< Is Gantry Joint Roll Angle Arrived
    bool isPositArrived_End_Pitch  = false;        ///< Is Gantry End Pitch Angle Arrived
    bool isPositArrived_End_Roll   = false;        ///< Is Gantry End Roll Angle Arrived
  } gantryInfo;

  struct SGantryCmd {
    bool isAutoCtrl             = false;       ///< Is Automatic Control
    bool setPumpOn_L            = false;       ///< Set Left Pump On
    bool setPumpOn_C            = false;       ///< Set Center Pump On
    bool setPumpOn_R            = false;       ///< Set Right Pump On
    float_t setPosit_Lift       = 0.0f;        ///< Gantry Lift Position (Range: 0mm ~ 660mm)
    float_t setPosit_Stretch    = 0.0f;        ///< Gantry Stretch Position (Range: 0mm ~ 410mm)
    float_t setPosit_Traverse   = 0.0f;        ///< Gantry Traverse Position (Range: 0mm ~ 190mm)
    float_t setAngle_Joint_Yaw  = -90.0f;      ///< Gantry Joint Yaw Angle (Range: -90° ~ 90°)
    float_t setAngle_Joint_Roll = 0.0f;        ///< Gantry Joint Roll Angle (Range: -180° ~ 150°)
    float_t setAngle_End_Pitch  = 0.0f;        ///< Gantry End Pitch Angle (Range: -135° ~ 80°)
    float_t setAngle_End_Roll   = 0.0f;        ///< Gantry End Roll Angle (Unit: degree)
  } gantryCmd;

  CModGantry() = default;

  explicit CModGantry(SModInitParam &param) { InitModule(param); }

  ~CModGantry() final { UnregisterModule_(); }

  ERpStatus InitModule(SModInitParam &param) final;

private:

  class CComPump final : public CComponent {
  public:

    enum { L = 0, C = 1, R = 2};

    struct SPumpInfo {
      bool isPumpOn_L = false;
      bool isPumpOn_C = false;
      bool isPumpOn_R = false;
    } pumpInfo;

    struct SPumpCmd {
      bool setPumpOn_L = false;
      bool setPumpOn_C = false;
      bool setPumpOn_R = false;
    } pumpCmd;

    std::array<GPIO_TypeDef *, 3> pumpGpioPort = {nullptr};

    std::array<uint16_t, 3> pumpGpioPin = {0};

    ERpStatus InitComponent(SModInitParam &param) final;

    ERpStatus UpdateComponent() final;

  } comPump_;

  /**
   * @brief Gantry Lift Component Class
   */
  class CComLift final : public CComponent {
  public:

    enum { L = 0, R = 1 };

    const int32_t rangeLimit = 610000;

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

  /**
   * @brief Gantry Traverse Component Class
   */
  class CComTraverse final : public CComponent {
  public:

    const int32_t rangeLimit =  1130000;

    struct STraverseInfo{
      int32_t posit = 0;
      bool isPositArrived = false;
    } traverseInfo;

    struct STraverseCmd{
      int32_t setPosit = 0;
    } traverseCmd;

    std::array<CMtrInstance *, 2> motor = {nullptr};

    CPidController pidPosCtrl;

    CPidController pidSpdCtrl;

    std::array<int16_t, 1> mtrOutputBuffer = {0};

    ERpStatus InitComponent(SModInitParam &param) final;

    ERpStatus UpdateComponent() final;

    static int32_t PhyPositToMtrPosit(float_t phyPosit);

    static float_t MtrPositToPhyPosit(int32_t mtrPosit);

    ERpStatus _UpdateOutput(float_t posit);

  } comTraverse_;

  /**
   * @brief Gantry Stretch Component Class
   */
  class CComStretch final : public CComponent {
  public:

    const int32_t rangeLimit = 435000;

    struct SStretchInfo{
      int32_t posit = 0;
      bool isPositArrived = false;
    } stretchInfo;

    struct SStretchCmd{
      int32_t setPosit = 0;
    } stretchCmd;

    std::array<CMtrInstance *, 1> motor = {nullptr};

    CPidController pidPosCtrl;

    CPidController pidSpdCtrl;

    std::array<int16_t, 1> mtrOutputBuffer = {0};

    ERpStatus InitComponent(SModInitParam &param) final;

    ERpStatus UpdateComponent() final;

    static int32_t PhyPositToMtrPosit(float_t phyPosit);

    static float_t MtrPositToPhyPosit(int32_t mtrPosit);

    ERpStatus _UpdateOutput(float_t posit);

  } comStretch_;

  /**
   * @brief Gantry Joint Component Class
   */
  class CComJoint final : public CComponent {
  public:

    enum { YAW = 0, ROLL = 1};

    const int32_t rangeLimit_YAW = 4096, rangeLimit_ROLL = 7509;

    struct SJointInfo {
      int32_t posit_Yaw = 0;
      int32_t posit_Roll = 0;
      bool isPositArrived_Yaw = false;
      bool isPositArrived_Roll = false;
    } jointInfo;

    struct SJointCmd {
      int32_t setPosit_Yaw = 0;
      int32_t setPosit_Roll = 0;
    } jointCmd;

    std::array<CMtrInstance *, 2> motor = {nullptr};

    CPidController pidPosCtrl;

    CPidController pidSpdCtrl;

    std::array<int16_t, 2> mtrOutputBuffer = {0};

    ERpStatus InitComponent(SModInitParam &param) final;

    ERpStatus UpdateComponent() final;

    static int32_t PhyPositToMtrPosit_Yaw(float_t phyPosit);

    static float_t MtrPositToPhyPosit_Yaw(int32_t mtrPosit);

    static int32_t PhyPositToMtrPosit_Roll(float_t phyPosit);

    static float_t MtrPositToPhyPosit_Roll(int32_t mtrPosit);

    ERpStatus _UpdateOutput(float_t posit_Yaw, float_t posit_Roll);

  } comJoint_;

  /**
   * @brief Gantry End Component Class
   */
  class CComEnd final : public CComponent {
  public:

    enum { L = 0, R = 1};

    const int32_t rangeLimit = 256000;

    struct SEndInfo {
      int32_t posit_Pitch = 0;
      int32_t posit_Roll = 0;
      bool isPositArrived_Pitch = false;
      bool isPositArrived_Roll = false;
    } endInfo;

    struct SEndCmd {
      int32_t setPosit_Pitch = 0;
      int32_t setPosit_Roll = 0;
    } endCmd;

    std::array<CMtrInstance *, 2> motor = {nullptr};

    CPidController pidPosCtrl;

    CPidController pidSpdCtrl;

    std::array<int16_t, 2> mtrOutputBuffer = {0};

    ERpStatus InitComponent(SModInitParam &param) final;

    ERpStatus UpdateComponent() final;

    static int32_t PhyPositToMtrPosit_Pitch(float_t phyPosit);

    static float_t MtrPositToPhyPosit_Pitch(int32_t mtrPosit);

    static int32_t PhyPositToMtrPosit_Roll(float_t phyPosit);

    static float_t MtrPositToPhyPosit_Roll(int32_t mtrPosit);

    ERpStatus _UpdateOutput(float_t posit_Pitch, float_t posit_Roll);

  } comEnd_;

  CCanInterface::CCanTxNode gantryMtrCanTxNode_, gantryLiftMtrCanTxNode_, gantryJointMtrCanTxNode_;

  void UpdateHandler_() final;

  void HeartbeatHandler_() final;

  ERpStatus CreateModuleTask_() final;

  static void StartGantryModuleTask(void *arg);

  ERpStatus RestrictGantryCommand_();

};

} // namespace robotpilots

#endif // RP_MOD_GANTRY_HPP
