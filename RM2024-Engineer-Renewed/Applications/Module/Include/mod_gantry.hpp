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
    GPIO_TypeDef *pumpGpioPort_1 = nullptr;
    uint16_t pumpGpioPin_1 = 0;
    GPIO_TypeDef *pumpGpioPort_2 = nullptr;
    uint16_t pumpGpioPin_2 = 0;
    uint32_t gantryMtrCanStdID = 0x000;
    EInterfaceID gantryMtrCanInfID = EInterfaceID::INF_NULL;
    uint32_t gantryTravMtrCanStdID = 0x000;
    EInterfaceID gantryTravMtrCanInfID = EInterfaceID::INF_NULL;
    uint32_t gantryManipMtrCanStdID = 0x000;
    EInterfaceID gantryManipMtrCanInfID = EInterfaceID::INF_NULL;
    uint32_t gantryFlipMtrCanStdID = 0x000;
    EInterfaceID gantryFlipMtrCanInfID = EInterfaceID::INF_NULL;
    EDeviceID liftMotorID_L = EDeviceID::DEV_NULL;
    EDeviceID liftMotorID_R = EDeviceID::DEV_NULL;
    CPidController::SPidControllerInitParam liftPosPidParam;
    CPidController::SPidControllerInitParam liftSpdPidParam;
    EDeviceID stretchMotorID_L = EDeviceID::DEV_NULL;
    EDeviceID stretchMotorID_R = EDeviceID::DEV_NULL;
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
    EDeviceID flipMotorID_F = EDeviceID::DEV_NULL;
    EDeviceID flipMotorID_B = EDeviceID::DEV_NULL;
    CPidController::SPidControllerInitParam flipSpdPidParam;
  };

  struct SGantryInfo {
    EFuncStatus isModuleAvailable  = false;        ///< Is Gantry Module Available
    bool isPumpOn                  = false;        ///< Is Pump On
    float_t posit_Lift             = 0.0f;         ///< Gantry Lift Position (Range: 0mm ~ 560mm)
    float_t posit_Stretch          = 0.0f;         ///< Gantry Stretch Position (Range: 0mm ~ 420mm)
    float_t posit_Traverse         = 0.0f;         ///< Gantry Traverse Position (Range: -90mm ~ 90mm)
    float_t angle_Joint_Yaw        = 0.0f;         ///< Gantry Joint Yaw Angle (Range: -90° ~ 180°)
    float_t angle_Joint_Roll       = 0.0f;         ///< Gantry Joint Roll Angle (Range: -90° ~ 200°)
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
    bool setPumpOn              = false;       ///< Set Pump On
    float_t setPosit_Lift       = 0.0f;        ///< Gantry Lift Position (Range: 0mm ~ 560mm)
    float_t setPosit_Stretch    = 0.0f;        ///< Gantry Stretch Position (Range: 0mm ~ 420mm)
    float_t setPosit_Traverse   = 0.0f;        ///< Gantry Traverse Position (Range: -90mm ~ 90mm)
    float_t setAngle_Joint_Yaw  = 180.0f;      ///< Gantry Joint Yaw Angle (Range: -90° ~ 180°)
    float_t setAngle_Joint_Roll = 180.0f;      ///< Gantry Joint Roll Angle (Range: -90° ~ 200°)
    float_t setAngle_End_Pitch  = 90.0f;       ///< Gantry End Pitch Angle (Range: -70° ~ 125°)
    float_t setAngle_End_Roll   = 0.0f;        ///< Gantry End Roll Angle (Unit: degree)
    float_t setSpeed_Flip_Y     = 0.0f;        ///< Gantry Flip Speed Y (Range: -100% ~ 100%)
    float_t setSpeed_Flip_W     = 0.0f;        ///< Gantry Flip Speed W (Range: -100% ~ 100%)
  } gantryCmd;

  CModGantry() = default;

  explicit CModGantry(SModInitParam &param) { InitModule(param); }

  ~CModGantry() final { UnregisterModule_(); }

  ERpStatus InitModule(SModInitParam &param) final;

private:

  /**
   * @brief Gantry Pump Component Class
   */
  class CComPump final : public CComponent {
  public:

    struct SPumpInfo {
      bool isPumpOn = false;
    } pumpInfo;

    struct SPumpCmd {
      bool setPumpOn = false;
    } pumpCmd;

    std::array<GPIO_TypeDef *, 2> pumpGpioPort = {nullptr};

    std::array<uint16_t, 2> pumpGpioPin = {0};

    ERpStatus InitComponent(SModInitParam &param) final;

    ERpStatus UpdateComponent() final;

  } comPump_;

  /**
   * @brief Gantry Lift Component Class
   */
  class CComLift final : public CComponent {
  public:

    enum { L = 0, R = 1 };

    const int32_t rangeLimit = 1000000;

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

    DataBuffer<float_t> liftPos, liftPosMeasure, liftSpd, liftSpdMeasure, output;

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

    const int32_t rangeLimit =  660000;

    struct STraverseInfo{
      int32_t posit = 0;
      bool isPositArrived = false;
    } traverseInfo;

    struct STraverseCmd{
      int32_t setPosit = 0;
    } traverseCmd;

    std::array<CMtrInstance *, 2> motor = { nullptr };

    CPidController pidPosCtrl;

    CPidController pidSpdCtrl;

    DataBuffer<float_t> traversePos, traversePosMeasure, traverseSpd, traverseSpdMeasure, output;

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

    enum { L = 0, R = 1 };

    const int32_t rangeLimit = 730000;

    struct SStretchInfo{
      int32_t posit = 0;
      bool isPositArrived = false;
    } stretchInfo;

    struct SStretchCmd{
      int32_t setPosit = 0;
    } stretchCmd;

    std::array<CMtrInstance *, 2> motor = {nullptr};

    CPidController pidPosCtrl;

    CPidController pidSpdCtrl;

    DataBuffer<float_t> stretchPos, stretchPosMeasure, stretchSpd, stretchSpdMeasure, output;

    std::array<int16_t, 2> mtrOutputBuffer = {0};

    ERpStatus InitComponent(SModInitParam &param) final;

    ERpStatus UpdateComponent() final;

    static int32_t PhyPositToMtrPosit(float_t phyPosit);

    static float_t MtrPositToPhyPosit(int32_t mtrPosit);

    ERpStatus _UpdateOutput(float_t posit);

  } comStretch_;

  /**
   * @brief Gantry Joint Yaw Component Class
   */
  class CComJointYaw final : public CComponent {
  public:

    const int32_t rangeLimit = 456000;

    struct SJointYawInfo {
      int32_t posit = 0;
      bool isPositArrived = false;
    } jointYawInfo;

    struct SJointYawCmd {
      int32_t setPosit = 0;
    } jointYawCmd;

    std::array<CMtrInstance *, 1> motor = {nullptr};

    CPidController pidPosCtrl;

    CPidController pidSpdCtrl;

    DataBuffer<float_t> jointYawPos, jointYawPosMeasure, jointYawSpd, jointYawSpdMeasure, output;

    std::array<int16_t, 1> mtrOutputBuffer = {0};

    ERpStatus InitComponent(SModInitParam &param) final;

    ERpStatus UpdateComponent() final;

    static int32_t PhyPositToMtrPosit(float_t phyPosit);

    static float_t MtrPositToPhyPosit(int32_t mtrPosit);

    ERpStatus _UpdateOutput(float_t posit);

  } comJointYaw_;

  /**
   * @brief Gantry Joint Roll Component Class
   */
  class CComJointRoll final : public CComponent {
  public:

    const int32_t rangeLimit = 653000;

    struct SJointRollInfo {
      int32_t posit = 0;
      bool isPositArrived = false;
    } jointRollInfo;

    struct SJointRollCmd {
      int32_t setPosit = 0;
    } jointRollCmd;

    std::array<CMtrInstance *, 1> motor = {nullptr};

    CPidController pidPosCtrl;

    CPidController pidSpdCtrl;

    DataBuffer<float_t> jointRollPos, jointRollPosMeasure, jointRollSpd, jointRollSpdMeasure, output;

    std::array<int16_t, 1> mtrOutputBuffer = {0};

    ERpStatus InitComponent(SModInitParam &param) final;

    ERpStatus UpdateComponent() final;

    static int32_t PhyPositToMtrPosit(float_t phyPosit);

    static float_t MtrPositToPhyPosit(int32_t mtrPosit);

    ERpStatus _UpdateOutput(float_t posit);

  } comJointRoll_;

  /**
   * @brief Gantry End Component Class
   */
  class CComEnd final : public CComponent {
  public:

    enum { L = 0, R = 1};

    const int32_t rangeLimit = 226000;

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

    DataBuffer<float_t> endPos, endPosMeasure, endSpd, endSpdMeasure, output;

    std::array<int16_t, 2> mtrOutputBuffer = {0};

    ERpStatus InitComponent(SModInitParam &param) final;

    ERpStatus UpdateComponent() final;

    static int32_t PhyPositToMtrPosit_Pitch(float_t phyPosit);

    static float_t MtrPositToPhyPosit_Pitch(int32_t mtrPosit);

    static int32_t PhyPositToMtrPosit_Roll(float_t phyPosit);

    static float_t MtrPositToPhyPosit_Roll(int32_t mtrPosit);

    ERpStatus _UpdateOutput(float_t posit_Pitch, float_t posit_Roll);

  } comEnd_;

  /**
   * @brief Gantry Flip Component Class
   */
  class CComFlip final : public CComponent {
  public:

    enum { F = 0, B = 1 };

    struct SFlipInfo {
      int32_t speed_Y = 0;
      int32_t speed_W = 0;
    } flipInfo;

    struct SFlipCmd {
      int32_t setSpeed_Y = 0;
      int32_t setSpeed_W = 0;
    } flipCmd;

    std::array<CMtrInstance *, 2> motor = {nullptr};

    CPidController pidSpdCtrl;

    DataBuffer<float_t> flipSpd, flipSpdMeasure, output;

    std::array<int16_t, 2> mtrOutputBuffer = {0};

    ERpStatus InitComponent(SModInitParam &param) final;

    ERpStatus UpdateComponent() final;

    ERpStatus _UpdateOutput(float_t speed_Y, float_t speed_W);

  } comFlip_;

  CCanInterface::CCanTxNode gantryMtrCanTxNode_, gantryManipMtrCanTxNode_, gantryTravMtrCanTxNode_, gantryFlipMtrCanTxNode_;

  void UpdateHandler_() final;

  void HeartbeatHandler_() final;

  ERpStatus CreateModuleTask_() final;

  static void StartGantryModuleTask(void *arg);

  ERpStatus RestrictGantryCommand_();

};

} // namespace robotpilots

#endif // RP_MOD_GANTRY_HPP
