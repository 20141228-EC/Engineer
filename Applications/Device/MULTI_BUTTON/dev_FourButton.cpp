/*
 * @Description: 
 * @Author: Sassinak
 * @version: 
 * @Date: 2025-07-14 17:51:47
 * @LastEditors: Sassinak
 * @LastEditTime: 2025-07-18 15:48:46
 */
/*
 * @Description: 
 * @Author: Sassinak
 * @version: 
 * @Date: 2025-07-14 17:51:47
 * @LastEditors: Sassinak
 * @LastEditTime: 2025-07-17 22:43:00
 */
#include "multi_button\dev_FourButton.hpp"

namespace my_engineer {

CDevFourButton *pDev_button_test = nullptr;

// 拨杆状态
bool CDevFourButton::isSwitchChassis = false;
bool CDevFourButton::isSwitchArmRollEnd = false;

// 夹爪按钮状态（单夹爪，PB9）
bool CDevFourButton::isGripperPressed = false;
bool CDevFourButton::isGripperClose = false;      // 长按闭合/双击张开
bool CDevFourButton::isGripperReGrip = false;     // 单击二次夹紧（脉冲信号）

CDevFourButton::singlebutton CDevFourButton::buttons_[static_cast<int>(EButtonID::BUTTON_MAX)] = {};

uint8_t CDevFourButton::ButtonGpioRead(uint8_t button_id){
  if (button_id >= static_cast<uint8_t>(EButtonID::BUTTON_MAX)) {
    return 0;
  }
  switch(button_id){
    case EButtonID::SWITCH_CHASSIS:
    case EButtonID::SWITCH_ARM_ROLL_END:
    case EButtonID::GRIPPER:
      return HAL_GPIO_ReadPin(buttons_[button_id].halGpioPort, buttons_[button_id].halGpioPin);
  }

  return 0;
}

void CDevFourButton::ButtonPressDownCallback(void *btn) {
  Button* button = static_cast<Button *>(btn);
  uint8_t button_id = button->button_id;

  switch(button_id){
    case EButtonID::GRIPPER:
      isGripperPressed = true;
      break;
    default:
      break;
  }
}

void CDevFourButton::ButtonPressUpCallback(void *btn) {
  Button* button = static_cast<Button *>(btn);
  uint8_t button_id = button->button_id;

  switch(button_id){
    case EButtonID::GRIPPER:
      isGripperPressed = false;
      break;
    default:
      break;
  }
}

void CDevFourButton::ButtonLongPressCallback(void *btn) {
  Button* button = static_cast<Button *>(btn);
  uint8_t button_id = button->button_id;
  // 长按夹爪闭合
  switch(button_id){
    case EButtonID::GRIPPER:
      isGripperClose = true;
      break;
    default:
      break;
  }
}

void CDevFourButton::ButtonDoubleClickCallback(void *btn) {
  Button* button = static_cast<Button *>(btn);
  uint8_t button_id = button->button_id;
  // 双击夹爪张开
  switch(button_id){
    case EButtonID::GRIPPER:
      isGripperClose = false;
      break;
    default:
      break;
  }
}

void CDevFourButton::ButtonSingleClickCallback(void *btn) {
  Button* button = static_cast<Button *>(btn);
  uint8_t button_id = button->button_id;
  // 单击触发二次夹紧（仅在已夹持闭合状态下有效）
  switch(button_id){
    case EButtonID::GRIPPER:
      if (isGripperClose) {
        isGripperReGrip = true;
      }
      break;
    default:
      break;
  }
}


/**
 * @brief 初始化设备
 * 拨杆使用直接GPIO电平检测，夹爪按钮使用按钮库事件驱动
 */
EAppStatus CDevFourButton::InitDevice(const SDevInitParam_Base *pStructInitParam) {

  // 检查参数是否正确
  if (pStructInitParam == nullptr) return APP_ERROR;
  if (pStructInitParam->deviceID == EDeviceID::DEV_NULL) return APP_ERROR;

  // 类型转换
  auto initParam = static_cast<const SDevInitParam_FourButton *>(pStructInitParam);
  deviceID = initParam->deviceID;

  // 初始化每个按键配置
  for (int i = 0; i < static_cast<int>(EButtonID::BUTTON_MAX); ++i) {
    buttons_[i].buttonID = initParam->buttons_[i].buttonID;
    buttons_[i] = initParam->buttons_[i];

    // 只对夹爪按钮使用按钮库（拨杆和保留按钮使用直接GPIO检测）
    if (buttons_[i].buttonID == EButtonID::GRIPPER) {
      button_init(&buttons_[i].User_button, ButtonGpioRead, buttons_[i].activeLevel, static_cast<uint8_t>(buttons_[i].buttonID));
      button_attach(&buttons_[i].User_button, PressEvent::PRESS_DOWN, ButtonPressDownCallback);
      button_attach(&buttons_[i].User_button, PressEvent::PRESS_UP, ButtonPressUpCallback);
      button_attach(&buttons_[i].User_button, PressEvent::LONG_PRESS_HOLD, ButtonLongPressCallback);
      button_attach(&buttons_[i].User_button, PressEvent::DOUBLE_CLICK, ButtonDoubleClickCallback);
      button_attach(&buttons_[i].User_button, PressEvent::SINGLE_CLICK, ButtonSingleClickCallback);
      button_start(&buttons_[i].User_button);
    }
  }
  RegisterDevice_();

  deviceStatus = APP_OK;
  pDev_button_test = this;
  return APP_OK;
}

void CDevFourButton::UpdateHandler_() {
  /** 拨杆状态：直接读取GPIO电平
   *  3脚3档摇臂开关：0口接3.3V，拨向与引脚连通反向
   *  PE13 高电平 = 底盘模式
   *  PE9 高电平 = 臂Roll末端模式
   */
  isSwitchChassis = (HAL_GPIO_ReadPin(
      buttons_[static_cast<int>(EButtonID::SWITCH_CHASSIS)].halGpioPort,
      buttons_[static_cast<int>(EButtonID::SWITCH_CHASSIS)].halGpioPin) == GPIO_PIN_SET);

  isSwitchArmRollEnd = (HAL_GPIO_ReadPin(
      buttons_[static_cast<int>(EButtonID::SWITCH_ARM_ROLL_END)].halGpioPort,
      buttons_[static_cast<int>(EButtonID::SWITCH_ARM_ROLL_END)].halGpioPin) == GPIO_PIN_SET);

  // 夹爪按钮：调用按钮库扫描（事件驱动：长按闭合/双击张开）
  button_ticks();
}

}
