/*
 * @Description: 
 * @Author: Sassinak
 * @version: 
 * @Date: 2025-07-14 17:51:47
 * @LastEditors: ciallo
 * @LastEditTime: 2026-06-18 15:48:46
 */
#include "multi_button\dev_Button.hpp"
  
namespace my_engineer {

CDevButton *pDev_button_test = nullptr;

// 初始化按键状态
bool CDevButton::islevel_1 = false;
bool CDevButton::islevel_2 = false;
bool CDevButton::islevel_3 = false;
bool CDevButton::isControllerReset = false;
bool CDevButton::isRobotReset = false;


CDevButton::singlebutton CDevButton::buttons_[static_cast<int>(EButtonID::BUTTON_MAX)] = {};

uint8_t CDevButton::ButtonGpioRead(uint8_t button_id){
  if (button_id >= static_cast<uint8_t>(EButtonID::BUTTON_MAX)) {
    return 0;
  }
  switch(button_id){
    case EButtonID::LEVEL_1:
    case EButtonID::LEVEL_2:
    case EButtonID::LEVEL_3:
    case EButtonID::RESET:
      return HAL_GPIO_ReadPin(buttons_[button_id].halGpioPort, buttons_[button_id].halGpioPin);
  }

  return 0;
}

// 按键事件回调函数,设置按键触发之后的响应
void CDevButton::ButtonPressDownCallback(void *btn) {
  Button* button = static_cast<Button *>(btn);
  uint8_t button_id = button->button_id;

  switch(button_id){
    case EButtonID::LEVEL_1:
      islevel_1 = true;
      break;
    case EButtonID::LEVEL_2:
      islevel_2 = true;
      break;
    case EButtonID::LEVEL_3:
      islevel_3 = true;
      break;
    default:
      break;
  }
}

void CDevButton::ButtonPressUpCallback(void *btn) {
  Button* button = static_cast<Button *>(btn);
  uint8_t button_id = button->button_id;

  switch(button_id){
    case EButtonID::LEVEL_1:
      islevel_1 = false;
      break;
    case EButtonID::LEVEL_2:
      islevel_2 = false;
      break;
    case EButtonID::LEVEL_3:
      islevel_3 = false;
      break;
    default:
      break;
  }
}

// 单击触发自定义控制器的重启
void CDevButton::ButtonSingleClickCallback(void *btn) {
  Button* button = static_cast<Button *>(btn);
  uint8_t button_id = button->button_id;
  switch(button_id){
    case EButtonID::RESET:
      isControllerReset = true;
      break;
    default:
      break;
  }
}

// 长按触发机器人的重启
void CDevButton::ButtonLongPressCallback(void *btn) {
  Button* button = static_cast<Button *>(btn);
  uint8_t button_id = button->button_id;
  switch(button_id){
    case EButtonID::RESET:
      isRobotReset = true;
      break;
    default:
      break;
  }
}

/**
 * @brief 初始化设备
 * 拨杆使用直接GPIO电平检测，夹爪按钮使用按钮库事件驱动
 */
EAppStatus CDevButton::InitDevice(const SDevInitParam_Base *pStructInitParam) {

  // 检查参数是否正确
  if (pStructInitParam == nullptr) return APP_ERROR;
  if (pStructInitParam->deviceID == EDeviceID::DEV_NULL) return APP_ERROR;

  // 类型转换
  auto initParam = static_cast<const SDevInitParam_Button *>(pStructInitParam);
  deviceID = initParam->deviceID;

  // 初始化每个按键配置
  for (int i = 0; i < static_cast<int>(EButtonID::BUTTON_MAX); ++i) {
    buttons_[i].buttonID = initParam->buttons_[i].buttonID;
    buttons_[i] = initParam->buttons_[i];

    // 按钮使用按钮库
    if (buttons_[i].buttonID == EButtonID::LEVEL_1 ||
        buttons_[i].buttonID == EButtonID::LEVEL_2 ||
        buttons_[i].buttonID == EButtonID::LEVEL_3 ) {
      button_init(&buttons_[i].User_button, ButtonGpioRead, buttons_[i].activeLevel, static_cast<uint8_t>(buttons_[i].buttonID));
      button_attach(&buttons_[i].User_button, PressEvent::PRESS_DOWN, ButtonPressDownCallback);
      // button_attach(&buttons_[i].User_button, PressEvent::PRESS_UP, ButtonPressUpCallback);
      // button_attach(&buttons_[i].User_button, PressEvent::LONG_PRESS_HOLD, ButtonLongPressCallback);
      // button_attach(&buttons_[i].User_button, PressEvent::DOUBLE_CLICK, ButtonDoubleClickCallback);
      //button_attach(&buttons_[i].User_button, PressEvent::SINGLE_CLICK, ButtonSingleClickCallback);
      button_start(&buttons_[i].User_button);
    }

    if(buttons_[i].buttonID == EButtonID::RESET){
      button_init(&buttons_[i].User_button, ButtonGpioRead, buttons_[i].activeLevel, static_cast<uint8_t>(buttons_[i].buttonID));
      button_attach(&buttons_[i].User_button, PressEvent::LONG_PRESS_START, ButtonLongPressCallback);
      button_attach(&buttons_[i].User_button, PressEvent::SINGLE_CLICK, ButtonSingleClickCallback);
      button_start(&buttons_[i].User_button);
    }
  }
  RegisterDevice_();

  deviceStatus = APP_OK;
  pDev_button_test = this;
  return APP_OK;
}

void CDevButton::UpdateHandler_() {
  /** 拨杆状态：直接读取GPIO电平
   *  3脚3档摇臂开关：0口接3.3V，拨向与引脚连通反向
   *  PE13 高电平 = 底盘模式
   *  PE9 高电平 = 臂Roll末端模式
   */
  // isSwitchChassis = (HAL_GPIO_ReadPin(
  //     buttons_[static_cast<int>(EButtonID::SWITCH_CHASSIS)].halGpioPort,
  //     buttons_[static_cast<int>(EButtonID::SWITCH_CHASSIS)].halGpioPin) == GPIO_PIN_SET);

  // isSwitchArmRollEnd = (HAL_GPIO_ReadPin(
  //     buttons_[static_cast<int>(EButtonID::SWITCH_ARM_ROLL_END)].halGpioPort,
  //     buttons_[static_cast<int>(EButtonID::SWITCH_ARM_ROLL_END)].halGpioPin) == GPIO_PIN_SET);

  // 夹爪按钮：调用按钮库扫描
  button_ticks();
}

}
