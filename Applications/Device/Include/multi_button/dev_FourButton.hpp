/*
 * @Description: 
 * @Author: Sassinak
 * @version: 
 * @Date: 2025-07-14 16:37:23
 * @LastEditors: Sassinak
 * @LastEditTime: 2025-07-17 22:42:03
 */
#ifndef DEV_FOUR_BUTTON_HPP
#define DEV_FOUR_BUTTON_HPP

#include "dev_multi_button.hpp"

namespace my_engineer {

class CDevFourButton : public CDevMultiButton {

public:
  enum EButtonID : uint8_t {
    BUTTON_NULL = 255,          // 空按钮标记（改为255避免负数转换）
    SWITCH_CHASSIS = 0,         // PE13 - 拨杆右档（底盘模式）
    SWITCH_ARM_ROLL_END = 1,    // PE9 - 拨杆左档（臂Roll末端模式）
    GRIPPER_LEFT = 2,           // PB8 - 左手夹爪按钮
    GRIPPER_RIGHT = 3,          // PB9 - 右手夹爪按钮
    BUTTON_MAX = 4              // 数组大小（连续索引0-3）
  };
private:
  typedef struct singlebutton{
    EButtonID buttonID = EButtonID::BUTTON_NULL; ///< 按键ID
    uint8_t activeLevel = 0; ///< 按键激活电平
    GPIO_TypeDef *halGpioPort = nullptr; ///< 按键端口
		uint16_t halGpioPin = 0; ///< 按键引脚
    Button User_button; ///< 按键结构体
  } singlebutton;

  void UpdateHandler_() override;
  static singlebutton buttons_[static_cast<int>(EButtonID::BUTTON_MAX)];

public:
  struct SDevInitParam_FourButton : public SDevInitParam_MultiButton {
    singlebutton buttons_[static_cast<int>(EButtonID::BUTTON_MAX)]; ///< 按键配置数组

  };

  CDevFourButton() { deviceType = EDevType::DEV_MULTI_BUTTON; }
  
  static uint8_t ButtonGpioRead(uint8_t button_id);
  static void ButtonPressDownCallback(void *btn);
  static void ButtonPressUpCallback(void *btn);
  static void ButtonLongPressCallback(void *btn);
  static void ButtonDoubleClickCallback(void *btn);  // 双击回调

  EAppStatus InitDevice(const SDevInitParam_Base *pStructInitParam) override;

  // 拨杆状态（直接电平检测，不使用按钮库）
  static bool isSwitchChassis;      // PE13 高电平 = 底盘模式
  static bool isSwitchArmRollEnd;   // PE9 高电平 = 臂Roll末端模式

  // 夹爪按钮状态（按钮库事件驱动）
  static bool isGripperLeft;        // 左手夹爪按下状态（临时）
  static bool isGripperRight;       // 右手夹爪按下状态（临时）
  static bool isGripperLeftClose;   // 左手夹爪闭合状态（持久，长按闭合/双击张开）
  static bool isGripperRightClose;  // 右手夹爪闭合状态（持久，长按闭合/双击张开）
};

}

#endif