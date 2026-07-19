/*
 * @Description: 
 * @Author: Sassinak
 * @version: 
 * @Date: 2025-07-14 16:37:23
 * @LastEditors: ciallo
 * @LastEditTime: 2026-06-03 22:42:03
 */
#ifndef DEV_BUTTON_HPP
#define DEV_BUTTON_HPP

#include "dev_multi_button.hpp"

namespace my_engineer {

class CDevButton : public CDevMultiButton {

public:
  enum EButtonID : uint8_t {
    BUTTON_NULL = 255,          // 空按钮标记
    LEVEL_1 = 0,                // 一级难度   
    LEVEL_2 = 1,                // 二级难度 
    LEVEL_3 = 2,                // 三级难度 
    RESET = 3,                  // 复位自定义控制器
    BUTTON_MAX = 4              // 数组大小
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
  struct SDevInitParam_Button : public SDevInitParam_MultiButton {
    singlebutton buttons_[static_cast<int>(EButtonID::BUTTON_MAX)]; ///< 按键配置数组

  };

  CDevButton() { deviceType = EDevType::DEV_MULTI_BUTTON; }
  
  static uint8_t ButtonGpioRead(uint8_t button_id);
  static void ButtonPressDownCallback(void *btn);
  static void ButtonPressUpCallback(void *btn);
  static void ButtonLongPressCallback(void *btn);
  static void ButtonDoubleClickCallback(void *btn);  // 双击回调
  static void ButtonSingleClickCallback(void *btn); // 单击回调（二次夹紧）

  EAppStatus InitDevice(const SDevInitParam_Base *pStructInitParam) override;

  // 按键状态
  static bool islevel_1;      
  static bool islevel_2;
  static bool islevel_3;
  static bool isControllerReset;
  static bool isRobotReset;

};

}

#endif
