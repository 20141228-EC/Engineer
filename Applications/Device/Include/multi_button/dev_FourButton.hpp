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
  enum  EButtonID {
    BUTTON_NULL = -1,
    RESET_BUTTON = 0,
    LEVEL_4_BUTTON,
    LEVEL_3_BUTTON,
    SELF_BUTTON,
    BUTTON_MAX,
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

  EAppStatus InitDevice(const SDevInitParam_Base *pStructInitParam) override;

  static bool isReset;
  static bool isLevel4;
  static bool isLevel3;
  static bool isSelf; ///< 是否启用对应按键
};

}

#endif