/*
 * @Description: 
 * @Author: Sassinak
 * @version: 
 * @Date: 2025-07-14 16:03:18
 * @LastEditors: Sassinak
 * @LastEditTime: 2025-07-17 22:37:14
 */

#ifndef DEV_MULTI_BUTTON_HPP
#define DEV_MULTI_BUTTON_HPP

#include <stdint.h>
#include <string.h>
#include "dev_common.hpp"

//According to your need to modify the constants.
#define TICKS_INTERVAL    2	//ms
#define DEBOUNCE_TICKS    3	//MAX 7 (0 ~ 7)
#define SHORT_TICKS       (300 /TICKS_INTERVAL)
#define LONG_TICKS        (1000 /TICKS_INTERVAL)

namespace my_engineer {

class CDevMultiButton : public CDevBase {

protected:

  struct SDevInitParam_MultiButton : public SDevInitParam_Base {

  };

  CDevMultiButton() { deviceType = EDevType::DEV_MULTI_BUTTON; }

  EAppStatus InitDevice(const SDevInitParam_Base *pStructInitParam) override {return APP_OK;}

public:

  typedef void (*BtnCallback)(void*);

  enum PressEvent{
    PRESS_DOWN = 0,
    PRESS_UP,
    PRESS_REPEAT,
    SINGLE_CLICK,
    DOUBLE_CLICK,
    LONG_PRESS_START,
    LONG_PRESS_HOLD,
    number_of_event,
    NONE_PRESS
  };

  typedef struct Button {
  uint16_t ticks;
  uint8_t  repeat : 4;
  uint8_t  event : 4;
  uint8_t  state : 3;
  uint8_t  debounce_cnt : 3;
  uint8_t  active_level : 1;
  uint8_t  button_level : 1;
  uint8_t  button_id;
  uint8_t  (*hal_button_Level)(uint8_t button_id_);
  BtnCallback  cb[number_of_event];
  struct Button* next;
  } Button;

  void UpdateHandler_() override {}

	void HeartbeatHandler_() override {}

  void button_init(Button* handle, uint8_t(*pin_level)(uint8_t), uint8_t active_level, uint8_t button_id);
  void button_attach(Button* handle, PressEvent event, BtnCallback cb);
  PressEvent get_button_event(Button* handle);
  int  button_start(Button* handle);
  void button_stop(Button* handle);
  void button_ticks(void);
};
}

#endif