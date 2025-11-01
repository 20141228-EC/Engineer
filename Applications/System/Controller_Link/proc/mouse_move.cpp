/*
 * @Description: 
 * @Author: Sassinak
 * @version: 
 * @Date: 2025-07-18 16:34:50
 * @LastEditors: Sassinak
 * @LastEditTime: 2025-07-18 18:05:53
 */
#include "sys_controller_link.hpp"
namespace my_engineer{

  EAppStatus CSystemControllerLink::Mouse_move_(uint16_t pos_x, uint16_t pos_y, uint8_t mouse_left, uint8_t mouse_right) {
    if (systemStatus != APP_OK) return APP_ERROR;
    /*move posit*/
    pcontrollerLink_->choseLevelData_info_pkg.Key_value1 = 0;
    pcontrollerLink_->choseLevelData_info_pkg.Key_value2 = 0;
    pcontrollerLink_->choseLevelData_info_pkg.x_position = pos_x;
    pcontrollerLink_->choseLevelData_info_pkg.y_position = pos_y;
    pcontrollerLink_->choseLevelData_info_pkg.mouse_left = 0;
    pcontrollerLink_->choseLevelData_info_pkg.mouse_right = 0;
    pcontrollerLink_->SendPackage(CDevControllerLink::ID_CHOSELEVEL_DATA,pcontrollerLink_->choseLevelData_info_pkg.header);
    /*click*/
    pcontrollerLink_->choseLevelData_info_pkg.Key_value1 = 0;
    pcontrollerLink_->choseLevelData_info_pkg.Key_value2 = 0;
    pcontrollerLink_->choseLevelData_info_pkg.x_position = pos_x;
    pcontrollerLink_->choseLevelData_info_pkg.y_position = pos_y;
    pcontrollerLink_->choseLevelData_info_pkg.mouse_left = mouse_left;
    pcontrollerLink_->choseLevelData_info_pkg.mouse_right = mouse_right;
    pcontrollerLink_->SendPackage(CDevControllerLink::ID_CHOSELEVEL_DATA,pcontrollerLink_->choseLevelData_info_pkg.header);
    /*stop click*/
    pcontrollerLink_->choseLevelData_info_pkg.Key_value1 = 0;
    pcontrollerLink_->choseLevelData_info_pkg.Key_value2 = 0;
    pcontrollerLink_->choseLevelData_info_pkg.x_position = pos_x;
    pcontrollerLink_->choseLevelData_info_pkg.y_position = pos_y;
    pcontrollerLink_->choseLevelData_info_pkg.mouse_left = 0;
    pcontrollerLink_->choseLevelData_info_pkg.mouse_right = 0;
    pcontrollerLink_->SendPackage(CDevControllerLink::ID_CHOSELEVEL_DATA,pcontrollerLink_->choseLevelData_info_pkg.header);

    return APP_OK;
  }
}

