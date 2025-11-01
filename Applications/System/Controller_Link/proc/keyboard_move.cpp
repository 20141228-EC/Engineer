#include "sys_controller_link.hpp"

namespace my_engineer{
EAppStatus CSystemControllerLink::KeyBoard_move_(uint8_t key_value1, uint8_t key_value2 ){
  /*key push*/
  pcontrollerLink_->choseLevelData_info_pkg.Key_value1 = key_value1;
  pcontrollerLink_->choseLevelData_info_pkg.Key_value2 = key_value2;
  pcontrollerLink_->choseLevelData_info_pkg.x_position = 0;
  pcontrollerLink_->choseLevelData_info_pkg.y_position = 0;
  pcontrollerLink_->choseLevelData_info_pkg.mouse_left = 0;
  pcontrollerLink_->choseLevelData_info_pkg.mouse_right = 0;
  pcontrollerLink_->SendPackage(CDevControllerLink::ID_CHOSELEVEL_DATA,pcontrollerLink_->choseLevelData_info_pkg.header);
  /*relax key*/
  pcontrollerLink_->choseLevelData_info_pkg.Key_value1 = 0;
  pcontrollerLink_->choseLevelData_info_pkg.Key_value2 = 0;
  pcontrollerLink_->choseLevelData_info_pkg.x_position = 0;
  pcontrollerLink_->choseLevelData_info_pkg.y_position = 0;
  pcontrollerLink_->choseLevelData_info_pkg.mouse_left = 0;
  pcontrollerLink_->choseLevelData_info_pkg.mouse_right = 0;
  pcontrollerLink_->SendPackage(CDevControllerLink::ID_CHOSELEVEL_DATA,pcontrollerLink_->choseLevelData_info_pkg.header);

  return APP_OK;
}
}