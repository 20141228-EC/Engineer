#include "sys_controller_link.hpp"
namespace my_engineer{
EAppStatus CSystemControllerLink::Level3Move_(){

KeyBoard_move_(H_KEY_VALUE,0);                    ///< push H to start
Mouse_move_(POSIT_LEVEL3_X,POSIT_LEVEL3_Y,0,0);   ///< move mouse to level3
Mouse_move_(POSIT_LEVEL3_X,POSIT_LEVEL3_Y,1,0);   ///< click
Mouse_move_(POSIT_YES_X,POSIT_YES_Y,0,0);         ///< goto YES BUTTON
Mouse_move_(POSIT_YES_X,POSIT_YES_Y,1,0);         ///< click 

return APP_OK;

}
}