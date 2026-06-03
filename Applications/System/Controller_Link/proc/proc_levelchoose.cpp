/******************************************************************************
 * @file         proc_levelchoose.cpp
 * @author       ciallo (1002046597@qq.com)
 * @brief        一键选择矿石难度
 * @version      V2.0
 * @date         2026-06-03
 *
 * @copyright    Copyright (c) 2026
 ******************************************************************************/
#include "sys_controller_link.hpp"

namespace my_engineer {

void CSystemControllerLink::StartLevelChoose_(uint8_t level) {
    if (level >= 4) return;
    levelTargetX_ = Level_Positions[level][0];
    levelTargetY_ = Level_Positions[level][1];
    levelStep_ = ELevelStep::KEY_PRESS;
}

void CSystemControllerLink::TickLevelChoose_() {
    if (levelStep_ == ELevelStep::IDLE) return;

    auto &pkg = pcontrollerLink_->choseLevelData_info_pkg;

    switch (levelStep_) {

    case ELevelStep::KEY_PRESS:
        pkg.Key_value1 = H_KEY_VALUE;
        pkg.Key_value2 = 0;
        pkg.x_position = 0;
        pkg.y_position = 0;
        pkg.mouse_left = 0;
        pkg.mouse_right = 0;
        pcontrollerLink_->SendPackage(CDevControllerLink::ID_CHOSELEVEL_DATA, pkg.header);
        levelStep_ = ELevelStep::KEY_RELEASE;
        break;

    case ELevelStep::KEY_RELEASE:
        pkg.Key_value1 = 0;
        pkg.Key_value2 = 0;
        pcontrollerLink_->SendPackage(CDevControllerLink::ID_CHOSELEVEL_DATA, pkg.header);
        levelStep_ = ELevelStep::MOVE_TO_LEVEL;
        break;

    case ELevelStep::MOVE_TO_LEVEL:
        pkg.x_position = levelTargetX_;
        pkg.y_position = levelTargetY_;
        pkg.mouse_left = 0;
        pkg.mouse_right = 0;
        pcontrollerLink_->SendPackage(CDevControllerLink::ID_CHOSELEVEL_DATA, pkg.header);
        levelStep_ = ELevelStep::CLICK_LEVEL;
        break;

    case ELevelStep::CLICK_LEVEL:
        pkg.mouse_left = 1;
        pcontrollerLink_->SendPackage(CDevControllerLink::ID_CHOSELEVEL_DATA, pkg.header);
        levelStep_ = ELevelStep::RELEASE_CLICK_LEVEL;
        break;

    case ELevelStep::RELEASE_CLICK_LEVEL:
        pkg.mouse_left = 0;
        pcontrollerLink_->SendPackage(CDevControllerLink::ID_CHOSELEVEL_DATA, pkg.header);
        levelStep_ = ELevelStep::MOVE_TO_YES;
        break;

    case ELevelStep::MOVE_TO_YES:
        pkg.x_position = Yes_Position[0];
        pkg.y_position = Yes_Position[1];
        pkg.mouse_left = 0;
        pkg.mouse_right = 0;
        pcontrollerLink_->SendPackage(CDevControllerLink::ID_CHOSELEVEL_DATA, pkg.header);
        levelStep_ = ELevelStep::CLICK_YES;
        break;

    case ELevelStep::CLICK_YES:
        pkg.mouse_left = 1;
        pcontrollerLink_->SendPackage(CDevControllerLink::ID_CHOSELEVEL_DATA, pkg.header);
        levelStep_ = ELevelStep::RELEASE_CLICK_YES;
        break;

    case ELevelStep::RELEASE_CLICK_YES:
        pkg.mouse_left = 0;
        pcontrollerLink_->SendPackage(CDevControllerLink::ID_CHOSELEVEL_DATA, pkg.header);
        levelStep_ = ELevelStep::IDLE;
        break;

    default:
        levelStep_ = ELevelStep::IDLE;
        break;
    }
}

} // namespace my_engineer
