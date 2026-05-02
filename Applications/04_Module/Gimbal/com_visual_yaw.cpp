/******************************************************************************
 * @brief        
 * 
 * @file         com_visual_yaw.cpp
 * @author       ciallo
 * @version      V1.0
 * @date         2026-03-28
 * 
 * @copyright    Copyright (c) 2026
 * 
 ******************************************************************************/

#include "mod_gimbal.hpp"
#include "algo_other.hpp"

namespace my_engineer{

EAppStatus CModGimbal::CComVisualyaw::InitComponent(SModInitParam_Base &param){
    
    if(param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

    auto gimbalParam = static_cast<SModInitParam_Gimbal &> (param);

    motor = MotorIDMap.at(gimbalParam.yawVisualMotorID);
    
    mitCtrl.kp = gimbalParam.MIT_YAW_kp;
    mitCtrl.kd = gimbalParam.MIT_YAW_kd;

    Component_FSMFlag_ = FSM_RESET;
	componentStatus = APP_OK;

    return APP_OK;
}

EAppStatus CModGimbal::CComVisualyaw::UpdateComponent(){

    if (componentStatus == APP_RESET) {
        static_cast<CDevMtrDM_MIT *>(motor)->Control_MIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        return APP_ERROR;
    }

    CDevMtrDM_MIT *pMtr = static_cast<CDevMtrDM_MIT *>(motor);
	VisuallyawInfo.angle = rad2deg(pMtr->motorPhyAngle);
    VisuallyawInfo.isAngleArrived = (fabs(VisuallyawInfo.angle - VisuallyawCmd.setAngle) < 2.f);
    
    //零点标定
    uint8_t test=0;
    if(test == 1){
		pMtr->SetZero();			
    }

    switch (Component_FSMFlag_)
    {
    case FSM_RESET:{
        pMtr->Control_MIT(0.0f, 0.0f, deg2rad(0.0f) * GIMBAL_VISUAL_MOTOR_MOTOR_DIR, 0.0f, 0.0f);
        VisuallyawCmd = SVisuallyawCmd{};
        return APP_OK;
    }
    case FSM_PREINIT:{
        VisuallyawCmd.setAngle = GIMBAL_VISUAL_MOTOR_INIT_ANGLE;
        Component_FSMFlag_ = FSM_INIT;
        return APP_OK;
    }
    case FSM_INIT:{
		if (fabs(VisuallyawInfo.angle - VisuallyawCmd.setAngle) < 10.0) {
			Component_FSMFlag_ = FSM_CTRL;
			componentStatus = APP_OK;
		}
		pMtr->Control_MIT(mitCtrl.kp, mitCtrl.kd, deg2rad(VisuallyawCmd.setAngle), 0.0f, 0.0f);
		return APP_OK;
    }  
	case FSM_CTRL: {
			pMtr->Control_MIT(mitCtrl.kp, mitCtrl.kd, deg2rad(LowPassFilter(VisuallyawInfo.angle,VisuallyawCmd.setAngle,0.7)), 0.0f, 0.0f);//低通滤波控制角度
			return APP_OK;
		}

	default: {
			componentStatus = APP_ERROR;
			return APP_ERROR;
	}

	return APP_OK;

}

}
}// namespace my_engineer