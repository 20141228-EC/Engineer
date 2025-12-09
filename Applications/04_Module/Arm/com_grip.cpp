/**
 * @file com_grip.cpp
 * @author Ciallo～(∠·ω< )⌒☆(1002046597@qq.com)
 * @brief 机械臂夹爪组件
 * @version 1.0
 * @date 2025-11-27
 *
 * @copyright Copyright (c) 2025
 *
 */

 #include "mod_arm.hpp"

 namespace my_engineer {

/**
 * @brief 初始化夹爪组件
 *
 * @param param
 * @return EAppStatus
 */
EAppStatus CModArm::CComGrip::InitComponent(SModInitParam_Base &param) {
    if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

    auto armParam = static_cast<SModInitParam_Arm &>(param);

    motor = MotorIDMap.at(armParam.MotorID_Grip);
    mtrCanTxNode = armParam.MotorTxNode_Grip;

    // 初始化位置PID参数
    armParam.GripPosPidParam.threadNum = 1;
    pidPosCtrl.InitPID(&armParam.GripPosPidParam);

    // 初始化速度PID参数
    armParam.GripSpdPidParam.threadNum = 1;
    pidSpdCtrl.InitPID(&armParam.GripSpdPidParam);

    //输出缓冲区清零
    mtrOutputBuffer = 0;

    Component_FSMFlag_ = FSM_RESET;
    componentStatus = APP_OK;

    return APP_OK;
}

/**
 * @brief 更新组件
 *
 */
EAppStatus CModArm::CComGrip::UpdateComponent() {
    if (componentStatus == APP_RESET) return APP_ERROR;

    // 更新夹爪当前角度
    gripInfo.posit_grip = motor->motorData[CDevMtr::DATA_POSIT] * ARM_GRIP_MOTOR_DIR;

    gripInfo.isPositArrived_Grip = (abs(gripCmd.setPosit_grip - gripInfo.posit_grip) < 819);///< 位置误差小于1度认为到达目标

    switch (Component_FSMFlag_){

            case FSM_RESET:{
                // 复位状态，夹爪电机输出为0
                mtrOutputBuffer = 0;
                pidPosCtrl.ResetPidController();
                pidSpdCtrl.ResetPidController();
                return APP_OK;
            }
            case FSM_PREINIT:{
                // 预初始化状态，夹爪电机输出为0
                mtrOutputBuffer = 0;
                motor->motorData[CDevMtr::DATA_POSIT] = 0;
                pidPosCtrl.ResetPidController();
                pidSpdCtrl.ResetPidController();
                Component_FSMFlag_ = FSM_INIT;
                return APP_OK;
            }
            case FSM_INIT:{
                if(motor->motorStatus == CDevMtr::EMotorStatus::STALL){

                    gripCmd = SGripCmd();///<堵转之后设置目标值
                    motor->motorData[CDevMtr::DATA_POSIT] = static_cast<int32_t> (0.1*8192 + rangeLimit_Grip) * ARM_GRIP_MOTOR_DIR;///<堵转零点超量标定，注意标定之后要确定方向
                    gripInfo.isGripped = false;        ///<重置夹持状态
                    gripInfo.holdPosit_Grip = 0;       ///<清空夹持记忆位置
                    pidPosCtrl.ResetPidController();
                    pidSpdCtrl.ResetPidController();
                    Component_FSMFlag_ = FSM_CTRL;
                    componentStatus = APP_OK;
                    return APP_OK;
                }
                gripCmd.setPosit_grip += 500;///<按步长增加目标位置
                return _UpdateOutput(static_cast<float_t>(gripCmd.setPosit_grip));
            }
            case FSM_CTRL:{

                gripCmd.setPosit_grip = std::clamp<int32_t>(gripCmd.setPosit_grip, static_cast<int32_t>(0), rangeLimit_Grip);

                if(gripInfo.isGripped){
                    // 夹持模式
                    if(gripCmd.setPosit_grip > gripInfo.holdPosit_Grip + 819){ ///<目标位置向张开方向移动超过阈值，退出夹持
                        gripInfo.isGripped = false;
                        return _UpdateOutput(static_cast<float_t>(gripCmd.setPosit_grip));
                    }
                    return _UpdateOutput(static_cast<float_t>(gripInfo.holdPosit_Grip)); ///<保持夹持位置
                }
                else{
                    // 正常模式
                    bool isClosingDirection = (gripCmd.setPosit_grip < gripInfo.posit_grip);

                    if(motor->motorStatus == CDevMtr::EMotorStatus::STALL && isClosingDirection){
                        gripInfo.isGripped = true;
                        gripInfo.holdPosit_Grip = gripCmd.setPosit_grip; ///<记忆夹持位置
                    }

                    return _UpdateOutput(static_cast<float_t>(gripCmd.setPosit_grip));
                }
            }
            default:{
                StopComponent();
                mtrOutputBuffer = 0;
                pidPosCtrl.ResetPidController();
                pidSpdCtrl.ResetPidController();
                componentStatus = APP_ERROR;
                return APP_ERROR;
            }

        }
        return APP_OK;
}

/**
 * @brief 输出更新函数
 *
 * @param posit_Grip
 * @return EAppStatus
 */
EAppStatus CModArm::CComGrip::_UpdateOutput(float posit_Grip) {
    DataBuffer<float_t> gripPos = {
        static_cast<float_t>(posit_Grip) * ARM_GRIP_MOTOR_DIR
    };

    DataBuffer<float_t> gripPosMeasured = {static_cast<float_t>(motor->motorData[CDevMtr::DATA_POSIT])};

    auto pidPosOutput = pidPosCtrl.UpdatePidController(gripPos, gripPosMeasured);

    DataBuffer<float_t> gripSpdMeasured = {static_cast<float_t>(motor->motorData[CDevMtr::DATA_SPEED])};

    auto Output = pidSpdCtrl.UpdatePidController(pidPosOutput, gripSpdMeasured);

    mtrOutputBuffer = static_cast<int16_t>(Output[0]);

    return APP_OK;

}

/**
 * @brief 物理位置转换为电机位置
 *
 * @param phyPosit
 * @return int32_t
 */
int32_t CModArm::CComGrip::PhyPositToMtrPosit(float_t phyPosit) {
    const int32_t zeroOffset = 0; // 夹爪电机零点偏移
    const float_t ratio = ARM_END_GRIP_MOTOR_RATIO; // 夹爪电机与物理位置转换比（单位mm）
    return static_cast<int32_t>(phyPosit * ratio) + zeroOffset;
}

/**
 * @brief 电机位置转换为物理位置
 *
 * @param mtrPosit
 * @return float_t
 */

float_t CModArm::CComGrip::MtrPositToPhyPosit(float_t mtrPosit) {
    const int32_t zeroOffset = 0; // 夹爪电机零点偏移
    const float_t ratio = ARM_END_GRIP_MOTOR_RATIO; // 夹爪电机与物理位置转换比（单位mm）
    return (static_cast<float_t>(mtrPosit - zeroOffset) / ratio);
}



}//namespace my_engineer
