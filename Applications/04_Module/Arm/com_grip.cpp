/**
 * @file com_grip.cpp
 * @author Ciallo～(∠·ω< )⌒☆(1002046597@qq.com)
 * @brief 机械臂夹爪组件
 * @version 1.1
 * @date 2025-12-11
 *
 * @details V1.1: 添加Roll轴耦合补偿，解决Roll转动时夹爪被动移动的问题，但是比较难受的就是两个模块发生了耦合
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

    // 更新夹爪当前位置（含Roll耦合补偿，反映真实物理位置）
    gripInfo.posit_grip = static_cast<int32_t>(
        motor->motorData[CDevMtr::DATA_POSIT] * ARM_GRIP_MOTOR_DIR
        + gripInfo.rollCompAccum);

    // 获取End Roll位置用于耦合补偿
    int32_t endRollPosit = 0;
    if (parentModule != nullptr) {
        endRollPosit = parentModule->comEnd_.endInfo.posit_Roll;
    }

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

                    // 初始化设定位置记录（用于方向判断）
                    gripInfo.lastSetPosit = gripCmd.setPosit_grip;

                    // 初始化增量式 Roll 补偿
                    if (parentModule == nullptr ||  parentModule->comEnd_.componentStatus != APP_OK) {
                        return APP_ERROR;
                    }
                    rollPositAtGripInit_ = parentModule->comEnd_.endInfo.posit_Roll;
                    gripInfo.lastEndRollPosit = rollPositAtGripInit_;  ///< 初始化上一次位置
                    gripInfo.rollCompAccum = 0.0f;  ///< 清零累积补偿量

                    pidPosCtrl.ResetPidController();
                    pidSpdCtrl.ResetPidController();
                    Component_FSMFlag_ = FSM_CTRL;
                    componentStatus = APP_OK;
                    return APP_OK;
                }
                gripCmd.setPosit_grip += 500;///<按步长增加目标位置
                // 初始化阶段不进行Roll补偿
                return _UpdateOutput(static_cast<float_t>(gripCmd.setPosit_grip), 0);
            }
            case FSM_CTRL:{

                gripCmd.setPosit_grip = std::clamp<int32_t>(gripCmd.setPosit_grip, static_cast<int32_t>(0), rangeLimit_Grip);

                // 更新增量式Roll补偿
                _UpdateRollCompensation(endRollPosit);

                if(gripInfo.isGripped){
                    if(gripCmd.cmdReGrip){
                        gripCmd.cmdReGrip = false;
                        gripInfo.isGripped = false;          // 退出夹持锁定
                        gripInfo.holdPosit_Grip = 0;         // 清空记忆位置
                        return _UpdateOutput(gripCmd.setPosit_grip, endRollPosit); // 二次夹紧
                    }

                    // 夹持模式：向张开方向超过阈值则退出夹持
                    if(gripCmd.setPosit_grip > gripInfo.holdPosit_Grip + 819){
                        gripInfo.isGripped = false;
                        gripInfo.lastSetPosit = gripCmd.setPosit_grip;  // 更新上次设定位置
                        return _UpdateOutput(static_cast<float_t>(gripCmd.setPosit_grip), endRollPosit);  ///< 退出夹持，用新设定位置
                    }
                    gripInfo.lastSetPosit = gripCmd.setPosit_grip;  
                    return _UpdateOutput(static_cast<float_t>(gripInfo.holdPosit_Grip), endRollPosit); ///<保持夹持位置
                }
                else{
                    // 正常模式：堵转且闭合方向移动时进入夹持状态
                    // 闭合方向判断：当前设定位置 < 上次设定位置，即在往闭合方向设定
                    // 排除接近最大张开位置的堵转（物理限位导致的堵转不是夹取）
                    bool nearMaxOpen = (gripInfo.posit_grip > rangeLimit_Grip - 8192);

                    if(motor->motorStatus == CDevMtr::EMotorStatus::STALL &&
                       (gripCmd.setPosit_grip - gripInfo.lastSetPosit < -100) &&
                       !nearMaxOpen){
                        gripInfo.isGripped = true;
                        gripInfo.holdPosit_Grip = gripCmd.setPosit_grip; ///<记忆夹持位置
                    }

                    gripInfo.lastSetPosit = gripCmd.setPosit_grip; 
                    return _UpdateOutput(static_cast<float_t>(gripCmd.setPosit_grip), endRollPosit);
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
 * @brief 输出更新函数（Roll耦合补偿）
 *
 * @param gripTarget 夹爪期望位置
 * @param endRollPosit 末端Roll位置（实际使用gripInfo.rollCompAccum）
 * @return EAppStatus
 *
 * @note Roll轴转动会通过锥齿轮带动夹爪电机被动转动
 *       补偿原理: Roll转动 - 夹爪电机被动转 - 编码器读数变化（但夹爪实际没动）
 *       补偿公式: 夹爪真实位置 = 编码器读数 + gripInfo.rollCompAccum
 */
EAppStatus CModArm::CComGrip::_UpdateOutput(float gripTarget, int32_t endRollPosit) {
    // 目标位置gripTarget 已是电机坐标系值
    DataBuffer<float_t> gripPos = {
        static_cast<float_t>(gripTarget)
    };

    // 编码器读数 + 累积Roll补偿量 = 夹爪真实位置
    float_t actualGripPosit = static_cast<float_t>(motor->motorData[CDevMtr::DATA_POSIT])
                              + gripInfo.rollCompAccum; 
    DataBuffer<float_t> gripPosMeasured = {actualGripPosit};

    auto pidPosOutput = pidPosCtrl.UpdatePidController(gripPos, gripPosMeasured);

    DataBuffer<float_t> gripSpdMeasured = {static_cast<float_t>(motor->motorData[CDevMtr::DATA_SPEED])};

    auto Output = pidSpdCtrl.UpdatePidController(pidPosOutput, gripSpdMeasured);

    mtrOutputBuffer = static_cast<int16_t>(Output[0]);

    return APP_OK;

}

/**
 * @brief Roll补偿累积更新函数
 *
 * @param endRollPosit 末端Roll当前位置
 *
 * @note 采用增量式补偿，避免多圈累积导致补偿值过大
 *       跨圈检测：增量超过半圈说明是编码器溢出，需要修正
 */
void CModArm::CComGrip::_UpdateRollCompensation(int32_t endRollPosit) {
    int32_t deltaRoll = endRollPosit - gripInfo.lastEndRollPosit;

    // 增量超过半圈说明是编码器溢出
    if (deltaRoll > ARM_END_ROLL_HALF_TURN) {
        deltaRoll -= ARM_END_ROLL_ONE_TURN;
    } else if (deltaRoll < -ARM_END_ROLL_HALF_TURN) {
        deltaRoll += ARM_END_ROLL_ONE_TURN;
    }

    gripInfo.rollCompAccum += static_cast<float_t>(deltaRoll) * ARM_ROLL_GRIP_COUPLING_RATIO;
    gripInfo.lastEndRollPosit = endRollPosit;
}

/**
 * @brief 物理位置转换为电机位置
 *
 * @param phyPosit
 * @return int32_t
 */
int32_t CModArm::CComGrip::PhyPositToMtrPosit(float_t phyPosit) {
    const int32_t zeroOffset = 0; ///< 夹爪电机零点偏移
    const float_t ratio = ARM_END_GRIP_MOTOR_RATIO; ///< 夹爪电机与物理位置转换比（单位mm）
    return static_cast<int32_t>(phyPosit * ratio) + zeroOffset;
}

/**
 * @brief 电机位置转换为物理位置
 *
 * @param mtrPosit
 * @return float_t
 */

float_t CModArm::CComGrip::MtrPositToPhyPosit(float_t mtrPosit) {
    const int32_t zeroOffset = 0; ///< 夹爪电机零点偏移
    const float_t ratio = ARM_END_GRIP_MOTOR_RATIO; ///< 夹爪电机与物理位置转换比（单位mm）
    return (static_cast<float_t>(mtrPosit - zeroOffset) / ratio);
}



}//namespace my_engineer