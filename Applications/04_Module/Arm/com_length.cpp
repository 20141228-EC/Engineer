
#include "mod_arm.hpp"

namespace my_engineer
{

    /**
     * @brief 初始化机械臂末端组件
     *
     * @param param
     * @return EAppStatus
     */
    EAppStatus CModArm::CComClaw::InitComponent(SModInitParam_Base &param)
    {
        if (param.moduleID == EModuleID::MOD_NULL)
            return APP_ERROR;

        auto armParam = static_cast<SModInitParam_Arm &>(param);

        motor[L] = MotorIDMap.at(armParam.MotorID_length_L);

        mtrCanTxNode[L] = armParam.MotorTxNode_length_L;
        // mtrCanTxNode[R] = armParam.MotorTxNode_length_R;

        armParam.endPosPidParam.threadNum = 1;
        pidPosCtrl.InitPID(&armParam.endPosPidParam);

        armParam.endSpdPidParam.threadNum = 1;
        pidSpdCtrl.InitPID(&armParam.endSpdPidParam);

        mtrOutputBuffer.fill(0);

        Component_FSMFlag_ = FSM_RESET;
        componentStatus = APP_OK;

        return APP_OK;
    }

    /**
     * @brief 更新组件
     *
     */
    EAppStatus CModArm::CComClaw::UpdateComponent()
    {
        if (componentStatus == APP_RESET)
            return APP_ERROR;

        clawInfo.posit_Length = motor[L]->motorData[CDevMtr::DATA_POSIT];

        clawInfo.isPositArrived_Length = (abs(clawCmd.setPosit_Length - clawInfo.posit_Length) < 8192 * 2);

        switch (Component_FSMFlag_)
        {

        case FSM_RESET:
        {
            mtrOutputBuffer.fill(0);
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            return APP_OK;
        }

        case FSM_PREINIT:
        {
            clawCmd.setPosit_Length = 0;
            motor[L]->motorData[CDevMtr::DATA_POSIT] = 0;
            // motor[R]->motorData[CDevMtr::DATA_POSIT] = 0;
            mtrOutputBuffer.fill(0);
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            Component_FSMFlag_ = FSM_INIT;
            return APP_OK;
        }

        case FSM_INIT:
        {
            if (motor[L]->motorStatus == CDevMtr::EMotorStatus::STALL)
            {
                motor[L]->motorData[CDevMtr::DATA_POSIT] = -(static_cast<int32_t>(0.5 * 8192));     // 初始值为0
                pidPosCtrl.ResetPidController();
                pidSpdCtrl.ResetPidController();
                componentStatus = APP_OK;
                Component_FSMFlag_ = FSM_CTRL;
                return APP_OK;
            }
            clawCmd.setPosit_Length += 200;         // 递增是收紧
            return _UpdateOutput(static_cast<float_t>(clawCmd.setPosit_Length));
        }

        case FSM_CTRL:
        {
            // clawCmd.setPosit_Length = std::clamp(clawCmd.setPosit_Length, static_cast<int32_t>(0), rangeLimit_Length);
            return _UpdateOutput(static_cast<float_t>(clawCmd.setPosit_Length));
        }

        default:
        {
            StopComponent();
            mtrOutputBuffer.fill(0);
            pidPosCtrl.ResetPidController();
            pidSpdCtrl.ResetPidController();
            componentStatus = APP_ERROR;
            return APP_ERROR;
        }
        }

        return APP_OK;
    }

    /**
     * @brief 物理位置转换为电机位置
     *
     * @param phyPosit
     * @return int32_t
     */
    int32_t CModArm::CComClaw::PhyPositToMtrPosit_Length(float_t phyPosit)
    {
        const int32_t zeroOffset = ARM_END_LENGTH_MOTOR_OFFSET;
        const float_t scale = - ARM_END_LENGTH_MOTOR_RANGE/100.0f;

        return (static_cast<int32_t>(phyPosit * scale) + zeroOffset);
    }

    /**
     * @brief 电机位置转换为物理位置
     *
     * @param mtrPosit
     * @return float_t
     */
    float_t CModArm::CComClaw::MtrPositToPhyPosit_Length(int32_t mtrPosit)
    {
        const int32_t zeroOffset = ARM_END_LENGTH_MOTOR_OFFSET;
        const float_t scale = - ARM_END_LENGTH_MOTOR_RANGE / 100.0f;

        return (static_cast<float_t>(mtrPosit - zeroOffset) / scale);
    }

    /**
     * @brief 输出更新函数
     *
     * @param posit_Pitch
     * @param posit_Roll
     * @return EAppStatus
     */
    EAppStatus CModArm::CComClaw::_UpdateOutput(float_t posit_Length)
    {
        DataBuffer<float_t> endPos = {
            posit_Length,
        };

        DataBuffer<float_t> endPosMeasure = {
            static_cast<float_t>(motor[L]->motorData[CDevMtr::DATA_POSIT]),
        };

        auto endSpd = pidPosCtrl.UpdatePidController(endPos, endPosMeasure);

        DataBuffer<float_t> endSpdMeasure = {
            static_cast<float_t>(motor[L]->motorData[CDevMtr::DATA_SPEED]),
            };

        auto output = pidSpdCtrl.UpdatePidController(endSpd, endSpdMeasure);

        mtrOutputBuffer = {
            static_cast<int16_t>(output[L]),
        };

        return APP_OK;
    }

} // namespace my_engineer
