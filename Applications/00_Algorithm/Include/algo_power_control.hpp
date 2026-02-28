/*
    * @file algo_power_control.hpp
    * @author yh
    * @brief 底盘功率限制头文件
    * @version 1.0
    * @date 2025-11-18
    *
*/
#ifndef ALGO_POWER_CONTROL_HPP
#define ALGO_POWER_CONTROL_HPP

#include "Configuration.hpp"

namespace my_engineer{
/**
 * @brief 底盘单电机功率限制类
 * 
 */
class CAlgoPowerControl {
public:
	struct SAlgoInitParamPower{
		uint16_t kDefaultMaxPower = 120;		//默认最大功率限制
		float kTorqueCoeff = 1.9968899944e-6f;	//力矩系数(20/16384)*(0.3)*(187/3591)/9.55
		float k1 = 1.23e-07f;					//力矩二次方系数
		float k2 = 1.453e-07f;					//转速二次方
		float kConstant = 4.081f; 				//静态功耗项
		int16_t kMotorOutputMax = 16000; 		//电机最大输出限制

	};
	
    // 功率限制算法状态
    EAppStatus PowerStatus = EAppStatus::APP_RESET;

	/**
     * @brief 初始化功率限制算法
     * @param pStructInitParam 初始化参数指针（传入不同参数适配不同电机）
     * @return EAppStatus 初始化状态（APP_OK/APP_ERROR）
	*/
	EAppStatus InitPowerControl(const SAlgoInitParamPower* pStructInitParam);

 	/**
     * @brief 实时更新功率限制（核心接口：输入电机当前状态，输出限制后转矩）
     * @param currentSpeedRpm 电机当前转速（单位：rpm）
     * @param currentTorque 电机当前目标转矩（未限制前的原始转矩）
     * @return int16_t 功率限制后的目标转矩（直接用于电机控制）
     */
	int16_t UpdatePowerLimit(float currentSpeedRpm, int16_t currentTorque);

	/**
     * @brief 计算电机当前功率（内部辅助函数）
     * @param speedRpm 转速（rpm）
     * @param torque 转矩
     * @return float 计算出的电机功率（W）
     */
	float CalcMotorPower(float speedRpm, float torque);
	/**
     * @brief 重置功率限制算法（恢复默认状态，清除历史计算值）
     * @return EAppStatus 重置状态（APP_OK/APP_ERROR）
     */
	 EAppStatus ResetPowerControl();
     /**
     * @brief 设置最大功率
     */
      void SetDefaultMaxPower(uint16_t maxPower) {
        kDefaultMaxPower = maxPower;
    }
private:
    // 功率限制核心参数,可通过初始化参数进行配置覆盖
    uint16_t kDefaultMaxPower;  ///< 最大允许功率
    float kTorqueCoeff;         ///< 转矩-功率转换系数
    float k1;                   ///< 转矩二次项损耗系数
    float k2;                   ///< 转速二次项损耗系数
    float kConstant;            ///< 固定损耗
    int16_t kMotorOutputMax;    ///< 最大转矩输出限幅
	/**
     * @brief 功率超限后，求解目标转矩（二次方程求解）
     * @param speedRpm 电机当前转速（rpm）
     * @param targetPower 目标功率（不超过maxPower_）
     * @param originalTorqueSign 原始转矩符号（保持方向一致）
     * @return float 限制后的目标转矩
     */
	float SolveTargetTorque(float speedRpm, float targetPower, int8_t originalTorqueSign);
};

}	//namespace my_engineer

#endif  // ALGO_POWER_CONTROL_HPP
