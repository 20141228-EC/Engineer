/******************************************************************************
 * @brief        定义履带组件
 * 
 * @file         com_crawler.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2026-01-31
 * 
 * @copyright    Copyright (c) 2026
 * 
 ******************************************************************************/

 #include "mod_chassis.hpp"

 namespace my_engineer{
/**
 * @brief 初始化底盘履带组件
 * 
 * @param param 
 * @return EAppStatus 
 */
EAppStatus CModChassis::CComCrawler::InitComponent(SModInitParam_Base &param){

    // 检查param是否正确
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	// 类型转换
	auto chassisParam = static_cast<SModInitParam_Chassis &>(param);

	// 保存电机指针
	motor[L] = MotorIDMap.at(chassisParam.crawlerMotorID_L);					
    motor[R] = MotorIDMap.at(chassisParam.crawlerMotorID_R);	                ///<通过对arm类图的索引找到初始注册的电机

	// 设置发送节点
    mtrCanTxNode[L] = chassisParam.crawlerMotorTxNodeID_L;
    mtrCanTxNode[R] = chassisParam.crawlerMotorTxNodeID_R;

	// 初始化PID控制器
    chassisParam.CrawlerSpdPidParam.threadNum = 2;
    PidCrawlerSpdCtrl.InitPID(&chassisParam.CrawlerSpdPidParam);

	// 初始化电机数据输出缓冲区
    mtrOutputBuffer.fill(0);

	Component_FSMFlag_ = FSM_RESET;
	componentStatus = APP_OK;

	return APP_OK;
}

/**
 * @brief 更新履带组件
 * 
 * @return EAppStatus 
 */
EAppStatus CModChassis::CComCrawler::UpdateComponent() {
	// 检查组件状态
	if (componentStatus == APP_RESET || !(parent->chassisInfo.crawler_on)) return APP_ERROR;

	// 更新组件信息
	CrawlerInfo.speed_L = motor[L]->motorData[CDevMtr::DATA_SPEED];
	CrawlerInfo.speed_R = motor[R]->motorData[CDevMtr::DATA_SPEED];

	switch (Component_FSMFlag_) {
		case FSM_RESET: {
			mtrOutputBuffer.fill(0);
            return APP_OK;
		}

		case FSM_PREINIT: {
			motor[L]->motorData[CDevMtr::DATA_POSIT] = 0;
            motor[R]->motorData[CDevMtr::DATA_POSIT] = 0;
            mtrOutputBuffer.fill(0);
            PidCrawlerSpdCtrl.ResetPidController();
            Component_FSMFlag_ = FSM_INIT;
            return APP_OK;
		}

		case FSM_INIT: {
			CrawlerCmd = SCrawlerCommand();
            Component_FSMFlag_ = FSM_CTRL;
            componentStatus = APP_OK;
            return _UpdateOutput(CrawlerCmd.speed_crawler);
		}

		case FSM_CTRL: {
            return _UpdateOutput(CrawlerCmd.speed_crawler);
		}

		default: {
			componentStatus = APP_ERROR;
			return APP_ERROR;
		}
	}

	return APP_OK;
}

EAppStatus CModChassis::CComCrawler::_UpdateOutput(float_t speed){

	// 从履带电机中读取当前速度存入缓冲区
    DataBuffer<float_t> crawlerSpdMeasure = {
        static_cast<float_t>(motor[L]->motorData[CDevMtr::DATA_SPEED]),
        static_cast<float_t>(motor[R]->motorData[CDevMtr::DATA_SPEED]),
    };

    DataBuffer<float_t> crawlerSpd = {speed,-speed,};    // 更新目标速度

    auto output = PidCrawlerSpdCtrl.UpdatePidController(crawlerSpd, crawlerSpdMeasure);

    mtrOutputBuffer = {static_cast<int16_t>(output[L]), static_cast<int16_t>(output[R]),};
		
	return APP_OK;
}
}  // namespace my_engineer