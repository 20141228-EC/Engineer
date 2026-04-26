/******************************************************************************
 * @brief        
 * 
 * @file         algo_other.hpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-04-01
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#ifndef ALGO_OTHER_HPP
#define ALGO_OTHER_HPP

#include "Configuration.hpp"

namespace my_engineer {

// 计算开平方的倒数
float_t inVSqrt(float_t x);

// 低通滤波
float_t LowPassFilter(float_t last_data, float_t current_data, float_t alpha);

// 四舍五入
float_t Round(float_t x);

// 半圈处理
template<typename T>
T HalfCycle(T source, T range) {
	if(source > range / static_cast<T>(2)){
		source -= range;
	}
	if(source < -range / static_cast<T>(2)){
		source += range;
	}
	return source;
}

/**
 * @brief 线性插值器
 *
 * @note 在固定周期内从起始值线性过渡到目标值，
 *       适用于低频数据源（如25Hz控制器）驱动高频控制环（如1000Hz）的场景。但是还是会存在延时可以考虑采用前进加后退的Eluer预测校正算法来进一步拟合
 *
 * 用法：
 *   1. 检测到新数据时调用 setTarget(当前值, 目标值)
 *   2. 每个控制周期调用 update() 获取插值结果
 */
class CAlgoLinearInterp {
public:
	explicit CAlgoLinearInterp(uint32_t period = 40) : period_(period) {}

	/// 收到新目标值，开始新一轮插值
	void setTarget(float_t current, float_t target) ;

	/// 每个控制周期调用，返回插值结果
	float_t update() ;

	/// 获取当前目标值
	float_t getTarget() const { return target_; }

private:
	float_t start_ = 0.0f;
	float_t target_ = 0.0f;
	uint32_t step_ = 0;
	uint32_t period_;
};

} // namespace my_engineer

#endif // ALGO_OTHER_HPP
