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

/**
 * @brief 斜坡函数
 * @note  每周期最多变化 step_ 的量，从当前值平滑逼近目标值。
 *        用于抑制阶跃输入造成的瞬时电流冲击。
 *
 * 用法：
 *   1. SetTarget(target, step)  设置目标和每周期最大变化量
 *   2. 每个控制周期调用 Update()  返回当前斜坡值
 *   3. isArrived()               判断是否已到达目标
 */
class CAlgoRamp{
public:
	CAlgoRamp() = default;

	explicit CAlgoRamp(float_t step) : step_(step) {}
	
	///< 设置目标值（step <= 0 则不改变步长）
	void SetTarget(float_t target, float_t step = 0.0f);
	
	///< 每周期调用，返回当前斜坡值
	float_t Update();

    ///< 直接设置当前值
    void SetValue(float_t value) { current_ = value; }

	///< 是否已到达目标
    bool IsArrived() const;
	
	///< 重置到指定值
    void Reset(float_t value = 0.0f) { current_ = value; target_ = value; }

private:
	float_t current_ = 0.f;
	float_t target_ = 0.f;
	float_t step_ = 0.f;
};

} // namespace my_engineer

#endif // ALGO_OTHER_HPP
