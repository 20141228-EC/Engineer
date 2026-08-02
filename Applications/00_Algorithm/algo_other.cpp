/******************************************************************************
 * @brief        
 * 
 * @file         algo_other.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-04-01
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "algo_other.hpp"

namespace my_engineer {

float_t inVSqrt(float_t x) {
	union {
		int32_t i;
		float_t x;
	} u;
	u.x = x;
	u.i = 0x5f3759df - (u.i >> 1);
	return u.x * (1.5f - 0.5f * x * u.x * u.x);
}

float_t LowPassFilter(float_t last_data, float_t current_data, float_t alpha) {
	return last_data * alpha + current_data * (1 - alpha);
}

float_t Round(float_t x) {
	if (x > 0) {
		return static_cast<float_t>(static_cast<int32_t>(x + 0.5f));
	}
	else {
		return static_cast<float_t>(static_cast<int32_t>(x - 0.5f));
	}
}

float_t CAlgoLinearInterp::update(){
		if (step_ < period_) step_++;
		float_t t = static_cast<float_t>(step_) / static_cast<float_t>(period_);
		return start_ + (target_ - start_) * t;
	};
void CAlgoLinearInterp::setTarget(float_t current, float_t target){
		start_ = current;
		target_ = target;
		step_ = 0;
	}	

void CAlgoRamp::SetTarget(float_t target, float_t step) {
    target_ = target;
    if (step > 0.0f) step_ = step;
}

float_t CAlgoRamp::Update() {
    float_t diff = target_ - current_;
    if (fabsf(diff) <= step_) {
        current_ = target_;
    } else {
        current_ += (diff > 0.0f ? step_ : -step_);
    }
    return current_;
}

bool CAlgoRamp::IsArrived() const {
    return fabsf(target_ - current_) < 1e-6f;
}
} // namespace my_engineer
