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
#include <cstddef>
#include <cstring>

namespace my_engineer {

// 计算开平方的倒数
float_t inVSqrt(float_t x);

// 低通滤波
float_t LowPassFilter(float_t last_data, float_t current_data, float_t alpha);

/**
 * @brief 滑动窗口均值滤波器
 * @tparam N 窗口大小
 * @note 模板的实现只能够在头文件中实现
 */
template <size_t N> //使用非类型的参数模板在编译期优化性能
class CMovingAvgFilter {
public:
    CMovingAvgFilter() { Reset(); }

    float Update(float input) {
        sum_ -= buffer_[idx_];
        buffer_[idx_] = input;
        sum_ += input;
        idx_ = (idx_ + 1) % N;
        return sum_ / static_cast<float>(N);
    }

    float GetAverage() const { return sum_ / static_cast<float>(N); }

    void Reset() {
        memset(buffer_, 0, sizeof(buffer_));
        sum_ = 0.0f;
        idx_ = 0;
    }

private:
    float buffer_[N] = {0};
    float sum_ = 0.0f;
    size_t idx_ = 0;
};

} // namespace my_engineer

#endif // ALGO_OTHER_HPP
