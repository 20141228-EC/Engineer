/**
 * @file conf_algo.hpp
 * @author sllllr (2997708711@qq.com)
 * @brief 算法配置
 * @version 1.0
 * @date 2026-01-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef CONF_ALGO_HPP
#define CONF_ALGO_HPP

#include "conf_common.hpp"

namespace my_engineer {

/**
 * @brief 配置并初始化所有算法
 * @return APP_OK - 初始化成功
 * @return APP_ERROR - 初始化失败
 */
EAppStatus InitAllAlgo();

} // namespace my_engineer

#endif  // CONF_ALGO_HPP