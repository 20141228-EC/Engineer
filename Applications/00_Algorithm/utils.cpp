/**
 ******************************************************************************
 * @file    utils.cpp/h
 * @brief   General math utils. 常用数学工具函数
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "utils.h"

// 限制范围
float math::limit(float val, const float& min, const float& max) {
  if (min > max)
    return val;
  else if (val < min)
    val = min;
  else if (val > max)
    val = max;
  return val;
}

// 限制下限
float math::limitMin(float val, const float& min) {
  if (val < min)
    val = min;
  return val;
}

// 限制上限
float math::limitMax(float val, const float& max) {
  if (val > max)
    val = max;
  return val;
}

// 多圈限制，如loopLimit(400, 0, 360)返回 40
float math::loopLimit(float val, const float& min, const float& max) {
  if (min >= max)
    return val;
  if (val > max) {
    while (val > max)
      val -= (max - min);
  } else if (val < min) {
    while (val < min)
      val += (max - min);
  }
  return val;
}

// 取符号函数
float math::sign(const float& val) {
  if (val > 0)
    return 1;
  else if (val < 0)
    return -1;
  return 0;
}

uint32_t math::NextRandom(uint32_t &state)
{
  state ^= state << 13;
  state ^= state >> 17;
  state ^= state << 5;
  return state;
}

float math::RandomRange(uint32_t &state, float minValue, float maxValue)
{
  constexpr float kUint32Max = 4294967295.0f;
  const float unitRandom =
      static_cast<float>(NextRandom(state)) / kUint32Max;
  return minValue + (maxValue - minValue) * unitRandom;
}

