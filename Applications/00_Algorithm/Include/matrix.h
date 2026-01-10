/**
 ******************************************************************************
 * @file    matrix.cpp/h
 * @brief   Matrix/vector calculation. 矩阵/向量运算
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef MATRIX_H
#define MATRIX_H

#include "arm_math.h"

/**
/-------------------------------------如何使用此矩阵库----------------------------------------------/
 * @brief 由于原库注释均为英文且风格与本工程较为不同，因此写使用说明，并添加了部分注释
 * @author sllllr
 * @date 2025-12-23
 * @details robotics与utils同
 * 
 * 
 * 1.创建类对象
 *   此库不同于本工程，其实现的不是具体类，而是一个接收两个参数的类模板
 *   Matrixf不能直接被用来创建对象，而应当先根据模板参数创建一个具体类，然后再创建对象
 *   如: Matrixf<3,3> my_matrix
 *   其中:Matrixf<3,3>通过模板类生成了一个具体的，用于表示3×3矩阵的类
 *        而my_matrix则是通过这个具体类创建的对象
 *   
 * 2.函数使用
 *   由于实现的是类模板，因此任何创建，返回或操作可变Matrixf对象的独立函数，都必须被定义为函数模板，并传入要操作的行与列数
 *   如此库中实现的零矩阵生成函数
 *   template <int _rows, int _cols>
 *   Matrixf<_rows, _cols> zeros(void){ 函数实现... }
 *   不过也有不是模板函数的
 *   比如vector3f命名空间中的hat和cross，这两个函数就是写死了来处理3×1和3×3矩阵的函数，因此不需要成为模板
 * 
 * 3.关于utils.h
 *   其中实现的上下限和范围限制都可用std::clamp平替，在本工程中许多限幅正式用clamp实现的，但loopLimit与sign相当实用
 * 
 */


/* @brief 矩阵类模板
 * 
 * @param[input1]   矩阵行数
 * @param[input2]   矩阵列数
 * 
 */
template <int _rows, int _cols> ///< 类模板
class Matrixf {
 public:
 
  // 默认构造函数
  Matrixf(void) : rows_(_rows), cols_(_cols) {
    arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_);
  }

  // 根据传入的数组构造
  Matrixf(float data[_rows * _cols]) : Matrixf() {
    memcpy(this->data_, data, _rows * _cols * sizeof(float));
  }

  // 复制构造函数
  Matrixf(const Matrixf<_rows, _cols>& mat) : Matrixf() {
    memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
  }
  // 析构函数
  ~Matrixf(void) {}

  // 返回行数
  int rows(void) { return _rows; }
  // 返回列数
  int cols(void) { return _cols; }

  // 以下是符号重载

  // 取矩阵元素
  float* operator[](const int& row) { return &this->data_[row * _cols]; }

  // 运算符
  Matrixf<_rows, _cols>& operator=(const Matrixf<_rows, _cols> mat) {
    memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
    return *this;
  }

  Matrixf<_rows, _cols>& operator+=(const Matrixf<_rows, _cols> mat) {
    arm_status s;
    s = arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
    return *this;
  }

  Matrixf<_rows, _cols>& operator-=(const Matrixf<_rows, _cols> mat) {
    arm_status s;
    s = arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
    return *this;
  }

  Matrixf<_rows, _cols>& operator*=(const float& val) {
    arm_status s;
    s = arm_mat_scale_f32(&this->arm_mat_, val, &this->arm_mat_);
    return *this;
  }

  Matrixf<_rows, _cols>& operator/=(const float& val) {
    arm_status s;
    s = arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &this->arm_mat_);
    return *this;
  }

  Matrixf<_rows, _cols> operator+(const Matrixf<_rows, _cols>& mat) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
    return res;
  }

  Matrixf<_rows, _cols> operator-(const Matrixf<_rows, _cols>& mat) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
    return res;
  }

  // 矩阵数乘
  Matrixf<_rows, _cols> operator*(const float& val) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_scale_f32(&this->arm_mat_, val, &res.arm_mat_);
    return res;
  }

  friend Matrixf<_rows, _cols> operator*(const float& val,
                                         const Matrixf<_rows, _cols>& mat) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_scale_f32(&mat.arm_mat_, val, &res.arm_mat_);
    return res;
  }

  Matrixf<_rows, _cols> operator/(const float& val) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &res.arm_mat_);
    return res;
  }

  // 矩阵乘法
  template <int cols>
  friend Matrixf<_rows, cols> operator*(const Matrixf<_rows, _cols>& mat1,
                                        const Matrixf<_cols, cols>& mat2) {
    arm_status s;
    Matrixf<_rows, cols> res;
    s = arm_mat_mult_f32(&mat1.arm_mat_, &mat2.arm_mat_, &res.arm_mat_);
    return res;
  }

/* @brief 取子矩阵
 * 
 * @start_row   从原矩阵哪行开始截取
 * @start_col   从原矩阵哪列开始截取
 * 
 * @details 需要注意，在调用这个函数时，Matrixf<>中两个参数用于指定需要什么维度的子矩阵
 */
  template <int rows, int cols>
  Matrixf<rows, cols> block(const int& start_row, const int& start_col) {
    Matrixf<rows, cols> res;
    for (int row = start_row; row < start_row + rows; row++) {
      memcpy((float*)res[0] + (row - start_row) * cols,
             (float*)this->data_ + row * _cols + start_col,
             cols * sizeof(float));
    }
    return res;
  }

  // 创建行向量
  Matrixf<1, _cols> row(const int& row) { return block<1, _cols>(row, 0); }

  // 创建列向量
  Matrixf<_rows, 1> col(const int& col) { return block<_rows, 1>(0, col); }

  // 矩阵转置
  Matrixf<_cols, _rows> trans(void) {
    Matrixf<_cols, _rows> res;
    arm_mat_trans_f32(&arm_mat_, &res.arm_mat_);
    return res;
  }

  // 求迹
  float trace(void) {
    float res = 0;
    for (int i = 0; i < fmin(_rows, _cols); i++) {
      res += (*this)[i][i];
    }
    return res;
  }

  // 求矩阵范数
  float norm(void) { return sqrtf((this->trans() * *this)[0][0]); }

 public:
  // arm matrix instance
  arm_matrix_instance_f32 arm_mat_;

 protected:
  // size
  int rows_, cols_;

  // data
  float data_[_rows * _cols];
};

// 矩阵相关函数
namespace matrixf {

// 特殊矩阵
// 零矩阵
template <int _rows, int _cols>
Matrixf<_rows, _cols> zeros(void) {
  float data[_rows * _cols] = {0};
  return Matrixf<_rows, _cols>(data);
}
// 全1矩阵
template <int _rows, int _cols>
Matrixf<_rows, _cols> ones(void) {
  float data[_rows * _cols] = {0};
  for (int i = 0; i < _rows * _cols; i++) {
    data[i] = 1;
  }
  return Matrixf<_rows, _cols>(data);
}
// 单位矩阵
template <int _rows, int _cols>
Matrixf<_rows, _cols> eye(void) {
  float data[_rows * _cols] = {0};
  for (int i = 0; i < fmin(_rows, _cols); i++) {
    data[i * _cols + i] = 1;
  }
  return Matrixf<_rows, _cols>(data);
}
// 对角矩阵
template <int _rows, int _cols>
Matrixf<_rows, _cols> diag(Matrixf<_rows, 1> vec) {
  Matrixf<_rows, _cols> res = matrixf::zeros<_rows, _cols>();
  for (int i = 0; i < fmin(_rows, _cols); i++) {
    res[i][i] = vec[i][0];
  }
  return res;
}

// 求矩阵逆(通过构造增广矩阵，在将左边的A变换成单位矩阵I时，右边的原I部分会变成A的逆)
template <int _dim>
Matrixf<_dim, _dim> inv(Matrixf<_dim, _dim> mat) {
  arm_status s;
  // 增广矩阵 [A|I]
  Matrixf<_dim, 2 * _dim> ext_mat = matrixf::zeros<_dim, 2 * _dim>();
  for (int i = 0; i < _dim; i++) {
    memcpy(ext_mat[i], mat[i], _dim * sizeof(float)); // 将A的副本放到左半部分
    ext_mat[i][_dim + i] = 1; // 在右半部分对角线放1，即单位阵
  }
  // elimination
  for (int i = 0; i < _dim; i++) {
    // find maximum absolute value in the first column in lower right block
    float abs_max = fabs(ext_mat[i][i]);
    int abs_max_row = i;
    for (int row = i; row < _dim; row++) {
      if (abs_max < fabs(ext_mat[row][i])) {
        abs_max = fabs(ext_mat[row][i]);
        abs_max_row = row;
      }
    }
    if (abs_max < 1e-12f) {  // 矩阵奇异(非满秩)，则无逆矩阵
      return matrixf::zeros<_dim, _dim>();
      s = ARM_MATH_SINGULAR;
    }
    if (abs_max_row != i) {  // 行交换
      float tmp;
      Matrixf<1, 2 * _dim> row_i = ext_mat.row(i);
      Matrixf<1, 2 * _dim> row_abs_max = ext_mat.row(abs_max_row);
      memcpy(ext_mat[i], row_abs_max[0], 2 * _dim * sizeof(float));
      memcpy(ext_mat[abs_max_row], row_i[0], 2 * _dim * sizeof(float));
    }
    float k = 1.f / ext_mat[i][i];
    for (int col = i; col < 2 * _dim; col++) {
      ext_mat[i][col] *= k;
    }
    for (int row = 0; row < _dim; row++) {
      if (row == i) {
        continue;
      }
      k = ext_mat[row][i];
      for (int j = i; j < 2 * _dim; j++) {
        ext_mat[row][j] -= k * ext_mat[i][j];
      }
    }
  }
  // inv = ext_mat(:,n+1:2n)
  s = ARM_MATH_SUCCESS;
  Matrixf<_dim, _dim> res;
  for (int i = 0; i < _dim; i++) {
    memcpy(res[i], &ext_mat[i][_dim], _dim * sizeof(float));
  }
  return res;
}

}  // namespace matrixf

namespace vector3f {

// hat of vector
Matrixf<3, 3> hat(Matrixf<3, 1> vec);

// cross product
Matrixf<3, 1> cross(Matrixf<3, 1> vec1, Matrixf<3, 1> vec2);

}  // namespace vector3f

#endif  // MATRIX_H
