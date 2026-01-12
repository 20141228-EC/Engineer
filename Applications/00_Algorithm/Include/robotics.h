/**
 ******************************************************************************
 * @file    robotics.cpp/h
 * @brief   Robotic toolbox on STM32. STM32机器人学库
 * @author  Spoon Guan
 * @ref     [1] SJTU ME385-2, Robotics, Y.Ding
 *          [2] Bruno Siciliano, et al., Robotics: Modelling, Planning and
 *              Control, Springer, 2010.
 *          [3] R.Murry, Z.X.Li, and S.Sastry, A Mathematical Introduction
 *              to Robotic Manipulation, CRC Press, 1994.
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef ROBOTICS_H
#define ROBOTICS_H

#include "utils.h"
#include "matrix.h"

namespace robotics {
// 旋转矩阵R转换为欧拉角(偏航角yaw,俯仰角pitch,横滚角roll)
Matrixf<3, 1> r2rpy(Matrixf<3, 3> R);
// 欧拉角(偏航角yaw,俯仰角pitch,横滚角roll)转换为旋转矩阵R
Matrixf<3, 3> rpy2r(Matrixf<3, 1> rpy);
// 旋转矩阵R转换为轴角法表示的4×1列向量([旋转轴r;旋转角度θ])
Matrixf<4, 1> r2angvec(Matrixf<3, 3> R);
// 轴角向量([旋转轴r;旋转角度θ])转换为旋转矩阵(R)
Matrixf<3, 3> angvec2r(Matrixf<4, 1> angvec);
// 旋转矩阵(R)转换为四元数, [q0;q1;q2;q3]=[cos(θ/2);r*sin(θ/2)]
Matrixf<4, 1> r2quat(Matrixf<3, 3> R);
// 四元数([q0;q1;q2;q3]=[cos(θ/2);r*sin(θ/2)])转换为旋转矩阵(R)
Matrixf<3, 3> quat2r(Matrixf<4, 1> quat);
// 四元数([q0;q1;q2;q3]=[cos(θ/2);r*sin(θ/2)])转换为欧拉角RPY([偏航角;俯仰角;横滚角])
Matrixf<3, 1> quat2rpy(Matrixf<4, 1> q);
// 欧拉角RPY([偏航角;俯仰角;横滚角])转换为四元数([q0;q1;q2;q3]=[cos(θ/2);r*sin(θ/2)])
Matrixf<4, 1> rpy2quat(Matrixf<3, 1> rpy);
// 四元数([q0;q1;q2;q3]=[cos(θ/2);r*sin(θ/2)])转换为轴角向量([旋转轴r;旋转角度θ])
Matrixf<4, 1> quat2angvec(Matrixf<4, 1> q);
// 轴角向量([旋转轴r;旋转角度θ])转换为四元数([q0;q1;q2;q3]=[cos(θ/2);r*sin(θ/2)])
Matrixf<4, 1> angvec2quat(Matrixf<4, 1> angvec);
// 齐次变换矩阵(T)提取旋转矩阵(R)
Matrixf<3, 3> t2r(Matrixf<4, 4> T);
// 旋转矩阵(R)扩展为齐次变换矩阵(T)
Matrixf<4, 4> r2t(Matrixf<3, 3> R);
// 齐次变换矩阵(T)提取平移向量(p)
Matrixf<3, 1> t2p(Matrixf<4, 4> T);
// 平移向量(p)扩展为齐次变换矩阵(T)
Matrixf<4, 4> p2t(Matrixf<3, 1> p);
// 旋转矩阵(R)和平移向量(p)组合为齐次变换矩阵(T)
Matrixf<4, 4> rp2t(Matrixf<3, 3> R, Matrixf<3, 1> p);
// 齐次变换矩阵(T)转换为欧拉角RPY([偏航角;俯仰角;横滚角])
Matrixf<3, 1> t2rpy(Matrixf<4, 4> T);
// 齐次变换矩阵的逆矩阵(T^-1=[R',-R'P;0,1])
Matrixf<4, 4> invT(Matrixf<4, 4> T);
// 欧拉角RPY([偏航角;俯仰角;横滚角])扩展为齐次变换矩阵(T)
Matrixf<4, 4> rpy2t(Matrixf<3, 1> rpy);
// 齐次变换矩阵(T)转换为轴角向量([旋转轴r;旋转角度θ])
Matrixf<4, 1> t2angvec(Matrixf<4, 4> T);
// 轴角向量([旋转轴r;旋转角度θ])扩展为齐次变换矩阵(T)
Matrixf<4, 4> angvec2t(Matrixf<4, 1> angvec);
// 齐次变换矩阵(T)转换为四元数([q0;q1;q2;q3]=[cos(θ/2);r*sin(θ/2)])
Matrixf<4, 1> t2quat(Matrixf<4, 4> T);
// 四元数([q0;q1;q2;q3]=[cos(θ/2);r*sin(θ/2)])扩展为齐次变换矩阵(T)
Matrixf<4, 4> quat2t(Matrixf<4, 1> quat);
// 齐次变换矩阵(T)转换为旋量坐标向量 ([平移分量p;旋转分量rθ])
Matrixf<6, 1> t2twist(Matrixf<4, 4> T);
// 旋量坐标向量 ([平移分量p;旋转分量rθ])转换为齐次变换矩阵(T)
Matrixf<4, 4> twist2t(Matrixf<6, 1> twist);

// 关节类型：R-旋转关节，P-移动关节
typedef enum joint_type {
  R = 0,    // 旋转关节
  P = 1,    // 移动关节
} Joint_Type_e;

// 标准DH参数法结构体
struct DH_t {
  // 正运动学求解
  Matrixf<4, 4> fkine();
  // DH参数
  float theta;  // 关节转角
  float d;      // 关节偏移量
  float a;      // 连杆长度
  float alpha;  // 连杆扭角
  Matrixf<4, 4> T;  // 齐次变换矩阵
};  // 定义DH法结构体

class Link {
 public:
  Link(){};
  /**
   * @brief 连杆类构造函数
   * @param theta DH参数-关节转角
   * @param d DH参数-关节偏移量
   * @param a DH参数-连杆长度
   * @param alpha DH参数-连杆扭角
   * @param type 关节类型(默认旋转关节)
   * @param offset 关节偏移量
   * @param qmin 关节运动下限
   * @param qmax 关节运动上限
   * @param m 连杆质量
   * @param rc 质心坐标(连杆局部坐标系)
   * @param I 惯性张量(3*3矩阵)
   */
  Link(float theta, float d, float a, float alpha, Joint_Type_e type = R,
       float offset = 0, float qmin = 0, float qmax = 0, float m = 1,
       Matrixf<3, 1> rc = matrixf::zeros<3, 1>(),
       Matrixf<3, 3> I = matrixf::zeros<3, 3>());
  // 拷贝构造函数
  Link(const Link& link);

  // 赋值运算符重载
  Link& operator=(Link link);

  // 获取关节下限
  float qmin() { return qmin_; }
  // 获取关节上限
  float qmax() { return qmax_; }
  // 获取关节类型
  Joint_Type_e type() { return type_; }
  // 获取连杆质量
  float m() { return m_; }
  // 获取质心坐标
  Matrixf<3, 1> rc() { return rc_; }
  // 获取惯性张量
  Matrixf<3, 3> I() { return I_; }

  // 正运动学求解：根据关节变量计算齐次变换矩阵
  Matrixf<4, 4> T(float q);  // 正运动学

 public:
  // 运动学参数
  DH_t dh_;          // DH参数结构体
  float offset_;     // 关节偏移量
  // 关节限位(qmin,qmax)，若qmin<=qmax则表示无限位
  float qmin_;       // 关节下限
  float qmax_;       // 关节上限
  Joint_Type_e type_;// 关节类型
  // 动力学参数
  float m_;           // 质量
  Matrixf<3, 1> rc_;  // 质心坐标(连杆局部坐标系)
  Matrixf<3, 3> I_;   // 惯性张量(3*3矩阵)
};

template <uint16_t _n = 1>
class Serial_Link {
 public:
  /**
   * @brief 串联连杆类构造函数
   * @param links 连杆数组
   */
  Serial_Link(Link links[_n]) {
    for (int i = 0; i < _n; i++)
      links_[i] = links[i];
    gravity_ = matrixf::zeros<3, 1>();
    gravity_[2][0] = -9.81f;  // 默认重力加速度(沿z轴负方向)
  }

  /**
   * @brief 串联连杆类构造函数(自定义重力)
   * @param links 连杆数组
   * @param gravity 重力加速度向量
   */
  Serial_Link(Link links[_n], Matrixf<3, 1> gravity) {
    for (int i = 0; i < _n; i++)
      links_[i] = links[i];
    gravity_ = gravity;
  }

  /**
   * @brief 正运动学求解：计算末端执行器相对于基坐标系的齐次变换矩阵T_n^0
   * @param q 关节变量向量
   * @return 末端执行器齐次变换矩阵T_n^0
   */
  Matrixf<4, 4> fkine(Matrixf<_n, 1> q) {
    T_ = matrixf::eye<4, 4>();  // 初始化为单位矩阵
    // 依次累乘各连杆的齐次变换矩阵
    for (int iminus1 = 0; iminus1 < _n; iminus1++)
      T_ = T_ * links_[iminus1].T(q[iminus1][0]);
    return T_;
  }

  /**
   * @brief 正运动学求解：计算第k个关节相对于基坐标系的齐次变换矩阵T_k^0
   * @param q 关节变量向量
   * @param k 关节编号
   * @return 第k个关节的齐次变换矩阵T_k^0
   */
  Matrixf<4, 4> fkine(Matrixf<_n, 1> q, uint16_t k) {
    if (k > _n)  // 防止越界
      k = _n;
    Matrixf<4, 4> T = matrixf::eye<4, 4>();
    // 累乘前k个连杆的齐次变换矩阵
    for (int iminus1 = 0; iminus1 < k; iminus1++)
      T = T * links_[iminus1].T(q[iminus1][0]);
    return T;
  }

  /**
   * @brief 计算第k个连杆相对于第k-1个连杆的齐次变换矩阵T_k^k-1
   * @param q 关节变量向量
   * @param kminus1 关节编号k的前一个关节(k-1)
   * @return 齐次变换矩阵T_k^k-1
   */
  Matrixf<4, 4> T(Matrixf<_n, 1> q, uint16_t kminus1) {
    if (kminus1 >= _n)  // 防止越界
      kminus1 = _n - 1;
    return links_[kminus1].T(q[kminus1][0]);
  }

  /**
   * @brief 雅可比矩阵求解，J_i = [位置雅可比J_pi;姿态雅可比J_oi]
   * @param q 关节变量向量
   * @return 6*n维雅可比矩阵J
   */
  Matrixf<6, _n> jacob(Matrixf<_n, 1> q) {
    Matrixf<3, 1> p_e = t2p(fkine(q));               // 末端执行器位置
    Matrixf<4, 4> T_iminus1 = matrixf::eye<4, 4>();  // 第i-1个连杆的齐次矩阵
    Matrixf<3, 1> z_iminus1;                         // 第i-1个连杆z轴单位向量(基坐标系)
    Matrixf<3, 1> p_iminus1;                         // 第i-1个连杆原点位置(基坐标系)
    Matrixf<3, 1> J_pi;                              // 位置雅可比分量
    Matrixf<3, 1> J_oi;                              // 姿态雅可比分量
    
    for (int iminus1 = 0; iminus1 < _n; iminus1++) {
      // 旋转关节：J_pi = z_i-1 × (p_e - p_i-1), J_oi = z_i-1
      if (links_[iminus1].type() == R) {
        z_iminus1 = T_iminus1.block<3, 1>(0, 2);     // 提取z轴向量
        p_iminus1 = t2p(T_iminus1);                  // 提取位置向量
        // 更新第i个连杆的齐次矩阵
        T_iminus1 = T_iminus1 * links_[iminus1].T(q[iminus1][0]);
        // 计算位置雅可比
        J_pi = vector3f::cross(z_iminus1, p_e - p_iminus1);
        // 姿态雅可比等于z轴向量
        J_oi = z_iminus1;
      }
      // 移动关节：J_pi = z_i-1, J_oi = 0
      else {
        z_iminus1 = T_iminus1.block<3, 1>(0, 2);     // 提取z轴向量
        // 更新第i个连杆的齐次矩阵
        T_iminus1 = T_iminus1 * links_[iminus1].T(q[iminus1][0]);
        // 位置雅可比等于z轴向量
        J_pi = z_iminus1;
        // 姿态雅可比为零向量
        J_oi = matrixf::zeros<3, 1>();
      }
      // 赋值到雅可比矩阵
      J_[0][iminus1] = J_pi[0][0];
      J_[1][iminus1] = J_pi[1][0];
      J_[2][iminus1] = J_pi[2][0];
      J_[3][iminus1] = J_oi[0][0];
      J_[4][iminus1] = J_oi[1][0];
      J_[5][iminus1] = J_oi[2][0];
    }
    return J_;
  }

  /**
   * @brief 逆运动学求解，数值解法(牛顿迭代法)
   * @param Td 期望的末端执行器齐次变换矩阵
   * @param q 牛顿迭代法初始关节变量向量(q0)
   * @param tol 误差容限(旋量向量误差的范数)
   * @param max_iter 最大迭代次数，默认50次
   * @return 求解得到的关节变量向量
   */
  Matrixf<_n, 1> ikine(Matrixf<4, 4> Td,
                       Matrixf<_n, 1> q = matrixf::zeros<_n, 1>(),
                       float tol = 1e-4f, uint16_t max_iter = 50) {
    Matrixf<4, 4> T;                // 当前末端执行器齐次矩阵
    Matrixf<3, 1> pe, we;           // 位置误差和姿态误差
    Matrixf<6, 1> err, new_err;     // 误差向量和新误差向量
    Matrixf<_n, 1> dq;              // 关节变量增量
    float step = 1;                 // 迭代步长
    
    for (int i = 0; i < max_iter; i++) {
      // 正运动学计算当前末端矩阵
      T = fkine(q);
      // 计算位置误差
      pe = t2p(Td) - t2p(T);
      // 计算姿态误差(世界坐标系下从当前姿态到期望姿态的轴角向量)
      we = t2twist(Td * invT(T)).block<3, 1>(3, 0);
      
      // 构造6维误差向量
      for (int i = 0; i < 3; i++) {
        err[i][0] = pe[i][0];       // 位置误差分量
        err[i + 3][0] = we[i][0];   // 姿态误差分量
      }
      
      // 误差小于容限，迭代收敛
      if (err.norm() < tol)
        return q;
      
      // 计算雅可比矩阵
      Matrixf<6, _n> J = jacob(q);
      // 调整迭代步长，最多尝试5次
      for (int j = 0; j < 5; j++) {
        // 计算关节增量(伪逆求解)
        dq = matrixf::inv(J.trans() * J) * (J.trans() * err) * step;
        
        // 处理雅可比矩阵奇异情况
        if (dq[0][0] == INFINITY)  // J'*J 奇异
        {
          // 添加阻尼项的伪逆求解
          dq = matrixf::inv(J.trans() * J + 0.1f * matrixf::eye<_n, _n>()) *
               J.trans() * err * step;
          // 更新关节变量
          q += dq;
          // 旋转关节角度归一化到[-π, π]
          for (int i = 0; i < _n; i++) {
            if (links_[i].type() == R)
              q[i][0] = math::loopLimit(q[i][0], -PI, PI);
          }
          break;
        }
        
        // 验证步长是否有效
        T = fkine(q + dq);
        pe = t2p(Td) - t2p(T);
        we = t2twist(Td * invT(T)).block<3, 1>(3, 0);
        // 计算新误差
        for (int i = 0; i < 3; i++) {
          new_err[i][0] = pe[i][0];
          new_err[i + 3][0] = we[i][0];
        }
        
        // 新误差更小，接受该步长
        if (new_err.norm() < err.norm()) {
          q += dq;
          // 旋转关节角度归一化
          for (int i = 0; i < _n; i++) {
            if (links_[i].type() == robotics::Joint_Type_e::R) {
              q[i][0] = math::loopLimit(q[i][0], -PI, PI);
            }
          }
          break;
        } else {
          // 新误差更大，步长减半
          step /= 2.0f;
        }
      }
      
      // 步长过小，停止迭代
      if (step < 1e-3f)
        return q;
    }
    return q;
  }

  // (预留函数)逆运动学解析解(几何法)
  Matrixf<_n, 1> (*ikine_analytic)(Matrixf<4, 4> T);

  /**
   * @brief 逆动力学求解，牛顿-欧拉法
   * @param q 关节位置向量
   * @param qv 关节速度向量(dq/dt)
   * @param qa 关节加速度向量(d²q/dt²)
   * @param he 末端执行器负载 [力f;力矩μ]，默认无负载
   * @return 各关节需要的驱动力/力矩
   */
  Matrixf<_n, 1> rne(Matrixf<_n, 1> q,
                     Matrixf<_n, 1> qv = matrixf::zeros<_n, 1>(),
                     Matrixf<_n, 1> qa = matrixf::zeros<_n, 1>(),
                     Matrixf<6, 1> he = matrixf::zeros<6, 1>()) {
    // 前向递推：记录各连杆运动状态
    Matrixf<3, _n + 1> w = matrixf::zeros<3, _n + 1>();   // [ωi] 角速度
    Matrixf<3, _n + 1> b = matrixf::zeros<3, _n + 1>();   // [βi] 角加速度
    Matrixf<3, _n + 1> p = matrixf::zeros<3, _n + 1>();   // [pi] 关节位置
    Matrixf<3, _n + 1> v = matrixf::zeros<3, _n + 1>();   // [vi] 关节速度
    Matrixf<3, _n + 1> a = matrixf::zeros<3, _n + 1>();   // [ai] 关节加速度
    Matrixf<3, _n + 1> ac = matrixf::zeros<3, _n + 1>();  // [aci] 质心加速度
    
    // 临时变量
    Matrixf<3, 1> w_i, b_i, p_i, v_i, ai, ac_i;
    // 齐次变换矩阵和旋转矩阵(基坐标系)
    Matrixf<4, 4> T_0i = matrixf::eye<4, 4>();        // T_i^0
    Matrixf<4, 4> T_0iminus1 = matrixf::eye<4, 4>();  // T_i-1^0
    Matrixf<3, 3> R_0i = matrixf::eye<3, 3>();        // R_i^0
    Matrixf<3, 3> R_0iminus1 = matrixf::eye<3, 3>();  // R_i-1^0
    // z轴单位向量
    Matrixf<3, 1> ez = matrixf::zeros<3, 1>();
    ez[2][0] = 1;

    // 前向递推：从基坐标系到末端执行器
    for (int i = 1; i <= _n; i++) {
      T_0i = T_0i * T(q, i - 1);     // 计算第i个连杆的齐次矩阵T_i^0
      R_0i = t2r(T_0i);              // 提取旋转矩阵R_i^0
      R_0iminus1 = t2r(T_0iminus1);  // 提取旋转矩阵R_i-1^0
      
      // 角速度递推：ω_i = ω_i-1 + qv_i * R_i-1^0 * ez
      w_i = w.col(i - 1) + qv[i - 1][0] * R_0iminus1 * ez;
      // 角加速度递推：β_i = β_i-1 + ω_i-1 × (qv_i*R_i-1^0*ez) + qa_i*R_i-1^0*ez
      b_i = b.col(i - 1) +
            vector3f::cross(w.col(i - 1), qv[i - 1][0] * R_0iminus1 * ez) +
            qa[i - 1][0] * R_0iminus1 * ez;
      // 关节位置
      p_i = t2p(T_0i);  // p_i = T_i^0(1:3,4)
      // 线速度递推：v_i = v_i-1 + ω_i × (p_i - p_i-1)
      v_i = v.col(i - 1) + vector3f::cross(w_i, p_i - p.col(i - 1));
      // 线加速度递推：a_i = a_i-1 + β_i × (p_i - p_i-1) + ω_i × (ω_i × (p_i - p_i-1))
      ai = a.col(i - 1) + vector3f::cross(b_i, p_i - p.col(i - 1)) +
           vector3f::cross(w_i, vector3f::cross(w_i, p_i - p.col(i - 1)));
      // 质心加速度递推：ac_i = a_i + β_i × (R_0^i*rc_i^i) + ω_i × (ω_i × (R_0^i*rc_i^i))
      ac_i =
          ai + vector3f::cross(b_i, R_0i * links_[i - 1].rc()) +
          vector3f::cross(w_i, vector3f::cross(w_i, R_0i * links_[i - 1].rc()));
      
      // 存储递推结果
      for (int row = 0; row < 3; row++) {
        w[row][i] = w_i[row][0];
        b[row][i] = b_i[row][0];
        p[row][i] = p_i[row][0];
        v[row][i] = v_i[row][0];
        a[row][i] = ai[row][0];
        ac[row][i] = ac_i[row][0];
      }
      T_0iminus1 = T_0i;  // 更新T_i-1^0为当前T_i^0
    }

    // 反向递推：计算关节力/力矩
    Matrixf<3, _n + 1> f = matrixf::zeros<3, _n + 1>();   // 关节力
    Matrixf<3, _n + 1> mu = matrixf::zeros<3, _n + 1>();  // 关节力矩
    // 临时变量
    Matrixf<3, 1> f_iminus1, mu_iminus1;
    // 齐次变换矩阵和旋转矩阵(i相对于i-1)
    Matrixf<4, 4> T_iminus1i;
    Matrixf<3, 3> RT_iminus1i;
    Matrixf<3, 1> P_iminus1i;
    // 连杆惯性张量(基坐标系)
    Matrixf<3, 3> I_i;
    // 关节力矩
    Matrixf<_n, 1> torq;

    // 末端执行器负载初始化
    for (int row = 0; row < 3; row++) {
      f[row][_n] = he.block<3, 1>(0, 0)[row][0];    // 末端受力
      mu[row][_n] = he.block<3, 1>(3, 0)[row][0];   // 末端受力矩
    }
    
    // 反向递推：从末端执行器到基坐标系
    for (int i = _n; i > 0; i--) {
      T_iminus1i = T(q, i - 1);               // T_i^i-1
      P_iminus1i = t2p(T_iminus1i);           // P_i^i-1
      RT_iminus1i = t2r(T_iminus1i).trans();  // R_i^i-1的转置
      R_0iminus1 = R_0i * RT_iminus1i;        // R_i-1^0
      
      // 连杆惯性张量(转换到基坐标系)：I_i^0 = R_i^0 * I_i^i * (R_i^0)'
      I_i = R_0i * links_[i - 1].I() * R_0i.trans();
      
      // 力递推：f_i-1 = f_i + m_i * ac_i - m_i * g
      f_iminus1 = f.col(i) + links_[i - 1].m() * ac.col(i) -
                  links_[i - 1].m() * gravity_;
      // 力矩递推：μ_i-1 = μ_i + f_i × rc_i - f_i-1 × rc_i-1 + I_i×β_i + ω_i × (I_i×ω_i)
      mu_iminus1 = mu.col(i) +
                   vector3f::cross(f.col(i), R_0i * links_[i - 1].rc()) -
                   vector3f::cross(f_iminus1, R_0i * (RT_iminus1i * P_iminus1i +
                                                      links_[i - 1].rc())) +
                   I_i * b.col(i) + vector3f::cross(w.col(i), I_i * w.col(i));
      // 关节力矩计算：τ_i = μ_i-1 · (R_i-1^0 × ez)
      torq[i - 1][0] = (mu_iminus1.trans() * R_0iminus1 * ez)[0][0];
      
      // 存储力和力矩
      for (int row = 0; row < 3; row++) {
        f[row][i - 1] = f_iminus1[row][0];
        mu[row][i - 1] = mu_iminus1[row][0];
      }
      R_0i = R_0iminus1;  // 更新R_i^0为R_i-1^0
    }

    return torq;
  }

 private:
  Link links_[_n];          // 连杆数组
  Matrixf<3, 1> gravity_;   // 重力加速度向量

  Matrixf<4, 4> T_;         // 临时齐次变换矩阵
  Matrixf<6, _n> J_;        // 临时雅可比矩阵
};
};  // namespace robotics

#endif  // ROBOTICS_H
