# CAlgo_Kf 动态卡尔曼滤波器使用说明

## 简介
本模块实现了支持动态观测量调整的多维卡尔曼滤波器，适用于多传感器异步采样、观测量随时变化等场景。

---

## 1. 初始化参数结构体

```cpp
my_engineer::CAlgo_Kf::SAlgoKfInitParam kf_param;
kf_param.memsDevID = ...; // 传感器ID
kf_param.DT = ...;        // 调度周期（秒）
kf_param.x_size = ...;    // 状态量维度
kf_param.u_size = ...;    // 输入量维度
kf_param.z_size = ...;    // 最大观测量维度

// 动态调整相关参数
kf_param.use_auto_adjustment = true;
kf_param.measurement_map = {1, 2, 3};         // 观测量对应状态量索引（1-based）
kf_param.measurement_degree = {1.0, 1.0, 1.0}; // H矩阵缩放
kf_param.r_diagonal_elements = {30, 25, 35};   // R矩阵对角线
kf_param.state_min_variance = {0.03, 0.005, 0.1}; // P限幅
```

---

## 2. 创建滤波器对象并初始化

```cpp
my_engineer::CAlgo_Kf kf(kf_param);
```

---

## 3. 更新观测向量（每次有新观测数据时）

```cpp
Matrixt<float> measured_vector(kf_param.z_size, 1);
measured_vector[0][0] = ...; // 观测1
measured_vector[1][0] = ...; // 观测2
measured_vector[2][0] = ...; // 观测3
// ...根据实际观测量填充

kf.Set_Measured_Vector(measured_vector);
```

---

## 4. 滤波器主循环调用

```cpp
kf.UpdateHandler_();
```

---

## 5. 获取滤波结果

```cpp
auto result = kf.Kf_Info.filtered_value;
// result 是一个状态向量
```

---

## 6. 可选：设置控制输入

```cpp
Matrixt<float> u(kf_param.u_size, 1);
// 填充u
kf.Set_u(u);
```

---

## 7. 典型流程示意

```cpp
// 1. 初始化参数和滤波器对象
my_engineer::CAlgo_Kf kf(kf_param);

while (1) {
    // 2. 采集观测数据
    Matrixt<float> measured_vector(kf_param.z_size, 1);
    // ...采集并填充measured_vector...
    kf.Set_Measured_Vector(measured_vector);

    // 3. 可选：设置控制输入
    // Matrixt<float> u(...);
    // kf.Set_u(u);

    // 4. 调用滤波器更新
    kf.UpdateHandler_();

    // 5. 获取滤波结果
    auto result = kf.Kf_Info.filtered_value;
    // ...使用result...
}
```

---

## 8. 注意事项
- measurement_map、measurement_degree、r_diagonal_elements 的长度需等于 z_size。
- state_min_variance 的长度需等于 x_size。
- 观测量无效时请赋值为0，滤波器会自动忽略。
- 支持观测量数量和内容动态变化。

---

## 9. 典型应用场景
- 多传感器异步采样
- 观测量丢失/恢复
- 需要防止P矩阵过度收敛的场合

---

如需更详细示例或有特殊需求，请查阅源码或联系维护者。
