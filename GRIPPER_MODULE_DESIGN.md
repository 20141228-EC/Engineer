# 夹爪模块设计方案

> **工程机器人末端夹爪模块完整设计文档**
> 基于 RM2006 电机的夹爪控制系统
> 设计时间：2025-11-26

---

## 📋 目录

1. [需求分析](#1-需求分析)
2. [系统架构设计](#2-系统架构设计)
3. [硬件接口设计](#3-硬件接口设计)
4. [软件模块设计](#4-软件模块设计)
5. [编码器绝对定位方案](#5-编码器绝对定位方案)
6. [限位标定流程](#6-限位标定流程)
7. [遥控器接口设计](#7-遥控器接口设计)
8. [UI 显示设计](#8-ui-显示设计)
9. [实现步骤](#9-实现步骤)
10. [测试验证方案](#10-测试验证方案)

---

## 1. 需求分析

### 1.1 功能需求

| 序号 | 功能模块 | 具体需求 | 优先级 |
|------|---------|---------|--------|
| F1 | 电机控制 | 基于 RM2006 电机实现夹爪开合控制 | P0 |
| F2 | 绝对定位 | 利用编码器实现夹爪位置的绝对定位 | P0 |
| F3 | 限位检测 | 通过实际测量获取并保存夹爪的最大/最小开度限位 | P0 |
| F4 | 遥控器控制 | 支持 DR16 遥控器输入，实现手动控制 | P1 |
| F5 | UI 显示 | 在裁判系统 UI 上显示夹爪开度和抓取状态 | P1 |
| F6 | 抓取检测 | 根据电流/位置判断是否成功抓取物体 | P1 |
| F7 | 安全保护 | 堵转保护、限位保护、过流保护 | P0 |

### 1.2 性能需求

- **响应速度**：遥控器输入到夹爪动作 < 100ms
- **定位精度**：±1° (RM2006 编码器分辨率 8192，减速比考虑后精度足够)
- **控制频率**：电机控制循环 ≥ 500Hz
- **UI 刷新率**：≥ 10Hz (符合裁判系统限制)

### 1.3 约束条件

- 必须使用项目现有架构（设备层 Device、模块层 Module、系统层 System）
- 遵循现有编码规范和命名约定
- 使用 C++ 面向对象设计
- 集成到现有的 `CModArm` 机械臂模块中

---

## 2. 系统架构设计

### 2.1 模块层级关系

```
┌─────────────────────────────────────────────────────────┐
│                   Application Layer                      │
│                  (用户任务 / 状态机)                      │
└───────────────────────┬─────────────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────────────┐
│                    Module Layer                          │
│            CModArm (机械臂模块 - 扩展)                   │
│  ┌──────────────────────────────────────────────────┐   │
│  │  CModGripper (夹爪子模块)                         │   │
│  │  - 位置控制                                       │   │
│  │  - 抓取检测                                       │   │
│  │  - 状态管理                                       │   │
│  └──────────────────┬───────────────────────────────┘   │
└─────────────────────┼───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│                   Device Layer                           │
│         CDevMtrM2006 (RM2006 电机设备)                  │
│         - 编码器读取                                     │
│         - 电流/速度/位置控制                             │
│         - 堵转检测                                       │
└─────────────────────┬───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│                 Interface Layer                          │
│              CInfCAN (CAN 通信接口)                      │
└─────────────────────────────────────────────────────────┘
```

### 2.2 数据流设计

```
遥控器输入 ──┐
             ├──> 模块命令处理 ──> 位置/速度控制 ──> 电机输出
自动控制 ────┘                       │
                                     ├──> 抓取状态判断
编码器反馈 ───────────────────────────┤
电流反馈 ─────────────────────────────┤
                                     │
                                     └──> UI 显示更新
```

---

## 3. 硬件接口设计

### 3.1 RM2006 电机参数

| 参数 | 数值 | 说明 |
|------|------|------|
| **额定电压** | 24V | DC 供电 |
| **最大电流** | 10A | 堵转电流约 8A |
| **编码器线数** | 8192 | 单圈分辨率 |
| **减速比** | 1:36 | 输出轴转速 = 电机转速 / 36 |
| **最大转速** | 469 RPM (输出轴) | 无负载时 |
| **CAN ID** | 可配置 (建议 0x205) | 与机械臂其他电机区分 |

### 3.2 夹爪机械参数（需实测）

| 参数 | 预估值 | 实测值（待填） | 说明 |
|------|-------|---------------|------|
| **最大开度** | ~90° | _________° | 完全张开时的角度 |
| **最小开度** | ~0° | _________° | 完全闭合时的角度 |
| **抓取开度** | ~30° | _________° | 抓取矿石时的典型开度 |
| **机械死区** | ±2° | _________° | 齿轮间隙等因素 |

### 3.3 CAN 总线配置

- **CAN 通道**：CAN1 或 CAN2 (与机械臂共用，需确认)
- **CAN ID**：0x205 (建议，避免与现有设备冲突)
- **波特率**：1Mbps (标准 DJI 电机配置)
- **发送频率**：500Hz (与控制循环同步)

---

## 4. 软件模块设计

### 4.1 夹爪设备类（复用现有）

**类名**：`CDevMtrM2006` (已存在)

**位置**：`Applications/03_Device/Include/mtr/mtr_m2006.hpp`

**初始化参数**：
```cpp
CDevMtrM2006::SMtrInitParam_M2006 gripperMotorParam = {
    .motorName          = "GripperMotor",
    .djiMtrID           = CDevMtrDJI::EDjiMtrID::ID_5,  // CAN ID 0x205
    .encoderResolution  = 8192,
    .useAngleToPosit    = true,   // 启用角度转位置
    .useStallMonit      = true,   // 启用堵转检测
    .stallMonitDataSrc  = DATA_CURRENT,
    .stallThreshold     = 5000,   // 堵转电流阈值 (mA)
    .stallTime          = 200,    // 堵转判定时间 (ms)
};
```

### 4.2 夹爪控制子模块设计

**方案选择**：建议作为 `CModArm` 的**扩展**，而非独立模块

#### 4.2.1 扩展 `CModArm` 类

**文件**：`Applications/04_Module/Include/mod_arm.hpp`

**新增成员变量**：
```cpp
class CModArm {
public:
    // ========== 原有机械臂关节 ==========
    // ... (保持不变)

    // ========== 新增：夹爪控制 ==========

    /**
     * @brief 夹爪状态枚举
     */
    enum class EGripperState {
        IDLE = 0,      ///< 空闲
        OPENING,       ///< 正在打开
        CLOSING,       ///< 正在关闭
        OPENED,        ///< 已打开（到达限位）
        CLOSED,        ///< 已关闭（到达限位）
        GRIPPING,      ///< 正在抓取（检测到阻力）
        GRIPPED,       ///< 已抓取（稳定抓住）
        ERROR,         ///< 错误（堵转/超时等）
    };

    /**
     * @brief 夹爪信息结构体
     */
    struct SGripperInfo {
        float_t currentAngle   = 0.0f;        ///< 当前开度 (度)
        float_t targetAngle    = 0.0f;        ///< 目标开度 (度)
        float_t openLimit      = 90.0f;       ///< 最大开度限位
        float_t closeLimit     = 0.0f;        ///< 最小开度限位
        int16_t currentTorque  = 0;           ///< 当前电流 (mA)
        EGripperState state    = EGripperState::IDLE;
        bool isGripped         = false;       ///< 是否抓住物体
        bool isCalibrated      = false;       ///< 是否已标定限位
    } gripperInfo;

    /**
     * @brief 夹爪控制命令结构体
     */
    struct SGripperCmd {
        bool enable         = false;       ///< 使能夹爪控制
        float_t setAngle    = 0.0f;        ///< 设定开度
        bool cmdOpen        = false;       ///< 命令：打开
        bool cmdClose       = false;       ///< 命令：关闭
        bool cmdGrip        = false;       ///< 命令：抓取（自适应力控）
        bool cmdRelease     = false;       ///< 命令：释放
        bool cmdCalibrate   = false;       ///< 命令：执行限位标定
    } gripperCmd;

private:
    CDevMtrM2006 *pMtrGripper_ = nullptr;  ///< 夹爪电机指针
    CInfCAN::CCanTxNode *pGripperTxNode_ = nullptr;

    CAlgoPid gripperPosPid_;               ///< 位置 PID
    CAlgoPid gripperSpdPid_;               ///< 速度 PID

    // 抓取检测参数
    static constexpr int16_t GRIP_CURRENT_THRESHOLD = 3000;  ///< 抓取电流阈值 (mA)
    static constexpr float_t GRIP_ANGLE_TOLERANCE   = 2.0f;  ///< 角度误差容忍 (度)

    /**
     * @brief 更新夹爪状态（在 UpdateHandler_() 中调用）
     */
    void UpdateGripperState_();

    /**
     * @brief 执行夹爪控制（在 UpdateHandler_() 中调用）
     */
    void ControlGripper_();

    /**
     * @brief 夹爪限位标定流程
     */
    void CalibrateGripperLimits_();

    /**
     * @brief 检测是否抓住物体
     * @return true - 已抓住，false - 未抓住
     */
    bool DetectGripStatus_();
};
```

#### 4.2.2 初始化参数扩展

在 `SModInitParam_Arm` 中添加：
```cpp
struct SModInitParam_Arm: public SModInitParam_Base {
    // ... 原有参数 ...

    // 新增：夹爪相关
    EDeviceID MotorID_Gripper = EDeviceID::DEV_NULL;
    CInfCAN::CCanTxNode *MotorTxNode_Gripper = nullptr;
    CAlgoPid::SAlgoInitParam_Pid gripperPosPidParam;
    CAlgoPid::SAlgoInitParam_Pid gripperSpdPidParam;
};
```

---

## 5. 编码器绝对定位方案

### 5.1 定位原理

RM2006 电机内置 **8192 线增量式编码器**，配合减速比 1:36，需要实现：
- **圈数累计**：检测编码器过零点，累计圈数
- **绝对位置计算**：`总位置 = 圈数 × 8192 + 当前编码值`
- **角度转换**：`物理角度 = (总位置 / 8192) × (360° / 36) × 传动比`

### 5.2 位置计算公式

假设夹爪机构的**机械传动比**为 `R`（待测量），则：

```cpp
// 1. 读取编码器原始值
int16_t rawAngle = motorData[DATA_ANGLE];  // 0 ~ 8191

// 2. 圈数累计（已由 CDevMtrDJI 基类实现）
int32_t totalPosition = motorData[DATA_POSIT];  // 累计位置

// 3. 转换为电机轴角度
float motorAngle = (float)totalPosition / 8192.0f * 360.0f;

// 4. 转换为夹爪物理开度
float gripperAngle = motorAngle / GRIPPER_TRANSMISSION_RATIO;
```

### 5.3 传动比标定

**方法1：理论计算**
- 查看机械图纸，计算齿轮/丝杠传动比
- 例如：电机轴齿轮 20 齿，夹爪轴齿轮 60 齿 → 传动比 = 3:1

**方法2：实测标定**
1. 手动将夹爪从最小开度移动到最大开度
2. 记录编码器位置变化量 `ΔPos`
3. 测量实际角度变化 `ΔAngle`（使用量角器）
4. 计算：`传动比 = (ΔPos / 8192 × 360 / 36) / ΔAngle`

---

## 6. 限位标定流程

### 6.1 标定模式设计

**触发方式**：
- 遥控器组合键：左开关上 + 右开关下 + 按键 'C'
- 或通过上位机串口命令触发

**标定流程状态机**：
```
[IDLE] ──(触发标定)──> [FIND_CLOSE_LIMIT] ──(检测到堵转)──> [RECORD_CLOSE]
                                                                    │
                            ┌───────────────────────────────────────┘
                            │
                            ▼
                     [FIND_OPEN_LIMIT] ──(检测到堵转)──> [RECORD_OPEN]
                            │
                            ▼
                     [CALIBRATION_DONE]
```

### 6.2 限位检测逻辑

**关闭限位检测**：
```cpp
// 以低速驱动夹爪关闭
gripperCmd.setAngle = -999.0f;  // 超出范围，触发速度控制
speedTarget = -50;  // 低速 (RPM)

// 检测堵转条件
if (abs(motorData[DATA_CURRENT]) > STALL_CURRENT_THRESHOLD &&
    abs(motorData[DATA_SPEED]) < 5) {  // 几乎静止

    // 记录限位位置
    gripperInfo.closeLimit = gripperInfo.currentAngle;

    // 进入下一阶段
    calibrationState = FIND_OPEN_LIMIT;
}
```

**打开限位检测**：类似逻辑，反向运动

### 6.3 限位保存

标定完成后，将限位值保存到：
- **FLASH 存储**：掉电保持（需实现 Flash 驱动）
- **临时方案**：编译时硬编码到代码中

```cpp
// 保存到 Flash (伪代码)
struct GripperCalibData {
    float openLimit;
    float closeLimit;
    uint32_t crc;  // 校验和
};

void SaveCalibrationToFlash(const GripperCalibData& data);
void LoadCalibrationFromFlash(GripperCalibData& data);
```

---

## 7. 遥控器接口设计

### 7.1 控制映射方案

基于现有 DR16 遥控器，建议映射：

| 输入 | 功能 | 说明 |
|------|------|------|
| **左摇杆 CH_0** | 夹爪速度控制 | 推上 = 打开，拉下 = 关闭 |
| **按键 'G'** | 快速抓取 | 按下后夹爪快速关闭至检测到物体 |
| **按键 'R'** | 快速释放 | 按下后夹爪快速打开至最大开度 |
| **按键 'C' (长按)** | 进入标定模式 | 需配合左开关上 + 右开关下 |

### 7.2 控制代码实现

```cpp
void CModArm::ProcessRemoteControl_() {
    // 获取遥控器数据
    auto& rc = pDevRemote_->remoteData;

    // 夹爪速度控制（摇杆）
    if (abs(rc.channel[CRcDR16::CH_0]) > 50) {  // 死区
        float speedCmd = rc.channel[CRcDR16::CH_0] / 660.0f * MAX_GRIPPER_SPEED;
        gripperCmd.setAngle += speedCmd * deltaTime;  // 积分得位置
        gripperCmd.enable = true;
    }

    // 快速抓取（按键 G）
    if (rc.channel[CRcDR16::CH_KEY_G] == ERcChannelStatus::PRESS) {
        gripperCmd.cmdGrip = true;
    }

    // 快速释放（按键 R）
    if (rc.channel[CRcDR16::CH_KEY_R] == ERcChannelStatus::PRESS) {
        gripperCmd.cmdRelease = true;
    }

    // 标定模式触发
    if (rc.channel[CRcDR16::CH_SW1] == ERcChannelStatus::HIGH &&
        rc.channel[CRcDR16::CH_SW2] == ERcChannelStatus::DOWN &&
        rc.channel[CRcDR16::CH_KEY_C] == ERcChannelStatus::PRESS) {
        gripperCmd.cmdCalibrate = true;
    }
}
```

---

## 8. UI 显示设计

### 8.1 UI 元素规划

在裁判系统客户端显示以下信息：

| UI 元素 | 类型 | 位置 | 内容 |
|---------|------|------|------|
| **夹爪开度文本** | 文本 | 屏幕右下角 | "Gripper: 45.2°" |
| **抓取状态指示** | 圆形 | 文本旁边 | 绿色 = 已抓取，红色 = 空载 |
| **开度进度条** | 线段 | 文本下方 | 视觉化显示当前开度 |

### 8.2 UI 配置代码

在 `CSystemReferee` 中添加：

```cpp
// 1. 定义 UI 配置 ID
enum EUiConfigID {
    // ... 原有 UI ...
    TEXT_GRIPPER_ANGLE,     ///< 夹爪角度文本
    CIRCLE_GRIPPER_STATUS,  ///< 抓取状态圆形指示
    LINE_GRIPPER_BAR,       ///< 开度进度条
};

// 2. 初始化 UI 元素
void CSystemReferee::UI_InitGripperDrawing() {
    // 夹爪角度文本
    uiConfig[TEXT_GRIPPER_ANGLE] = {
        .name       = "grip_txt",
        .operation  = CDevReferee::EGraphicOperation::ADD,
        .layer      = 5,
        .color      = CDevReferee::EGraphicColor::CYAN,
        .startX     = 1600,
        .startY     = 200,
        .fontSize   = 20,
    };

    // 抓取状态圆形
    uiConfig[CIRCLE_GRIPPER_STATUS] = {
        .name       = "grip_st",
        .operation  = CDevReferee::EGraphicOperation::ADD,
        .layer      = 5,
        .color      = CDevReferee::EGraphicColor::GREEN,  // 动态变化
        .startX     = 1750,
        .startY     = 200,
        .radius     = 10,
    };

    // 开度进度条
    uiConfig[LINE_GRIPPER_BAR] = {
        .name       = "grip_bar",
        .operation  = CDevReferee::EGraphicOperation::ADD,
        .layer      = 5,
        .color      = CDevReferee::EGraphicColor::ORANGE,
        .startX     = 1600,
        .startY     = 180,
        .endX       = 1600 + (int)(gripperAngle / 90.0f * 150),  // 动态
        .endY       = 180,
        .lineWidth  = 5,
    };
}

// 3. 动态更新 UI
void CSystemReferee::UI_UpdateGripperStatus() {
    // 获取夹爪状态
    auto& gripper = pModArm_->gripperInfo;

    // 更新文本
    char angleText[30];
    snprintf(angleText, sizeof(angleText), "Grip:%.1f", gripper.currentAngle);
    // ... 发送文本更新消息 ...

    // 更新状态圆形颜色
    if (gripper.isGripped) {
        uiConfig[CIRCLE_GRIPPER_STATUS].color = CDevReferee::EGraphicColor::GREEN;
    } else {
        uiConfig[CIRCLE_GRIPPER_STATUS].color = CDevReferee::EGraphicColor::RED;
    }

    // 更新进度条长度
    int barLength = (int)(gripper.currentAngle / gripper.openLimit * 150);
    uiConfig[LINE_GRIPPER_BAR].endX = 1600 + barLength;

    // 发送 UI 更新包
    // ... (参考现有 UI_StartStateFigureDrawing_() 实现)
}
```

### 8.3 UI 刷新策略

- **刷新频率**：10Hz (受裁判系统带宽限制)
- **触发条件**：
  - 夹爪角度变化 > 1°
  - 抓取状态改变
  - 定时刷新（每 100ms）

---

## 9. 实现步骤

### 9.1 第一阶段：基础电机控制 (P0)

**目标**：实现夹爪电机的基本控制和位置反馈

**任务清单**：
- [ ] 在 `Configuration.hpp` 中添加夹爪电机 ID 定义
- [ ] 在 `mod_arm.hpp/cpp` 中添加夹爪相关成员变量
- [ ] 初始化 `CDevMtrM2006` 电机实例
- [ ] 配置 CAN 发送节点
- [ ] 实现基础位置读取和速度控制
- [ ] 使用 RTT 调试输出验证编码器数据

**验证方式**：
```cpp
// 在 UpdateHandler_() 中添加调试输出
RTT_LOG_INFO("Gripper Angle: %.2f, Current: %d\n",
             gripperInfo.currentAngle,
             pMtrGripper_->motorData[DATA_CURRENT]);
```

### 9.2 第二阶段：限位标定 (P0)

**目标**：实现夹爪限位的自动标定

**任务清单**：
- [ ] 实现 `CalibrateGripperLimits_()` 状态机
- [ ] 添加堵转检测逻辑
- [ ] 实现限位值保存（先用硬编码，后续加 Flash）
- [ ] 添加遥控器触发标定的接口
- [ ] 测试并记录实际限位值

**测试步骤**：
1. 手动将夹爪移到中间位置
2. 触发标定命令
3. 观察夹爪自动关闭至限位
4. 观察夹爪自动打开至限位
5. 验证限位值是否合理

### 9.3 第三阶段：位置控制和抓取检测 (P0)

**目标**：实现精确的位置控制和物体抓取判断

**任务清单**：
- [ ] 配置位置/速度 PID 参数
- [ ] 实现 `ControlGripper_()` 控制逻辑
- [ ] 实现 `DetectGripStatus_()` 抓取检测
- [ ] 添加限位保护（防止超出范围）
- [ ] 调试 PID 参数，优化响应速度和稳定性

**PID 参数建议**（需调试）：
```cpp
// 位置环
gripperPosPidParam.kp = 5.0f;
gripperPosPidParam.ki = 0.1f;
gripperPosPidParam.kd = 0.5f;
gripperPosPidParam.maxOutput = 500.0f;  // 限制速度输出

// 速度环
gripperSpdPidParam.kp = 8.0f;
gripperSpdPidParam.ki = 0.5f;
gripperSpdPidParam.kd = 0.0f;
gripperSpdPidParam.maxOutput = 10000.0f;  // 电流限制
```

### 9.4 第四阶段：遥控器接口 (P1)

**目标**：实现遥控器手动控制夹爪

**任务清单**：
- [ ] 在 `ProcessRemoteControl_()` 中添加夹爪控制映射
- [ ] 实现摇杆速度控制
- [ ] 实现快捷键抓取/释放功能
- [ ] 添加死区和滤波处理
- [ ] 测试操作手感，优化控制曲线

### 9.5 第五阶段：UI 显示 (P1)

**目标**：在裁判系统客户端显示夹爪状态

**任务清单**：
- [ ] 在 `CSystemReferee` 中添加夹爪 UI 元素定义
- [ ] 实现 `UI_InitGripperDrawing()` 初始化函数
- [ ] 实现 `UI_UpdateGripperStatus()` 更新函数
- [ ] 在系统初始化时调用 UI 初始化
- [ ] 在更新循环中定时刷新 UI
- [ ] 验证 UI 显示效果

### 9.6 第六阶段：集成测试与优化 (P2)

**任务清单**：
- [ ] 与机械臂其他关节联调
- [ ] 测试完整抓取流程（移动 → 抓取 → 搬运 → 释放）
- [ ] 性能优化（减少 CPU 占用、优化控制算法）
- [ ] 安全性测试（异常情况处理）
- [ ] 编写用户手册和维护文档

---

## 10. 测试验证方案

### 10.1 单元测试

| 测试项 | 测试方法 | 通过标准 |
|--------|---------|---------|
| **编码器读取** | 手动转动电机，观察角度变化 | 角度连续且方向正确 |
| **限位标定** | 执行标定流程 3 次 | 每次限位值误差 < 1° |
| **位置控制** | 设定目标角度，检查到达精度 | 稳态误差 < 2° |
| **堵转保护** | 阻止夹爪运动，触发堵转 | 2s 内停止输出 |
| **抓取检测** | 夹持物体后检测状态 | 100% 识别成功 |

### 10.2 集成测试

**场景1：自动抓取矿石**
1. 机械臂移动到矿石上方
2. 发送抓取命令
3. 夹爪下降并自适应抓取
4. 检测抓取成功
5. 提升矿石

**场景2：遥控器手动控制**
1. 操作手使用摇杆控制夹爪
2. 验证响应速度 < 100ms
3. 验证操作平滑性

**场景3：异常恢复**
1. 夹爪运动中突然掉电
2. 重启后重新初始化
3. 验证是否能恢复正常工作

### 10.3 性能指标

| 指标 | 目标值 | 实测值 |
|------|--------|--------|
| 最大开度 | ≥ 80° | ______ |
| 最小开度 | ≤ 5° | ______ |
| 全程运动时间 | ≤ 1s | ______ |
| 抓取成功率 | ≥ 95% | ______ |
| 定位精度 | ≤ 2° | ______ |
| 控制延迟 | ≤ 100ms | ______ |

---

## 11. 附录

### 11.1 关键代码框架

#### A. 夹爪控制主循环

```cpp
void CModArm::ControlGripper_() {
    // 1. 读取当前状态
    float currentAngle = gripperInfo.currentAngle;
    float targetAngle  = gripperCmd.setAngle;

    // 2. 限位保护
    targetAngle = constrain(targetAngle,
                           gripperInfo.closeLimit,
                           gripperInfo.openLimit);

    // 3. 位置环 PID
    float speedTarget = gripperPosPid_.Calculate(targetAngle, currentAngle);

    // 4. 速度环 PID
    float currentSpeed = pMtrGripper_->motorData[DATA_SPEED];
    float currentOutput = gripperSpdPid_.Calculate(speedTarget, currentSpeed);

    // 5. 输出到电机
    pGripperTxNode_->dataBuffer[0] = (int16_t)(currentOutput) >> 8;
    pGripperTxNode_->dataBuffer[1] = (int16_t)(currentOutput) & 0xFF;
    pGripperTxNode_->needUpdate = true;

    // 6. 抓取状态检测
    gripperInfo.isGripped = DetectGripStatus_();
}
```

#### B. 抓取检测算法

```cpp
bool CModArm::DetectGripStatus_() {
    // 方法1：电流阈值法
    int16_t current = pMtrGripper_->motorData[DATA_CURRENT];
    float angleError = abs(gripperCmd.setAngle - gripperInfo.currentAngle);

    if (current > GRIP_CURRENT_THRESHOLD &&
        angleError > GRIP_ANGLE_TOLERANCE) {
        // 电流大且未到达目标位置 → 可能抓住了物体
        return true;
    }

    // 方法2：速度误差法（备选）
    // 如果目标速度不为 0，但实际速度很小 → 可能被阻挡

    return false;
}
```

### 11.2 调试技巧

1. **使用 RTT 实时输出**
   ```cpp
   RTT_LOG_INFO("Gripper: Ang=%.1f Tgt=%.1f Cur=%d State=%d\n",
                gripperInfo.currentAngle,
                gripperCmd.setAngle,
                pMtrGripper_->motorData[DATA_CURRENT],
                (int)gripperInfo.state);
   ```

2. **波形记录**
   - 使用 J-Link RTT 配合上位机绘制角度/电流曲线
   - 分析 PID 响应特性

3. **渐进式测试**
   - 先测试速度控制（开环）
   - 再测试位置控制（闭环）
   - 最后测试自适应抓取

### 11.3 常见问题 FAQ

**Q1：夹爪抖动怎么办？**
- A：降低 PID 的 Kp 和 Kd 参数，增加滤波

**Q2：抓取检测误判？**
- A：调整电流阈值，增加延迟判断（连续检测 N 次）

**Q3：限位标定不准？**
- A：检查机械间隙，使用低速标定，多次测量取平均

**Q4：电机堵转后无法恢复？**
- A：添加堵转后的自动回退逻辑

---

## 📌 总结

本设计方案涵盖了夹爪模块的完整实现思路，核心要点：

1. **复用现有架构**：基于 `CDevMtrM2006` 和 `CModArm` 扩展，符合项目规范
2. **绝对定位**：利用编码器圈数累计实现位置跟踪
3. **自动标定**：通过堵转检测自动获取限位
4. **多种控制模式**：支持位置控制、速度控制、自适应抓取
5. **可视化反馈**：通过裁判系统 UI 实时显示状态

**下一步行动**：
- ✅ 审阅本设计文档
- ✅ 确认机械参数（传动比、限位等）
- ✅ 按阶段逐步实现和测试

---

> **文档版本**：V1.0
> **最后更新**：2025-11-26
> **作者**：幽浮喵 (AI Assistant)
> **审核**：待定
