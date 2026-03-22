/**
 * @file mod_chassis.hpp
 * @author sllllr (2997708711@qq.com)
 * @brief 定义底盘模块
 * @version 1.0
 * @date 2025-12-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef MOD_CHASSIS_HPP
#define MOD_CHASSIS_HPP

#include "mod_common.hpp"
#include "algo_ave_filter.hpp"

#define L_LIFT_MOTOR_DIR -1 //-1                 ///< 左腿编码器与腿长增加方向是否一致 一致为1 否则为-1
#define R_LIFT_MOTOR_DIR 1 //1                  ///< 右腿编码器与腿长增加方向是否一致 一致为1 否则为-1
#define ROLL_LIFT_DIR   -1                   ///< roll轴增大方向是否和抬头方向一致 一致为1 否则为-1
#define CHASSIS_HIP_INIT_LENGTH 0.0f        ///< 初始化腿长 后续待改
#define CHASSIS_HIP_INIT_ECD_L  0.3f//15.f//
#define CHASSIS_HIP_INIT_ECD_R  -0.5f//-38.f//      ///< 这两个是左右电机在初始化腿长时候的编码器值  这个得和陀螺仪数据0对应
#define CHASSIS_HIP_PHY_MAX     100.0f
#define CHASSIS_HIP_PHY_MIN     0.0f        ///< 这个是最大和最短腿长  这两个目前还用不到
#define CHASSIS_HIP_ECD_MAX_L   9.3f//1685.f//
#define CHASSIS_HIP_ECD_MIN_L   0.f//0.0f
#define CHASSIS_HIP_ECD_MAX_R   0.f//0.0f
#define CHASSIS_HIP_ECD_MIN_R   -9.4f//-1717.f//        ///< 这几个是极限腿长时候两个电机对应的编码值 即软件限位 待改
#define ECD_LENGTH_RATIO        -1.0f        ///< 这是腿长range和编码器range的线性对应关系，即传动比 这个保持为1就行
#define ROLL_DEG_ECD_RATIO     50.f        ///< 这是roll动一度的时候编码器的变化值，待改
#define G 9.7803f    ///< 南山区的g值

#define deg2rad(x) ((x) * 0.017453292519943295769236907684886)
#define rad2deg(x) ((x) * 57.295779513082320876798154814105)
#define ecd2rad(x) ((x) * 0.0000958251953125) ///< 编码器总值到角度转化 0.0054931640625

#define DM8009P_CURRENT_TO_TORQUE_L 1 //0.1946174202  ///< 1.5 * 9 * 21 * 0.0006864812
#define DM8009P_CURRENT_TO_TORQUE_R 1 //1.1798274915  ///< 1.5 * 9 * 21 * 0.004161649

/* public定义用户层方便调试和获取信息，private定义了底层用于直接驱动电机，而不会因为外界的干扰影响了输出的值 */

namespace my_engineer {

/**
 * @brief 底盘模块类
 * 
 */
class CModChassis final: public CModBase{
public:
    static constexpr uint16_t kDefaultChassisMaxPower = 115;

    // 定义底盘模块初始化参数结构体
    struct SModInitParam_Chassis: public SModInitParam_Base{
        EDeviceID memsDevID = EDeviceID::DEV_NULL;
        EAlgoID FilterID = EAlgoID::ALGO_NULL;
        EDeviceID wheelsetMotorID_LF = EDeviceID::DEV_NULL;
        EDeviceID wheelsetMotorID_RF = EDeviceID::DEV_NULL;
        EDeviceID wheelsetMotorID_LB = EDeviceID::DEV_NULL;
        EDeviceID wheelsetMotorID_RB = EDeviceID::DEV_NULL;
        EDeviceID steerMotorID_LF = EDeviceID::DEV_NULL;
        EDeviceID steerMotorID_RF = EDeviceID::DEV_NULL;
        EDeviceID steerMotorID_LB = EDeviceID::DEV_NULL;
        EDeviceID steerMotorID_RB = EDeviceID::DEV_NULL;
        EDeviceID hipMotorID_L_L = EDeviceID::DEV_NULL;
        EDeviceID hipMotorID_L_R = EDeviceID::DEV_NULL; ///< 后腿电机
        EDeviceID crawlerMotorID_L = EDeviceID::DEV_NULL;
        EDeviceID crawlerMotorID_R = EDeviceID::DEV_NULL;   ///< 履带电机
        CInfCAN::CCanTxNode *wheelsetMotorTxNode_LF;
        CInfCAN::CCanTxNode *wheelsetMotorTxNode_RF;
        CInfCAN::CCanTxNode *wheelsetMotorTxNode_LB;
        CInfCAN::CCanTxNode *wheelsetMotorTxNode_RB;
        CInfCAN::CCanTxNode *steerMotorTxNode_LF = nullptr;
        CInfCAN::CCanTxNode *steerMotorTxNode_RF = nullptr;
        CInfCAN::CCanTxNode *steerMotorTxNode_LB = nullptr;
        CInfCAN::CCanTxNode *steerMotorTxNode_RB = nullptr;
        CInfCAN::CCanTxNode *hipMotorTxNodeID_L_L;
        CInfCAN::CCanTxNode *hipMotorTxNodeID_L_R;
        CInfCAN::CCanTxNode *crawlerMotorTxNodeID_L;
        CInfCAN::CCanTxNode *crawlerMotorTxNodeID_R;
        float_t MIT_L_kp = 0.0f; ///< MIT控制器比例系数
		float_t MIT_L_kd = 0.0f; ///< MIT控制器微分系数
        float_t MIT_L_tau = 0.0f;   ///< MIT控制器前馈扭矩
        float_t MIT_R_kp = 0.0f; ///< MIT控制器比例系数
		float_t MIT_R_kd = 0.0f; ///< MIT控制器微分系数
        float_t MIT_R_tau = 0.0f;   ///< MIT控制器前馈扭矩
        std::array<CAlgoPid::SAlgoInitParam_Pid, 4> wheelsetSpdPidParam;
        CAlgoPid::SAlgoInitParam_Pid lineCorrectionPidParam;
        CAlgoPid::SAlgoInitParam_Pid yawCorrectionPidParam;
        std::array<CAlgoPid::SAlgoInitParam_Pid, 4> steerPosPidParam;
        std::array<CAlgoPid::SAlgoInitParam_Pid, 4> steerSpdPidParam;
        CAlgoPid::SAlgoInitParam_Pid rollCorrectionPidParam; ///< roll轴控制pid
        CAlgoPid::SAlgoInitParam_Pid HipPosPidParam_L;
        CAlgoPid::SAlgoInitParam_Pid HipPosPidParam_R;
        CAlgoPid::SAlgoInitParam_Pid HipSpdPidParam_L;
        CAlgoPid::SAlgoInitParam_Pid HipSpdPidParam_R;
        CAlgoPid::SAlgoInitParam_Pid CrawlerSpdPidParam;    ///< 履带电机用同一套pid
        CAlgoPowerControl::SAlgoInitParamPower powerParamLF;  // 左前电机功率参数
        CAlgoPowerControl::SAlgoInitParamPower powerParamRF;  // 右前电机功率参数
        CAlgoPowerControl::SAlgoInitParamPower powerParamLB;  // 左后电机功率参数
        CAlgoPowerControl::SAlgoInitParamPower powerParamRB;  // 右后电机功率参数
        CAlgoPowerControl::SAlgoInitParamPower steerPowerParamLF; // 左前舵向电机功率参数
        CAlgoPowerControl::SAlgoInitParamPower steerPowerParamRF; // 右前舵向电机功率参数
        CAlgoPowerControl::SAlgoInitParamPower steerPowerParamLB; // 左后舵向电机功率参数
        CAlgoPowerControl::SAlgoInitParamPower steerPowerParamRB; // 右后舵向电机功率参数
        EInterfaceID powerMeterCanID = EInterfaceID::INF_NULL;     // 功率计CAN接口ID
        uint32_t powerMeterStdID = 0;                              // 功率计标准帧ID
        CInfCAN::ECanFrameDlc powerMeterFrameDlc = CInfCAN::ECanFrameDlc::DLC_8; // 功率计帧长度
        uint16_t chassisMaxPower = kDefaultChassisMaxPower;   // 底盘总功率限制
    };

    // 定义底盘信息结构体并实例化
    struct SChassisInfo{
        EVarStatus isModuleAvailable  = false; ///< 模块是否可用                         ///<这里定义了info用于接收用户指令

        // 下面这三个变量由于没有传感器可以直接读取，所以并不会更新
        float_t speed_X = 0.0f; ///< 底盘X轴速度
        float_t speed_Y = 0.0f; ///< 底盘Y轴速度
        float_t speed_W = 0.0f; ///< 底盘角速度
        float_t L_Length = 0.0f; ///< 后腿腿长
        int16_t laser_distance_L = 0; ///< 激光传感器左距离
        int16_t laser_distance_R = 0; ///< 激光传感器右距离
    } chassisInfo;

    // 定义底盘控制命令结构体并实例化
    struct SChassisCmd {
        bool isAutoCtrl = false; ///< 是否自动控制
        float_t speed_X = 0;    ///< 底盘X轴速度(范围-100％~100％)
        float_t speed_Y = 0;    ///< 底盘Y轴速度(范围-100％~100％)
        float_t speed_W = 0;    ///< 底盘角速度(范围-100％~100％)
        float_t L_length = 0.0f; ///< 后腿腿长
        float_t speed_crawler = 0;  ///< 履带电机速度
    } chassisCmd;

    // 互补滤波算法实例指针
    CAlgo_IMU_Ave *filter = nullptr;

    // 整车roll轴角度
    DataBuffer<float_t> roll_Measure;

    // 运动模式
    enum class EmovMode 
    {
        NORMAL = 0, ///< 普通模式(拨轮控腿长)
        CLIMBING,   ///< 上台阶模式(陀螺仪控腿长)
    };

    CModChassis() = default;

    // 定义带参数的模块构造函数，创建模块时自动调用初始化函数
    explicit CModChassis(SModInitParam_Base &param) { InitModule(param); }

    // 模块析构函数,好像重不重定义无所谓，和基类一样
    ~CModChassis() final { UnregisterModule_();}

    // 初始化模块
    EAppStatus InitModule(SModInitParam_Base &param) final;

    // 面向模块的运动模式标志位
    EmovMode MovMode = EmovMode::NORMAL;

    // 复位腿的标志位
    EVarStatus reset_hip = false;

    // 启动履带的标志位
    EVarStatus crawler_on = false;

private:

    static constexpr float kSoftLimitRatio_ = 0.84f;
    static constexpr float kWarnBudgetRatio_ = 95.0f / static_cast<float>(kDefaultChassisMaxPower); // 功率预算警告阈值占比，超过这个占比时会触发警告，但不强制限制输出
    static constexpr float kFloorBudgetRatio_ = 70.0f / static_cast<float>(kDefaultChassisMaxPower);
    static constexpr uint32_t kPowerMeterOfflineTimeoutMs_ = 50U;

    uint16_t chassisMaxPower_ = kDefaultChassisMaxPower; // 底盘总功率限制
    // 底盘电机功率控制实例
    CAlgoPowerControl powerCtrlLF_;  // 左前电机功率控制实例
    CAlgoPowerControl powerCtrlRF_;  // 右前电机功率控制实例
    CAlgoPowerControl powerCtrlLB_;  // 左后电机功率控制实例
    CAlgoPowerControl powerCtrlRB_;  // 右后电机功率控制实例
    CAlgoPowerControl powerCtrlSteerLF_;  // 左前舵向电机功率控制实例
    CAlgoPowerControl powerCtrlSteerRF_;  // 右前舵向电机功率控制实例
    CAlgoPowerControl powerCtrlSteerLB_;  // 左后舵向电机功率控制实例
    CAlgoPowerControl powerCtrlSteerRB_;  // 右后舵向电机功率控制实例
    CInfCAN::CCanRxNode powerMeterRxNode_; // 功率计CAN接收节点
    uint32_t powerMeterLastTimestamp_ = 0; // 功率计最近一次更新时间戳
    float measuredPowerLpf_ = 0.0f;        // 功率计反馈低通值
    float lastMeasuredPower_ = 0.0f;       // 上一拍功率计值
    float feedbackMeasuredPower_ = 0.0f;   // 基于电机反馈电流估算的实际总功率(W)
    float feedbackMeasuredPowerRaw_ = 0.0f;// 基于反馈电流估算的瞬时总功率(W)
    float terrainOverloadPenalty_ = 0.0f;  // 陡坡/突增负载下的附加收紧量(W)
    float dynamicPowerBudget_ = 0.0f;      // 动态硬功率预算(W)
    float softPowerBudget_ = 0.0f;         // 软限功预算(W)
    bool powerBudgetInitialized_ = false;  // 功率预算状态是否已完成初始化

    // 定义底盘轮组组件类并实例化
    class CComWheelset: public CComponentBase{
    public:
        enum {LF = 0, RF = 1, LB = 2, RB = 3};

        // 定义底盘轮组信息结构体并实例化
        struct SWheelsetInfo {
            float_t speed_LF = 0.0f;    ///< Chassis Speed LF (Unit: rpm)
            float_t speed_RF = 0.0f;    ///< Chassis Speed RF (Unit: rpm)
            float_t speed_LB = 0.0f;    ///< Chassis Speed LB (Unit: rpm)
            float_t speed_RB = 0.0f;    ///< Chassis Speed RB (Unit: rpm)                   ///<这里定义了组件用于底层驱动
        } wheelsetInfo;

        // 定义底盘轮组控制命令结构体并实例化
        struct SWheelsetCommand {
            float_t speed_X = 0.0f;    ///< Chassis Speed X (Range: -100% ~ 100%)
            float_t speed_Y = 0.0f;    ///< Chassis Speed Y (Range: -100% ~ 100%)
            float_t speed_W = 0.0f;    ///< Chassis Speed W (Range: -100% ~ 100%)    
        } wheelsetCmd;

        // 传感器实例指针
        CMemsBase *mems = nullptr;

        // 电机实例指针数组
        CDevMtr *motor[4] = {nullptr};
        CDevMtr *steerMotor[4] = {nullptr};

        // 定义底盘PID控制器
        CAlgoPid pidYawCtrl;                    ///<控制底盘角速度（Yaw旋转）
        CAlgoPid pidLineCorrectionCtrl;         ///<修正X、Y、W三个方向的误差
        std::array<CAlgoPid, 4> pidSpdCtrl;     ///<控制4个轮子的速度
        std::array<CAlgoPid, 4> pidSteerPosCtrl;///<控制4个舵向电机的位置
        std::array<CAlgoPid, 4> pidSteerSpdCtrl;///<控制4个舵向电机的速度
        // 电机数据输出缓冲区
        std::array<int16_t, 4> mtrOutputBuffer = {0};
        std::array<int16_t, 4> mtrSteerOutputBuffer = {0};
        std::array<float, 4> steerErrRad = {0.0f, 0.0f, 0.0f, 0.0f};

        // 初始化组件
        EAppStatus InitComponent(SModInitParam_Base &param) final;

        // 重写组件更新函数
        EAppStatus UpdateComponent() final;

        // 声明组件输出更新函数(负责根据控制量进行解算，以及进行PID运算，最后得到输出值)
        EAppStatus _UpdateOutput(float speed_X, float speed_Y, float speed_W);
    
        // 电机can发送节点
        std::array<CInfCAN::CCanTxNode*, 4> mtrCanTxNode = {nullptr, nullptr, nullptr, nullptr};
        std::array<CInfCAN::CCanTxNode*, 4> mtrSteerCanTxNode = {nullptr, nullptr, nullptr, nullptr};

        // 舵轮使能（4个舵向电机和发送节点均有效时启用）
        EVarStatus enableSwerve = false;

    } comWheelset_;

    // 定义髋关节组件并实例化
    class CComHip final: public CComponentBase{

    public:
        enum{LL = 0,LR = 1};

        // 定义底盘髋关节信息结构体并实例化
        struct SHipInfo {                                   
            float_t pos_L_L = 0.0f; ///< 定义了组件用于底层驱动
            float_t pos_L_R = 0.0f; ///< 后腿电机编码器值
        } HipInfo;

        // 定义底盘髋关节控制命令结构体并实例化
        struct SHipCommand {
            float_t L_Set_Angle = 0.0f; ///< 左腿腿长（编码器值）
            float_t R_Set_Angle = 0.0f; ///< 右腿腿长（编码器值）        
        } HipCmd;

        // MIT控制结构体
		struct SMitCtrl {
			float_t kp = 0.0f;
			float_t kd = 0.0f;
			float_t q = 0.0f;
			float_t dq = 0.0f;
			float_t tau = 0.0f;
		} mitCtrl[2];

        // 传感器实例指针
        CMemsBase *mems = nullptr;

        // 电机实例指针数组
        CDevMtr *motor[2] = {nullptr};

        // 底盘类父类指针，用于访问髋关节复位标志位
        CModChassis *parent = nullptr;

        // 定义髋关节PID控制器
        CAlgoPid HipPosPid[2];
        CAlgoPid HipSpdPid[2];
        CAlgoPid pidRollCtrl;                  ///< 整车roll轴控制

        // 电机数据输出缓冲区
        std::array<float_t, 2> mtrOutputBuffer = {0};

        // 初始化组件
        EAppStatus InitComponent(SModInitParam_Base &param) final;

        // 重写组件更新函数
        EAppStatus UpdateComponent() final;

        // 声明组件输出更新函数(负责根据控制量进行解算，以及进行PID运算，最后得到输出值)
        EAppStatus _UpdateOutput(float_t posit_L, float_t posit_R);
    
        // 电机can发送节点
        std::array<CInfCAN::CCanTxNode*, 2> mtrCanTxNode;

        // 面向髋关节组件的运动模式标志位
        EmovMode MovMode_ = EmovMode::NORMAL;
    }comHip_;

    // 定义履带组件并实例化
    class CComCrawler final: public CComponentBase{
     public:
        enum{L = 0, R = 1};

        // 定义底盘履带信息结构体并实例化
        struct SCrawlerInfo {                                   
            float_t speed_L = 0.0f;
            float_t speed_R = 0.0f;
        } CrawlerInfo;

        // 定义底盘履带控制命令结构体并实例化
        struct SCrawlerCommand {
            float_t speed_crawler = 0.f;    ///< 履带转速       
        } CrawlerCmd;

        // 电机实例指针数组
        CDevMtr *motor[2] = {nullptr};

        // 底盘类父类指针，用于访问启停履带标志位
        CModChassis *parent = nullptr;

        // 定义履带PID控制器
        CAlgoPid PidCrawlerSpdCtrl;

        // 电机数据输出缓冲区
        std::array<int16_t, 2> mtrOutputBuffer = {0};

        // 初始化组件
        EAppStatus InitComponent(SModInitParam_Base &param) final;

        // 重写组件更新函数
        EAppStatus UpdateComponent() final;

        // 声明组件输出更新函数(负责根据控制量进行解算，以及进行PID运算，最后得到输出值)
        EAppStatus _UpdateOutput(float_t speed);
    
        // 电机can发送节点
        std::array<CInfCAN::CCanTxNode*, 2> mtrCanTxNode;

        // 面向履带组件的运动模式标志位
        EmovMode MovMode_ = EmovMode::NORMAL;
    }comCrawler_;

    // 重写基类函数
    void UpdateHandler_() final;
    void HeartbeatHandler_() final;
    EAppStatus CreateModuleTask_() final;

    // 声明底盘模块任务函数(在proc_chassis.cpp中定义)
    static void StartChassisModuleTask(void *argument);

    // 声明控制量限制函数(负责对控制量进行限幅，在上面那个任务中进行调用)
    EAppStatus RestrictChassisCommand_();

    /**
     * @brief 计算当前控制周期内 4 个电机的需求总功率
     *
     * “需求功率”指根据当前轮组控制量（例如 PID 输出、电机目标速度等）
     * 推算得到的期望功率，而非实际测量得到的功率值。该函数会综合四个轮子
     * 的控制输出，估算每个电机在本周期内所需要的功率，并将其求和，用于
     * 后续的功率分配与限幅算法（如 AllocDynamicPower ）
     *
     * @param wheelset 底盘轮组组件，包含 4 个电机的当前状态及控制输出信息
     * @return float 4 个电机的需求总功率之和，单位：瓦特（W）
     */
    float CalcTotalDemandPower(const CComWheelset& wheelset, float wheelDemand[4], float steerDemand[4]);
    /** 动态分配每个电机的功率上限（总功率≤120W） */
    void AllocDynamicPower(const CComWheelset& wheelset, float targetWheelPower[4], float targetSteerPower[4]);
    void UpdatePowerBudget_();

};

} // namespace my_engineer

extern float wheel_power_lf;
extern float wheel_power_rf;
extern float wheel_power_lb;
extern float wheel_power_rb;
extern float powermeter;
extern float power_feedback_est;
extern float power_measure_used;
extern float power_guard_budget;
extern float power_demand_total;
extern float power_cmd_total;
extern float power_budget;
extern float power_buffer_est;

#endif // MOD_CHASSIS_HPP   