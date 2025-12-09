/**
 * @file mod_chassis.hpp
 * @author Fish_Joe (2328339747@qq.com)
 * @brief 定义底盘模块
 * @version 1.0
 * @date 2024-11-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef MOD_CHASSIS_HPP
#define MOD_CHASSIS_HPP

#include "mod_common.hpp"

#define LASER_ZERO_OFFSET_L 0
#define LASER_ZERO_OFFSET_R 0
#define L_LIFT_MOTOR_DIR 1 ///< 左腿编码器与腿长增加方向是否一致 一致为1 否则为-1
#define R_LIFT_MOTOR_DIR -1 ///< 右腿编码器与腿长增加方向是否一致 一致为1 否则为-1  暂定 这个待出车后改
#define CHASSIS_HIP_INIT_LENGTH 0.0f ///< 初始化腿长 后续待改
#define CHASSIS_HIP_INIT_ECD_L  0.0f
#define CHASSIS_HIP_INIT_ECD_R  0.0f    ///< 这两个是左右电机在初始化腿长时候的编码器值  这个得和陀螺仪数据0对应
#define CHASSIS_HIP_PHY_MAX     100.0f
#define CHASSIS_HIP_PHY_MIN     0.0f ///< 这个是最大和最短腿长
#define CHASSIS_HIP_ECD_MAX_L   0.0f
#define CHASSIS_HIP_ECD_MIN_L   0.0f
#define CHASSIS_HIP_ECD_MAX_R   0.0f
#define CHASSIS_HIP_ECD_MIN_R   0.0f    ///< 这几个是极限腿长时候两个电机对应的编码值 即软件限位 待改
#define ECD_LENGTH_RATIO        1.0f    ///< 这是腿长range和编码器range的线性对应关系，即传动比 这个保持为1就行
#define PITCH_DEG_ECD_RATIO     100.f   ///< 这是pitch动一度的时候编码器的变化值，待改
#define G 9.7803f    ///< 南山区的g值

#define deg2rad(x) ((x) * 0.017453292519943295769236907684886)
#define rad2deg(x) ((x) * 57.295779513082320876798154814105)

/* public定义用户层方便调试和获取信息，private定义了底层用于直接驱动电机，而不会因为外界的干扰影响了输出的值 */

namespace my_engineer {

/**
 * @brief 底盘模块类
 * 
 */
class CModChassis final: public CModBase{
public:
    // 定义底盘模块初始化参数结构体
    struct SModInitParam_Chassis: public SModInitParam_Base{
        EDeviceID memsDevID = EDeviceID::DEV_NULL;
        EDeviceID wheelsetMotorID_LF = EDeviceID::DEV_NULL;
        EDeviceID wheelsetMotorID_RF = EDeviceID::DEV_NULL;
        EDeviceID wheelsetMotorID_LB = EDeviceID::DEV_NULL;
        EDeviceID wheelsetMotorID_RB = EDeviceID::DEV_NULL;
        EDeviceID hipMotorID_L_L = EDeviceID::DEV_NULL;
        EDeviceID hipMotorID_L_R = EDeviceID::DEV_NULL; ///< 后腿电机
        CInfCAN::CCanTxNode *wheelsetMotorTxNode_LF;
        CInfCAN::CCanTxNode *wheelsetMotorTxNode_RF;
        CInfCAN::CCanTxNode *wheelsetMotorTxNode_LB;
        CInfCAN::CCanTxNode *wheelsetMotorTxNode_RB;
        CInfCAN::CCanTxNode *hipMotorTxNodeID_L_L;
        CInfCAN::CCanTxNode *hipMotorTxNodeID_L_R;
        float_t MIT_L_kp = 0.0f; ///< MIT控制器比例系数
		float_t MIT_L_kd = 0.0f; ///< MIT控制器微分系数
        CAlgoPid::SAlgoInitParam_Pid wheelsetSpdPidParam;
        CAlgoPid::SAlgoInitParam_Pid lineCorrectionPidParam;
        CAlgoPid::SAlgoInitParam_Pid yawCorrectionPidParam;
        CAlgoPid::SAlgoInitParam_Pid pitchCorrectionPidParam; ///< pitch轴控制pid
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
    } chassisCmd;

    // 整车pitch轴角度
    DataBuffer<float_t> pitch_Measure;

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

    // 模式标志位
    EmovMode MovMode = EmovMode::NORMAL;

private:

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

        // 定义底盘PID控制器
        CAlgoPid pidYawCtrl;                    ///<控制底盘角速度（Yaw旋转）
        CAlgoPid pidLineCorrectionCtrl;         ///<修正X、Y、W三个方向的误差
        CAlgoPid pidSpdCtrl;                    ///<控制4个轮子的速度

        // 电机数据输出缓冲区
        std::array<int16_t, 6> mtrOutputBuffer = {0};

        // 初始化组件
        EAppStatus InitComponent(SModInitParam_Base &param) final;

        // 重写组件更新函数
        EAppStatus UpdateComponent() final;

        // 声明组件输出更新函数(负责根据控制量进行解算，以及进行PID运算，最后得到输出值)
        EAppStatus _UpdateOutput(float speed_X, float speed_Y, float speed_W);
    
        // 电机can发送节点
        std::array<CInfCAN::CCanTxNode*, 4> mtrCanTxNode;

        // 面向轮组的模式标志位
        EmovMode MovMode_ = EmovMode::NORMAL;
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

        // 定义底盘PID控制器
        CAlgoPid pidPitchCtrl;                  ///< 整车pitch轴控制

        // 电机数据输出缓冲区
        std::array<int16_t, 2> mtrOutputBuffer = {0};

        // 初始化组件
        EAppStatus InitComponent(SModInitParam_Base &param) final;

        // 重写组件更新函数
        EAppStatus UpdateComponent() final;

        // 声明组件输出更新函数(负责根据控制量进行解算，以及进行PID运算，最后得到输出值)
        EAppStatus _UpdateOutput(float L_Length);
    
        // 电机can发送节点
        std::array<CInfCAN::CCanTxNode*, 2> mtrCanTxNode;

        // 面向轮组的运动模式标志位
        EmovMode MovMode_ = EmovMode::NORMAL;
    }comHip_;

    // 重写基类函数
    void UpdateHandler_() final;
    void HeartbeatHandler_() final;
    EAppStatus CreateModuleTask_() final;

    // 声明底盘模块任务函数(在proc_chassis.cpp中定义)
    static void StartChassisModuleTask(void *argument);

    // 声明控制量限制函数(负责对控制量进行限幅，在上面那个任务中进行调用)
    EAppStatus RestrictChassisCommand_();


};

} // namespace my_engineer

#endif // MOD_CHASSIS_HPP   