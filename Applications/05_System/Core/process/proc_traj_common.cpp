/******************************************************************************
 * @brief        轨迹回放模块实现
 * @file         proc_traj_common.cpp
 * @author       ciallo
 * @version      V2.0
 * @date         2026-4-23
 ******************************************************************************/

 #include "proc_common.hpp"

//debug
extern "C" {
volatile int32_t traj_dbg_seg = -1;
volatile int32_t traj_dbg_exit_reason = 0;      // 0 running, 1 ok, 2 ctrl_z, 3 关节超时, 4 夹爪超时
volatile int32_t traj_dbg_warn_reason = 0;      // 0 none, 3 等待慢关节, 4 夹爪闭合的慢
volatile int32_t traj_dbg_wait_joint = -1;
volatile float traj_dbg_max_joint_error = 0.0f;
volatile float traj_dbg_joint_current = 0.0f;
volatile float traj_dbg_joint_target = 0.0f;
volatile int32_t traj_dbg_wait_grip = 0;
volatile float traj_dbg_grip_cmd = 0.0f;
volatile float traj_dbg_grip_info = 0.0f;
}
 namespace my_engineer{
    
    /*------------------------------ 轨迹帧定义区  -----------------------------------*/

    /*-----------------------------------存矿石--------------------------------------*/

    // 存左矿轨迹帧, 夹爪 0夹紧 1松开
    // 这里的取矿的路径还是有点问题2026/5/9
    
    const float_t Traj_Grab_L[][FC_COUNT] = {
        // time      yaw        p1          p2       p3       roll        endP        endR       grip       speed
        {      0,   79.31f,   36.62f,  51.35f,   -77.62f,   190.18f,   -106.33f,       1.f,        0 ,      4.1f},  // 起始位
        {  1000,   72.8f,  35.32f,  49.438f,   -68.620f,  189.179f,  -106.427f,       1.f,        0 ,      4.f},  //
        {  2000,   72.8f,  40.81f,  56.34f,   -67.484f,  186.2f,  -29.427f,          1.f,        0,      2.5f},
        { 3000,   72.8f,  49.34f,  37.35f,   -68.86f,  186.248f,  -39.427f,         1.f,        0 ,      2.5f},
        { 4000,   72.8f,  49.34f,  37.35f,   -68.86f,  186.248f,  -39.427f,         1.f,        0 ,      2.5f},
        { 5000,   72.8f,  51.34f,  31.35f,   -61.86f,  184.248f,  -39.427f,         1.f,        0 ,      2.5f},
        { 6000,   72.8f,  61.24f,  37.52f,   -67.86f,  184.248f,  -39.427f,         1.f,        0 ,      2.5f},
        //{ 5000,   78.2f,  49.34f,  37.35f,   -71.86f,  187.248f,  -121.427f,         1.8f,        0 ,      0.7f},  // 夹爪松开前 (t≈23s)
        //{ 6000,   78.2f,  49.34f,  37.35f,   -71.86f,  187.248f,  -121.427f,         1.8f,        0 ,      0.7f},  // <<<< 夹爪松开后

        { 7000,   72.8f,  71.24f,  37.52f,   -69.86f,  184.248f,  -45.427f,         1.f,        1 ,      2.5f},
        { 8000,   72.2f,  27.24f,  32.85f,   -69.86f,  188.248f,  -45.427f,         1.f,        1 ,      2.5f},//松开之后
        { 9000,   80.02f,  27.65f,  32.75f,   -67.86f,  190.248f,  30.427f,         1.f,        1 ,      2.f},
        // { 10000,   79.02f,  27.65f,  48.75f,   -65.86f,  190.248f,  30.427f,         1.f,        1 ,      4.0f},
        // { 11000,   78.02f,  21.65f,  54.75f,   -63.86f,  190.248f,  30.427f,         1.f,        1 ,      4.0f},
        { 10000,   -2.02f,  22.65f,  55.75f,   -60.86f,  188.248f,  30.427f,         1.f,        1 ,      4.0f},
        { 11000,   -2.02f,  52.45f,  43.75f,   -1.86f,  180.248f,  -53.427f,         1.f,        1 ,      4.f},

    };
    const int Traj_GrabLen_L = sizeof(Traj_Grab_L) / sizeof(Traj_Grab_L[0]); //计算帧数

    /*-----------------------------------------------------------------------------------------------------------------------*/
    // 原右存矿，数据效果差
    // const float_t Traj_Grab_R_Measured[][FC_COUNT] = {
    //     // time      yaw        p1          p2       p3       roll        endP        endR       grip       speed
    //     {      0,   -71.31f,   36.62f,  51.35f,   -77.62f,   180.18f,   -121.33f,      -1.f,      0 ,      1.1f},
    //     {  1000,   -71.8f,    45.32f,  61.438f,  -77.620f,  183.179f,  -89.427f,      -1.f,      0 ,      1.f},
    //     {  2000,   -71.8f,    40.81f,  38.34f,   -67.484f,  183.2f,    -89.427f,       8.f,      0,       0.9f},
    //     {  3000,   -71.8f,    56.34f,  34.35f,   -53.86f,   183.248f, -121.427f,       8.f,      0 ,      0.9f},
    //     {  5000,   -71.8f,    70.24f,  28.52f,   -44.86f,   178.248f, -110.427f,       8.f,      0 ,      0.5f},
    //     {  6000,   -71.8f,    75.24f,  32.52f,   -44.86f,   178.248f, -105.427f,       8.f,      0 ,      0.5f},
    //     {  7000,   -71.8f,    70.24f,  31.52f,   -54.86f,   178.248f,  -90.427f,       8.f,      0 ,      0.5f},
    //     {  8000,   -71.8f,    75.24f,  32.52f,   -54.86f,   178.248f,  -90.427f,       8.f,      0 ,      0.5f},
    //     {  9000,   -71.8f,    86.24f,  33.52f,   -55.86f,   178.248f,  -90.427f,       8.f,      1 ,      0.5f},
    //     { 10000,   -71.2f,    27.24f,  32.85f,   -50.86f,   178.248f,   -0.427f,       8.f,      1 ,      0.8f},
    //     { 11000,   -71.02f,   27.65f,  50.75f,   -67.86f,   180.248f,   20.427f,       8.f,      1 ,      1.2f},
    //     { 12000,   -71.02f,   27.65f,  53.75f,   -65.86f,   180.248f,   20.427f,       8.f,      1 ,      1.2f},
    //     { 13000,   -71.02f,   27.65f,  54.75f,   -66.86f,   180.248f,   20.427f,       8.f,      1 ,      1.0f},
    //     { 14000,   -2.02f,    22.65f,  55.75f,   -60.86f,   180.248f,   20.427f,       8.f,      1 ,      1.0f},
    //     { 15000,   -2.02f,    52.45f,  43.75f,   -1.86f,    180.248f,  -68.427f,       8.f,      1 ,      1.3f},
    // };

    // 存右矿石：右手 YAW 使用原实测值，其余关节复用左手存矿轨迹特征。
    const float_t Traj_Grab_R[][FC_COUNT] = {
        // time      yaw        p1          p2       p3       roll        endP        endR       grip       speed
        {      0,   -71.31f,   36.62f,  51.35f,   -77.62f,   190.18f,   -106.33f,      1.f,      0 ,      4.1f},
        {  1000,   -71.8f,    35.32f,  49.438f,  -68.620f,  189.179f,  -106.427f,     1.f,      0 ,      4.f},
        {  2000,   -71.8f,    40.81f,  56.34f,   -67.484f,  186.2f,    -29.427f,      1.f,      0,       2.5f},
        {  3000,   -71.8f,    49.34f,  37.35f,   -68.86f,   186.248f,  -45.427f,      1.f,      0 ,      2.5f},
        {  4000,   -71.8f,    49.34f,  37.35f,   -68.86f,   186.248f,  -38.427f,      1.f,      0 ,      2.5f},
        {  5000,   -71.8f,    51.34f,  31.35f,   -61.86f,   184.248f,  -38.427f,      1.f,      0 ,      2.5f},
        {  6000,   -71.8f,    61.24f,  37.52f,   -67.86f,   184.248f,  -38.427f,      1.f,      0 ,      2.5f},
        {  7000,   -71.8f,    65.24f,  37.52f,   -67.86f,   184.248f,  -45.427f,      1.f,      1 ,      2.5f},
        {  8000,   -71.8f,    27.24f,  32.85f,   -69.86f,   184.248f,  -45.427f,      1.f,      1 ,      2.5f},
        {  9000,   -71.2f,    27.65f,  32.75f,   -67.86f,   184.248f,    30.427f,      1.f,      1 ,      2.5f},
        // { 10000,   -79.02f,   27.65f,  48.75f,   -65.86f,   184.248f,    30.427f,      1.f,      1 ,      4.0f},
        // { 11000,   -79.02f,   21.65f,  54.75f,   -63.86f,   184.248f,   30.427f,      1.f,      1 ,      4.0f},
        { 10000,   -2.02f,    22.65f,  55.75f,   -60.86f,   188.248f,   30.427f,      1.f,      1 ,      4.0f},
        { 11000,   -2.02f,    52.45f,  43.75f,   -1.86f,    180.248f,  -53.427f,      1.f,      1 ,      4.f},
    };
    const int Traj_GrabLen_R = sizeof(Traj_Grab_R) / sizeof(Traj_Grab_R[0]); //计算帧数
    
     /*-----------------------------------取矿石--------------------------------------*/
    // 取左矿轨迹帧 (从CSV数据提取), 夹爪 0夹紧 1松开
    
    const float_t Traj_Get_L[][FC_COUNT] = {  //由于灯条的干涉所以大部分的时间都是夹取的状态
        // time      yaw        p1          p2       p3       roll        endP        endR       grip       speed
        {      0,   22.8f,   18.35f,  45.808f,   -50.89f,   184.24f,   45.36f,       1.f,        0 ,      4.f},  // 起始位//roll:188
        {  1000,   22.8f,   18.35f,  45.808f,   -50.89f,   184.24f,   45.36f,       1.f,        0 ,      4.f},  // 起始位
        {  2000,   70.8f, 40.32f,  47.438f,   -67.920f,  185.179f,  45.427f,       1.8f,        0 ,      4.f},  // 
        //{  3000,   61.8f,  38.32f,  40.438f,   -64.920f,  185.179f,  10.427f,       0.f,        1 ,      1.f},  //

         {  3000,  70.8f,  20.32f,  23.438f,   -46.920f,  184.179f,  8.573f,       1.8f,        0 ,      4.f},  //这里有点问题需要调整一下
         {  4000,   70.8f,  19.32f,  23.438f,   -46.920f,  184.179f,  11.427f,       3.f,        0 ,      2.f},  //

         {  6000,   70.8f,  27.32f,  26.438f,   -50.920f,  184.179f,  0.427f,       3.f,        1 ,      2.f},  //
        {  7000,   73.8f,  42.32f,  28.438f,   -53.920f,  184.179f,  -29.427f,       3.f,        1 ,      2.f},  //
         
        {  8000,   77.8f,  50.91f,  31.14f,   -50.3f,  189.2f,  -54.427f,          3.f,        0,      2.f},                //test
         {  9000,  77.8f,  39.91f,  20.14f,   -46.3f,  189.2f,  -9.427f,          3.f,        0,      2.2f},

        
        { 10000,  77.2f,  30.24f,  22.85f,   -48.86f,  189.248f,  -106.427f,         3.f,        0 ,      6.0f},
        { 11000,   59.92f,  20.65f,  26.75f,   -40.86f,  189.248f,  -106.427f,         5.f,        0 ,      4.0f},
        { 12000,   -2.02f,  15.45f,  20.75f,   -1.86f,  184.248f,  -61.427f,         0.f,        0 ,      4.f},
    };
    const int Traj_GetLen_L = sizeof(Traj_Get_L) / sizeof(Traj_Get_L[0]); //计算帧数

    // 原右取矿数据效果差
    // const float_t Traj_Get_R_Measured[][FC_COUNT] = {
    //     // time      yaw        p1          p2       p3       roll        endP        endR       grip       speed
    //     {      0,   -22.8f,   18.35f,  45.808f,   -50.89f,   184.24f,   30.36f,       1.f,        0 ,      1.f},
    //     {  1000,   -22.8f,   18.35f,  45.808f,   -50.89f,   184.24f,   10.36f,       1.f,        0 ,      1.3f},
    //     {  2000,   -78.8f,   40.32f,  47.438f,   -67.920f,  185.179f,  10.427f,      1.8f,      0 ,      1.f},
    //     {  3000,   -78.8f,   20.32f,  23.438f,   -41.920f,  185.179f, -10.427f,      1.8f,      0 ,      1.f},
    //     {  4000,   -78.8f,   20.32f,  20.438f,   -38.920f,  185.179f, -10.427f,      2.f,        1 ,      1.f},
    //     {  5000,   -78.8f,   31.32f,  28.438f,   -50.920f,  185.179f, -30.427f,      2.f,        1 ,      1.f},
    //     {  6000,   -78.8f,   35.32f,  20.438f,   -37.920f,  185.179f, -60.427f,     -1.f,        1 ,      1.f},
    //     {  7000,   -78.8f,   53.91f,  18.14f,    -37.3f,    184.2f,   -60.427f,      6.f,        0,       1.f},
    //     {  8000,   -78.8f,   44.91f,  26.14f,    -42.3f,    182.2f,   -82.427f,      6.f,        0,       1.f},
    //     {  9000,   -78.8f,   35.24f,  30.85f,    -29.86f,   182.248f,-121.427f,     -1.f,        0 ,      1.2f},
    //     { 10000,   -78.8f,   30.24f,  21.85f,    -29.86f,   182.248f,-121.427f,     -1.f,        0 ,      1.0f},
    //     { 11000,   -78.8f,   18.24f,  20.85f,    -29.86f,   182.248f, -60.427f,     -1.f,        0 ,      1.0f},
    //     { 12000,   -59.92f,  18.65f,  26.75f,    -30.86f,   180.248f, -60.427f,     -1.f,        0 ,      1.0f},
    //     { 13000,   -2.02f,   45.45f,  26.75f,    -1.86f,    180.248f, -76.427f,      8.f,        0 ,      1.3f},
    // };

    // 取右矿石：右手 YAW 使用原实测值，其余关节复用左手取矿轨迹。
    const float_t Traj_Get_R[][FC_COUNT] = {
        // time      yaw        p1          p2       p3       roll        endP        endR       grip       speed
        {      0,   -22.8f,   18.35f,  45.808f,   -50.89f,   184.24f,   45.36f,       1.f,        0 ,      4.0f},
        {  1000,   -22.8f,   18.35f,  45.808f,   -50.89f,   184.24f,   45.36f,       1.f,        0 ,      4.0f},
        {  2000,   -74.8f,   40.32f,  47.438f,   -67.920f,  185.179f,  45.427f,      1.8f,      0 ,      4.0f},
        {  3000,   -74.8f,   20.32f,  23.438f,   -46.920f,  185.179f,   10.573f,      1.8f,      0 ,      4.0f},
        {  4000,   -74.8f,   19.32f,  23.438f,   -46.920f,  181.179f, 5.427f,      3.f,        0 ,      2.0f},
        {  5000,   -74.8f,   27.32f,  26.438f,   -50.920f,  184.179f, -6.427f,      3.f,        1 ,      2.5f},
        {  6000,   -74.8f,   36.32f,  17.438f,   -51.920f,  184.179f, -45.427f,      3.f,        1 ,      2.5f},
        {  7000,   -74.8f,   47.91f,  10.14f,    -29.3f,    190.2f,   -54.427f,      3.f,        0,       2.5f},
        {  8000,   -74.8f,   36.91f,  24.14f,    -48.3f,    190.2f,   -9.427f,      3.f,        0,       2.5f},//test
        {  9000,   -74.8f,   18.24f,  28.85f,    -48.86f,   190.248f,-106.427f,      3.f,        0 ,      6.0f},
        { 10000,   -59.92f,  20.65f,  26.75f,    -40.86f,   190.248f,-106.427f,      5.f,        0 ,      4.0f},
        { 11000,   -2.02f,   15.45f,  20.75f,    -1.86f,    184.248f, -61.427f,      0.f,        0 ,      4.0f},
    };
    const int Traj_GetLen_R = sizeof(Traj_Get_R) / sizeof(Traj_Get_R[0]); //计算帧数

    /*------------------------------ 定义对应的轨迹图 -----------------------------------*/
    // 将取矿加入到图中
    std::map<ETrajID,TrajClip> TrajMap = {
        {TRAJ_GRAB_L,{Traj_Grab_L,Traj_GrabLen_L}},
        {TRAJ_GET_L,{Traj_Get_L,Traj_GetLen_L}},
        {TRAJ_GRAB_R,{Traj_Grab_R,Traj_GrabLen_R}},
        {TRAJ_GET_R,{Traj_Get_R,Traj_GetLen_R}},
    };

    /** @brief 读取机械臂关节角度
     *  @param arm 机械臂对象
     *  @param output 输出数组
     */

    void ReadArmjoint(const CModArm &arm,float_t output[7]){
        output[J::J_YAW]  = arm.armInfo.angle_Yaw;
        output[J::J_P1]   = arm.armInfo.angle_Pitch1;
        output[J::J_P2]   = arm.armInfo.angle_Pitch2;
        output[J::J_P3]   = arm.armInfo.angle_Pitch3;
        output[J::J_ROLL] = arm.armInfo.angle_Roll;
        output[J::J_ENDP] = arm.armInfo.angle_end_pitch;
        output[J::J_ENDR] = arm.armInfo.angle_end_roll;
    }

    /** @brief 写入机械臂关节角度
     *  @param arm 机械臂对象
     *  @param output 输出数组
     */
    void WriteArmjoint(CModArm &arm,const float_t output[7]){
        arm.armCmd.set_angle_Yaw = output[J::J_YAW];
        arm.armCmd.set_angle_Pitch1 = output[J::J_P1];
        arm.armCmd.set_angle_Pitch2 = output[J::J_P2];
        arm.armCmd.set_angle_Pitch3 = output[J::J_P3];
        arm.armCmd.set_angle_Roll = output[J::J_ROLL];
        arm.armCmd.set_angle_end_pitch = output[J::J_ENDP];
        arm.armCmd.set_angle_end_roll = output[J::J_ENDR];
    }

    /** @brief 提取轨迹
     *  @param traj 轨迹数据
     *  @param row 轨迹行号
     *  @param output 输出数组
     */

    void Extrarow(const float_t traj[][FC_COUNT], int row, float_t output[7]){
        for(int i = 0;i < 7;i++){
            output[i] = traj[row][i+1];
        }
    }

    /** @brief 检查关节角度是否到达目标的角度
     */
    bool CheckAllJointsArrived(const CModArm &arm, const float_t target[J::COUNT], const SArrivalCheckConfig &cfg) {
        float_t current[J::COUNT];
        ReadArmjoint(arm, current);

        bool arrived = true;
        int32_t maxErrJoint = -1;
        float_t maxErr = 0.0f;

        //等待超时标记关节方便debug
        for (int i = 0; i < J::COUNT; i++) {
            float_t err = std::fabs(current[i] - target[i]);
            if (err > maxErr) {
                maxErr = err;
                maxErrJoint = i;
                traj_dbg_joint_current = current[i];
                traj_dbg_joint_target = target[i];
            }
            if (err > cfg.toleranceDeg) {
                arrived = false;
            }
        }

        traj_dbg_wait_joint = maxErrJoint;
        traj_dbg_max_joint_error = maxErr;

        return arrived;
    }

    // 提取第 row 行的夹爪状态：0=夹紧, 非0=松开
    bool ExtractGripClose(const float_t traj[][FC_COUNT], int row) {
        return traj[row][FC_GRIP] < 1.0f;
    }
    // 提取第 row 行的关节速度状态
    float_t ExtractSpeed(const float_t traj[][FC_COUNT], int row) {
        return traj[row][FC_SPEED];
    }
    
    //控制夹爪的张开和闭合
    void WriteGripCommand(CModArm &arm, bool close) {
        if (close) {
            arm.armCmd.gripClose = true;
            arm.armCmd.gripOpen = false;
        } else {
            arm.armCmd.gripClose = false;
            arm.armCmd.gripOpen = true;
        }
    }

    //检查夹爪是否到位
    bool CheckGripArrived(const CModArm &arm, bool close) {
        const auto state = arm.armInfo.gripState;
        if (close) {
            return state == CModArm::SArmInfo::EGripState::HOLD && arm.armInfo.length_grip <=  ARM_END_GRIP_PHYSICAL_RANGE_MAX - GRIP_CLOSE_Stop_distance;//增强判断依据防止夹爪的状态误判
        }

        return state == CModArm::SArmInfo::EGripState::RELEASE && arm.armInfo.length_grip >= ARM_END_GRIP_PHYSICAL_RANGE_MAX - GRIP_OPEN_Stop_distance;
    }

    //辅助debug夹爪函数
    bool WaitGripArrived(CModArm &arm, bool close, bool checkctrl,
                         const SArrivalCheckConfig &cfg) {
        const uint32_t startTick = HAL_GetTick();

        while (true) {
            if(checkctrl && SysRemote.remoteInfo.keyboard.key_Ctrl
               && SysRemote.remoteInfo.keyboard.key_Z) {
                traj_dbg_exit_reason = 2;
                return false;
            }

            WriteGripCommand(arm, close);

            const bool gripArrived = CheckGripArrived(arm, close);
            traj_dbg_wait_grip = gripArrived ? 0 : 1;
            traj_dbg_grip_cmd = arm.armCmd.set_length_grip;
            traj_dbg_grip_info = arm.armInfo.length_grip;

            if (gripArrived) {
                traj_dbg_exit_reason = 1;
                return true;
            }

            if (HAL_GetTick() - startTick >= cfg.gripTimeoutMs) {
                traj_dbg_warn_reason = 4;
            }

            if (HAL_GetTick() - startTick >= cfg.hardTimeoutMs) {
                traj_dbg_exit_reason = 4;
                return false;
            }

            proc_waitMs(1);
        }
    }

    /** @brief 轨迹播放器
     *  @param arm 臂的控制和信息参数
     *  @param target 目标关节角度
     *  @param gripDuringMotion 关节运动过程中夹爪保持的状态，关节运动过程中保持的夹爪状态（true=夹紧, false=松开）
     *  @param gripAfter        关节到位之后才切换的夹爪状态
     *  @param player 播放器的内部速度参数定义
     *  @param startOverride 非空：用其作为规划起点
     *  @param minTimeS 可选最小总时长(秒)
     */
    bool PlaySegment(CModArm &arm, const float_t target[J::COUNT],
                           float_t speedScale,
                           bool gripDuringMotion, bool gripAfter,
                           CAlgoTrajPlayback &player, bool checkctrl,
                           const float_t *startOverride,
                           float_t minTimeS){

        const SArrivalCheckConfig arrivalCfg;//到位检查函数

        /*-----------------------  功能函数  ---------------------------*/
        // 夹爪控制，true=夹紧, false=松开
        float_t current[J::COUNT];
        float_t joints[J::COUNT];

        // 规划起点：优先使用 startOverride（上一段 target），否则读实时反馈
        if (startOverride != nullptr) {
            for (int i = 0; i < J::COUNT; i++) current[i] = startOverride[i];
        } else {
            ReadArmjoint(arm, current);
        }

        player.speedScale = speedScale;                       // 设置当前的速度比例
        player.PlanMultiAxisTraj(current, target, minTimeS);  // 最小时长约束

        bool arrivalCheckStarted = false;
        bool stableTiming = false;
        bool frameJointsArrived = false;
        uint32_t arrivalStartTick = 0;
        uint32_t stableStartTick = 0;

        uint32_t startTick = HAL_GetTick();//获取时间轴

        while(true){

            // ctrl+z操作手打断回放避免实际位姿错误或者出现干涉
            if(checkctrl && SysRemote.remoteInfo.keyboard.key_Ctrl
               && SysRemote.remoteInfo.keyboard.key_Z) {
                traj_dbg_exit_reason = 2;
                return false;
            }

            const uint32_t nowTick = HAL_GetTick();
            const uint32_t elapsedMs = nowTick - startTick;
            float_t elapsed = static_cast<float_t>(elapsedMs) / 1000.0f;//转换成当前秒数
            const bool frameTargetCommanded = player.IsFinished(elapsed);

            // 关节角度播放
            if(frameTargetCommanded) {
                WriteArmjoint(arm, target);
            } else {
                player.MultiAxisTrajDistance(elapsed, joints);//根据当前的秒数度取关节的信息
                WriteArmjoint(arm, joints);//将获取到的关节信息反写入arm中
            }

            // 运动期间夹爪保持上一段状态，禁止提前切换
            WriteGripCommand(arm, gripDuringMotion);

//测试代码：
            if(frameTargetCommanded) {
                if(!arrivalCheckStarted) {
                    arrivalCheckStarted = true;
                    arrivalStartTick = nowTick;
                }

                if(!frameJointsArrived) {
                    if(CheckAllJointsArrived(arm, target, arrivalCfg)) {
                        if(!stableTiming) {
                            stableTiming = true;
                            stableStartTick = nowTick;
                        }

                        if(nowTick - stableStartTick >= arrivalCfg.stableMs) {
                            frameJointsArrived = true;
                        }
                    } else {
                        stableTiming = false;
                    }

                    if(nowTick - arrivalStartTick >= arrivalCfg.timeoutMs) {
                        traj_dbg_warn_reason = 3;
                    }

                    //Debug: 超时退出
                    if(nowTick - arrivalStartTick >= arrivalCfg.hardTimeoutMs) {
                        traj_dbg_exit_reason = 3;
                        return false;
                    }
                }

                // 关节到位后退出，夹爪切换由 PlayFrameSegment
                if(frameJointsArrived) break;
            }

            proc_waitMs(1);
        }

        // 关节到位后才切换到本段目标夹爪状态（夹爪到位检查由 PlayFrameSegment 负责）
        WriteArmjoint(arm, target);
        WriteGripCommand(arm, gripAfter);
        traj_dbg_exit_reason = 1;
        return true;
    }


    //再原来的播放器的基础上再封装一个速度读取的函数
    //  prevTarget 非空：用上一段 target 做起点
    //  prevTarget 为空：用实时反馈做起点
    //  earlyGrip  true: 夹爪在段开始时切换(与关节运动重叠)，false: 关节到位后才切换
    bool PlayFrameSegment(CModArm &arm,
                                const float_t traj[][FC_COUNT], int seg,
                                CAlgoTrajPlayback &player, bool checkctrl,
                                float_t endRollOffset,
                                const float_t *prevTarget,
                                bool earlyGrip){
        float_t target[J::COUNT];
        const bool gripAfter = ExtractGripClose(traj, seg);   // 本段目标状态

        // 判断夹爪状态在本段是否真的发生了切换
        const bool gripActuallyChanged = (seg == 0)
            ? !CheckGripArrived(arm, gripAfter)
            : (gripAfter != ExtractGripClose(traj, seg - 1));

        // earlyGrip: 段一开始就切换夹爪
        // 否则: 关节运动期间保持上一段状态，到位后才切
        bool gripDuringMotion;
        if (earlyGrip) {
            gripDuringMotion = gripAfter;
        } else if (seg == 0) {
            gripDuringMotion = (arm.armInfo.gripState == CModArm::SArmInfo::EGripState::HOLD);
        } else {
            gripDuringMotion = ExtractGripClose(traj, seg - 1);
        }

        float_t speed = ExtractSpeed(traj, seg);
        Extrarow(traj, seg, target);
        target[J::J_ENDR] += endRollOffset;

        traj_dbg_seg = seg;
        traj_dbg_exit_reason = 0;
        traj_dbg_warn_reason = 0;
        traj_dbg_wait_joint = -1;
        traj_dbg_wait_grip = 0;

        // 关节运动期间保持 gripDuringMotion；关节到位后才切到 gripAfter
        if(!PlaySegment(arm, target, speed,
                        gripDuringMotion, gripAfter,
                        player, checkctrl,
                        prevTarget )) {// minTimeS = 0，按物理参数自由规划 
            return false;
        }
        if(gripActuallyChanged) {   // 夹爪有切换，等待夹爪到位
            const SArrivalCheckConfig arrivalCfg;
            return WaitGripArrived(arm, gripAfter, checkctrl, arrivalCfg);
        }
        return true;
    }


 }// namespace my_engineer
