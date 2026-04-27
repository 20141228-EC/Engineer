/******************************************************************************
 * @brief        轨迹回放模块实现
 * @file         proc_traj_common.cpp
 * @author       ciallo
 * @version      V2.0
 * @date         2026-4-23
 ******************************************************************************/

 #include "proc_common.hpp"

 namespace my_engineer{
    
    /*------------------------------ 轨迹帧定义区  -----------------------------------*/
    // 存矿轨迹帧, 夹爪 0夹紧 1松开
    // 夹爪在第95帧松开 (约23秒处)
    
    const float_t Traj_Grab[][FC_COUNT] = {
        // time      yaw        p1          p2       p3       roll        endP        endR       grip       speed
        {      0,   79.31f,   36.62f,  51.35f,   -77.62f,   190.18f,   -121.33f,       1.8f,        0 ,      1.2f},  // 起始位
        {  1000,   78.2f,  39.32f,  45.438f,   -76.620f,  189.179f,  -121.427f,       1.8f,        0 ,      1.f},  // 
        {  2000,   78.2f,  36.81f,  35.34f,   -72.484f,  189.2f,  -121.427f,          1.8f,        0,      0.7f},  
        { 3000,   78.2f,  49.34f,  37.35f,   -71.86f,  187.248f,  -121.427f,         1.8f,        0 ,      0.7f},
        { 4000,   78.2f,  49.34f,  37.35f,   -71.86f,  187.248f,  -121.427f,         1.8f,        0 ,      0.7f},
        { 5000,   78.2f,  51.34f,  31.35f,   -61.86f,  187.248f,  -121.427f,         1.8f,        0 ,      0.7f},
        { 6000,   78.2f,  61.24f,  37.52f,   -67.86f,  187.248f,  -118.427f,         1.8f,        0 ,      0.7f},  
        //{ 5000,   78.2f,  49.34f,  37.35f,   -71.86f,  187.248f,  -121.427f,         1.8f,        0 ,      0.7f},  // 夹爪松开前 (t≈23s)
        //{ 6000,   78.2f,  49.34f,  37.35f,   -71.86f,  187.248f,  -121.427f,         1.8f,        0 ,      0.7f},  // <<<< 夹爪松开后

        { 7000,   78.2f,  71.24f,  26.85f,   -49.86f,  188.248f,  -118.427f,         1.8f,        0 ,      0.5f},
        { 8000,   78.2f,  27.24f,  32.85f,   -50.86f,  188.248f,  -0.427f,         1.8f,        1 ,      0.5f},
        { 9000,   80.02f,  27.65f,  50.75f,   -67.86f,  190.248f,  10.427f,         -8.f,        1 ,      1.0f},
        { 10000,   79.02f,  27.65f,  52.75f,   -65.86f,  190.248f,  10.427f,         -8.f,        1 ,      1.0f},
        { 11000,   78.02f,  21.65f,  53.75f,   -63.86f,  190.248f,  10.427f,         -8.f,        1 ,      1.0f},
        { 12000,   -2.02f,  22.65f,  54.75f,   -60.86f,  190.248f,  10.427f,         -8.f,        1 ,      1.0f},
        { 13000,   -2.02f,  52.45f,  43.75f,   -1.86f,  189.248f,  -68.427f,         -8.f,        1 ,      1.3f},


    };
    const int Traj_GrabLen = sizeof(Traj_Grab) / sizeof(Traj_Grab[0]); //计算帧数

    // 取矿轨迹帧 (从CSV数据提取), 夹爪 0夹紧 1松开
    // 夹爪在第95帧松开 (约23秒处)
    
    const float_t Traj_Get[][FC_COUNT] = {
        // time      yaw        p1          p2       p3       roll        endP        endR       grip       speed
        {      0,   22.8f,   18.35f,  45.808f,   -50.89f,   184.24f,   30.36f,       1.f,        1 ,      1.3f},  // 起始位//roll:188
        {  1000,   22.8f,   18.35f,  45.808f,   -50.89f,   184.24f,   30.36f,       1.f,        1 ,      1.3f},  // 起始位
        {  2000,   61.8f, 40.32f,  47.438f,   -67.920f,  185.179f,  30.427f,       1.8f,        1 ,      1.f},  // 
        //{  3000,   61.8f,  38.32f,  40.438f,   -64.920f,  185.179f,  10.427f,       0.f,        1 ,      1.f},  //

         {  3000,   68.8f,  20.32f,  23.438f,   -41.920f,  185.179f,  -26.427f,       1.8f,        1 ,      1.f},  //
         {  4000,   68.8f,  20.32f,  20.438f,   -38.920f,  185.179f,  -26.427f,       2.f,        1 ,      1.f},  //

         {  5000,   68.8f,  33.32f,  17.438f,   -44.920f,  185.179f,  -62.427f,       2.f,        1 ,      1.f},  //
        {  6000,   68.8f,  33.32f,  17.438f,   -44.920f,  185.179f,  -62.427f,       1.f,        1 ,      1.f},  //
         
        {  7000,   68.8f,  40.91f,  14.14f,   -38.3f,  181.2f,  -69.427f,          1.f,        0,      0.7f},    
         {  8000,   68.8f,  17.91f,  16.14f,   -25.3f,  181.2f,  -24.427f,          1.f,        0,      0.7f},
       
        
        { 9000,  78.2f,  18.24f,  28.85f,   -63.86f,  188.248f,  -121.427f,         1.8f,        0 ,      1.0f},//pitchend拔出来
        { 10000,   59.92f,  20.65f,  26.75f,   -40.86f,  196.248f,  -127.427f,         5.f,        0 ,      1.0f},
        { 11000,   -2.02f,  15.45f,  20.75f,   -1.86f,  189.248f,  -76.427f,         0.f,        0 ,      1.3f},


    };
    const int Traj_GetLen = sizeof(Traj_Get) / sizeof(Traj_Get[0]); //计算帧数
    /*------------------------------ 定义对应的轨迹图 -----------------------------------*/
    // 将取矿加入到图中
    std::map<ETrajID,TrajClip> TrajMap = {
        {TRAJ_GRAB_L,{Traj_Grab,Traj_GrabLen}},
        {TRAJ_GET_L,{Traj_Get,Traj_GetLen}},
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

    // 提取第 row 行的夹爪状态：0=夹紧, 非0=松开
    bool ExtractGripClose(const float_t traj[][FC_COUNT], int row) {
        return traj[row][FC_GRIP] < 1.0f;
    }
    // 提取第 row 行的关节速度状态
    float_t ExtractSpeed(const float_t traj[][FC_COUNT], int row) {
        return traj[row][FC_SPEED];
    }

    /** @brief 轨迹播放器
     *  @param arm 臂的控制和信息参数
     *  @param target 目标关节角度
     *  @param gripClose 夹爪是否闭上
     *  @param player 播放器的内部速度参数定义
     */
    bool PlaySegment(CModArm &arm, const float_t target[J::COUNT],
                           float_t speedScale,bool gripClose,
                           CAlgoTrajPlayback &player, bool checkctrl){
        
        /*-----------------------  功能函数  ---------------------------*/
        // 夹爪控制，true=夹紧, false=松开
        auto updateGrip = [&arm](bool close) {
            const float_t gripSpeed = 160.0f;
            const float_t freq = 1000.0f;
            float_t &grip = arm.armCmd.set_length_grip;
            if (close) {
                grip -= gripSpeed / freq;
                if (grip < 0.0f) grip = 0.0f;
            } else {
                grip += gripSpeed / freq;
                if (grip > 65.0f) grip = 65.0f;
            }
        };

        // 夹爪到位判断
        auto gripReached = [&arm](bool close) -> bool {
            const float_t tol = 2.0f;
            float_t grip = arm.armCmd.set_length_grip;
            return close ? (grip <= tol) : (grip >= 65.0f - tol);
        };
        /*-------------------------------------------------------*/

        float_t current[J::COUNT];
        float_t joints[J::COUNT];
        ReadArmjoint(arm,current);//将当前的关节角度读取到current数组中

        player.speedScale = speedScale;//设置当前的速度
        player.PlanMultiAxisTraj(current, target);//计算出总的运动的时间

        int32_t startTick = HAL_GetTick();//获取时间轴

        while(true){

            // ctrl+z操作手打断回放避免实际位姿错误或者出现干涉
            if(checkctrl && SysRemote.remoteInfo.keyboard.key_Ctrl
               && SysRemote.remoteInfo.keyboard.key_Z) {
                return false;
            }

            float_t elapsed = static_cast<float_t>(HAL_GetTick() - startTick) / 1000.0f;//转换成当前秒数

            // 关节角度播放
            if(player.IsFinished(elapsed)) {
                WriteArmjoint(arm, target);
            } else {
                player.MultiAxisTrajDistance(elapsed, joints);//根据当前的秒数度取关节的信息
                WriteArmjoint(arm, joints);//将获取到的关节信息反写入arm中
            }

            updateGrip(gripClose);

            // 关节到位且夹爪到位才退出
            if(player.IsFinished(elapsed) && gripReached(gripClose)) break;

            proc_waitMs(1);
        }

        WriteArmjoint(arm, target);
        return true;
    }
    

    //再原来的播放器的基础上再封装一个速度读取的函数
    bool PlayFrameSegment(CModArm &arm,
                                const float_t traj[][FC_COUNT], int seg,
                                CAlgoTrajPlayback &player, bool checkctrl){
        float_t target[J::COUNT];
        bool gripClose = ExtractGripClose(traj, seg);
        float_t speed  = ExtractSpeed(traj, seg);
        Extrarow(traj, seg, target);//这个函数以上的作用只是控制夹爪的闭合
        return PlaySegment(arm, target, speed, gripClose, player, checkctrl);//这个函数才是控制关节的运动
    }


 }// namespace my_engineer
