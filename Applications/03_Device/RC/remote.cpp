/**
 * @file remote.cpp
 * @author sllllr (2997708711@qq.com)
 * @brief 遥控器的通用控制层的实现
 * @version 1.0
 * @date 2026-01-11
 * 
 * @copyright Copyright (c) 2026
 * 
 * @details
 * 因为不管用什么遥控器，都需要处理心跳和通道状态初始化和更新
 * 所以在这个文件里面定义这些通用的方法
 */

#include "rc/remote.hpp"

namespace my_engineer {

/**
 * @brief 心跳处理
 * 
 */
void CRcBase::HeartbeatHandler_(){

    if(deviceStatus == APP_RESET) return;

    // 检查遥控器是否在线(根据是否在100ms内收到数据包来判断)
    rcStatus = ((HAL_GetTick() - lastHeartbeatTime_) > 100) ? ERcStatus::OFFLINE : ERcStatus::ONLINE;

    // 原来的通道信息更新函数是放在这里的，但由于心跳的更新是100Hz，而系统层获取边沿是1KHz，毫无疑问会导致丢信息
    // 所以把通道更新放在派生类的updatehandler里面
    
}

/**
 * @brief 初始化单个通道
 * @param channelNum - 通道号
 * @param chType - 通道类型
 * @return EAppStatus - 返回初始化状态
 */
EAppStatus CRcBase::InitChannel_(size_t channelNum, ERcChannelType chType){
    // 检查通道号是否合法
    if(channelNum >= remoteData.size()) return APP_ERROR;

    // 初始化通道
    remoteData[channelNum].chType = chType;
    remoteData[channelNum].chStatus = ERcChannelStatus::RESET;
    remoteData[channelNum].chValue = 0;
    remoteData[channelNum].chEdge = ERcChannelEdge::NONE;

    return APP_OK;
}

/**
 * @brief 更新所有的通道状态
 * @return EAppStatus - 返回更新状态
 */
EAppStatus CRcBase::UpdateChannels_(){

    enum{HIG = 1,MID = 3,LOW = 2};

    // 检查设备状态
    if(deviceStatus == APP_RESET) return APP_ERROR;

    // 使用带索引的 for 循环
    for(size_t i = 0; i < remoteData.size(); ++i)
    {
        auto& channel = remoteData[i]; // 当前通道的引用
        auto& last_channel = last_remoteData[i]; // 上一次状态的通道的引用

        channel.chEdge = ERcChannelEdge::NONE;

        // 仅当值发生变化时才检测边沿
        if(channel.chValue == last_channel.chValue) {
            continue; // 值未变 跳过该通道的边沿检测
        }

        // 先将通道状态设置为重置
        channel.chStatus = ERcChannelStatus::RESET;
        // 更新通道状态
        switch (channel.chType)
        {
            case ERcChannelType::BUTTON:
                if (channel == 1) 
                {
                    channel.chStatus = ERcChannelStatus::PRESS;
                    channel.chEdge = ERcChannelEdge::Rising;
                }
                else
                {
                    channel.chEdge = ERcChannelEdge::Falling;
                }
                break;
            case ERcChannelType::LEVER:
                // 摇杆类型的通道，绝对值大于一定值时，通道状态有效
                if (channel > 50) channel.chStatus = ERcChannelStatus::HIGH;
                else if (channel < -50) channel.chStatus = ERcChannelStatus::DOWN;

                // if(channel.chValue != last_channel.chValue){

                    if(abs(channel.chValue) > 330) {    ///< 推过一半
                    if(last_channel.chValue > 0 && last_channel.chValue < 330) {
                        channel.chEdge = ERcChannelEdge::Rising;    ///< 上升沿 
                    }
                    if(last_channel.chValue < 0 && last_channel.chValue > -330) {
                        channel.chEdge = ERcChannelEdge::Falling;    ///< 下降沿 
                    }
                // }
                }
                // else{
                //     channel.chEdge = ERcChannelEdge::NONE;
                // }

                break;
            case ERcChannelType::SWITCH:
                // if(channel.chValue != last_channel.chValue){
                    switch (channel.chValue)
                    {
                    case HIG:
                        channel.chEdge = ERcChannelEdge::Rising;    ///< 向上拨
                        break;
                    
                    case LOW:
                        channel.chEdge = ERcChannelEdge::Falling;   ///< 向下拨
                        break;
                    default:
                        break;
                    }
                // }
                // else{
                //     channel.chEdge = ERcChannelEdge::NONE;
                // }
                
            default:
                break;
        }
    }
    last_remoteData = remoteData;   ///< 记录上一次遥控器信息

    return APP_OK;
}

} // namespace my_engineer