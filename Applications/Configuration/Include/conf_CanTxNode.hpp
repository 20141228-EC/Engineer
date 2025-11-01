/*
 * @Description: 
 * @Author: Sassinak
 * @version: 
 * @Date: 2025-02-14 15:10:36
 * @LastEditors: Sassinak
 * @LastEditTime: 2025-07-13 10:48:49
 */
/**
 * @file conf_CanTxNode.hpp
 * @author Fish_Joe (2328339747@qq.com)
 * @brief 
 * @version 1.0
 * @date 2024-11-21
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef CONF_CANTXNODE_HPP
#define CONF_CANTXNODE_HPP

#include "Interface.hpp"
#include "conf_common.hpp"

namespace my_engineer {

extern CInfCAN::CCanTxNode TxNode_Can1_200;
extern CInfCAN::CCanTxNode TxNode_Can1_1FF;
extern CInfCAN::CCanTxNode TxNode_Can1_3FE;
extern CInfCAN::CCanTxNode TxNode_Can1_280;
extern CInfCAN::CCanTxNode TxNode_Can2_200;
extern CInfCAN::CCanTxNode TxNode_Can2_1FF;
extern CInfCAN::CCanTxNode TxNode_Can2_3FE;
extern CInfCAN::CCanTxNode TxNode_Can2_280;
extern CInfCAN::CCanTxNode TxNode_Can3_200;
extern CInfCAN::CCanTxNode TxNode_Can3_1FF;
extern CInfCAN::CCanTxNode TxNode_Can3_3FE;
extern CInfCAN::CCanTxNode TxNode_Can3_280;

extern CInfCAN::CCanTxNode MitTxNode_Can1_30;
extern CInfCAN::CCanTxNode MitTxNode_Can1_32;
extern CInfCAN::CCanTxNode MitTxNode_Can2_30;
extern CInfCAN::CCanTxNode MitTxNode_Can2_32;
extern CInfCAN::CCanTxNode MitTxNode_Can3_30;
extern CInfCAN::CCanTxNode MitTxNode_Can3_32;

EAppStatus InitAllCanTxNode();

}


#endif // CONF_CANTXNODE_HPP
