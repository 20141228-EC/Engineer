/**
 * @file conf_CanTxNode.hpp
 * @author Sassinak, Fish_Joe, Ciallo
 * @brief CAN发送节点配置
 * @version 1.1
 * @date 2024-11-21
 * @LastEditors Ciallo(1002046597@qq.com)
 * @LastEditTime 2026-01-15
 *
 * @details
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
/*----------- Roll和PitchEnd的MIT发送节点（DM3510） -----------*/
extern CInfCAN::CCanTxNode MitTxNode_Can2_34;
extern CInfCAN::CCanTxNode MitTxNode_Can2_36;
extern CInfCAN::CCanTxNode MitTxNode_Can3_34;
extern CInfCAN::CCanTxNode MitTxNode_Can3_36;
extern CInfCAN::CCanTxNode MitTxNode_Can1_39;

EAppStatus InitAllCanTxNode();

}


#endif // CONF_CANTXNODE_HPP
