/******************************************************************************
 * @brief        
 * 
 * @file         referee_ui.cpp
 * @author       sllllr (2997708711@qq.com)
 * @version      V1.0
 * @date         2026-03-16
 * @note         Start...为前缀的是静态ui，Update...为前缀的是动态ui
 * 
 * @copyright    Copyright (c) 2026
 * 
 ******************************************************************************/

#include "Core.hpp"

namespace my_engineer {

// 初始化UI
void CSystemReferee::UI_InitDrawing() {

  // 底盘信息初始化
  chassis = reinterpret_cast<CModChassis *>(ModuleIDMap.at(EModuleID::MOD_CHASSIS))->chassisInfo;

  /* Text - Hip Config */
  hipTextMsg.header = CDevReferee::SPkgHeader();
  hipTextMsg.header.len = sizeof(hipTextMsg) - 9;
  hipTextMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  hipTextMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&hipTextMsg.header), 4);
  hipTextMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  hipTextMsg.transmitterID += (refereeInfo.robot.robotID);
  hipTextMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  hipTextMsg.receiverID += (refereeInfo.robot.robotID);
  hipTextMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_TEXT;
  hipTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  hipTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  hipTextMsg.message.figureConfig.figureName[2] = 0;    // Figure ID
  hipTextMsg.message.figureConfig.operate = 1;
  hipTextMsg.message.figureConfig.figureType = 7;
  hipTextMsg.message.figureConfig.layerID = 0;
  hipTextMsg.message.figureConfig.details_1 = 20;       // Font Size
  hipTextMsg.message.figureConfig.posit_X = 1400; //1400
  hipTextMsg.message.figureConfig.posit_Y = 690,  // 790
  hipTextMsg.message.figureConfig.color = 4;
  hipTextMsg.message.figureConfig.details_2 = 11;        // String Length
  hipTextMsg.message.figureConfig.width = 2;            // Line Width
  strcpy(reinterpret_cast<char *>(hipTextMsg.message.text), "Hip_Length:");

  /* Text - Mode Config */
  modeTextMsg.header = CDevReferee::SPkgHeader();
  modeTextMsg.header.len = sizeof(modeTextMsg) - 9;
  modeTextMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  modeTextMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&modeTextMsg.header), 4);
  modeTextMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  modeTextMsg.transmitterID += (refereeInfo.robot.robotID);
  modeTextMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  modeTextMsg.receiverID += (refereeInfo.robot.robotID);
  modeTextMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_TEXT;
  modeTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  modeTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  modeTextMsg.message.figureConfig.figureName[2] = 1;    // Figure ID
  modeTextMsg.message.figureConfig.operate = 1;
  modeTextMsg.message.figureConfig.figureType = 7;
  modeTextMsg.message.figureConfig.layerID = 0;
  modeTextMsg.message.figureConfig.details_1 = 20;       // Font Size
  modeTextMsg.message.figureConfig.posit_X = 920;
  modeTextMsg.message.figureConfig.posit_Y = 820,
  modeTextMsg.message.figureConfig.color = 4;
  modeTextMsg.message.figureConfig.details_2 = 4;        // String Length
  modeTextMsg.message.figureConfig.width = 2;            // Line Width
  strcpy(reinterpret_cast<char *>(modeTextMsg.message.text), "MODE");

  /* Text - CurMode Config */
  curModeTextMsg.header = CDevReferee::SPkgHeader();
  curModeTextMsg.header.len = sizeof(curModeTextMsg) - 9;
  curModeTextMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  curModeTextMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&curModeTextMsg.header), 4);
  curModeTextMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  curModeTextMsg.transmitterID += (refereeInfo.robot.robotID);
  curModeTextMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  curModeTextMsg.receiverID += (refereeInfo.robot.robotID);
  curModeTextMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_TEXT;
  curModeTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  curModeTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  curModeTextMsg.message.figureConfig.figureName[2] = 2;    // Figure ID
  curModeTextMsg.message.figureConfig.operate = 1;
  curModeTextMsg.message.figureConfig.figureType = 7;
  curModeTextMsg.message.figureConfig.layerID = 0;
  curModeTextMsg.message.figureConfig.posit_X = 960 - (25 * 2);
  curModeTextMsg.message.figureConfig.posit_Y = 780;
  curModeTextMsg.message.figureConfig.color = 1;
  curModeTextMsg.message.figureConfig.details_1 = 25;       // Font Size
  curModeTextMsg.message.figureConfig.details_2 = 4;        // String Length
  curModeTextMsg.message.figureConfig.width = 4;            // Line Width
  strcpy(reinterpret_cast<char *>(curModeTextMsg.message.text), "NONE");

  /* Text - Radar Config */
  RadarTextMsg.header = CDevReferee::SPkgHeader();
  RadarTextMsg.header.len = sizeof(RadarTextMsg) - 9;
  RadarTextMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  RadarTextMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&RadarTextMsg.header), 4);
  RadarTextMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  RadarTextMsg.transmitterID += (refereeInfo.robot.robotID);
  RadarTextMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  RadarTextMsg.receiverID += (refereeInfo.robot.robotID);
  RadarTextMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_TEXT;
  RadarTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  RadarTextMsg.message.figureConfig.figureName[1] = 0;    // Layer
  RadarTextMsg.message.figureConfig.figureName[2] = 3;    // Figure ID
  RadarTextMsg.message.figureConfig.operate = 1;
  RadarTextMsg.message.figureConfig.figureType = 7;
  RadarTextMsg.message.figureConfig.layerID = 1;
  RadarTextMsg.message.figureConfig.details_1 = 25;       // Font Size
  RadarTextMsg.message.figureConfig.posit_X = 960;
  RadarTextMsg.message.figureConfig.posit_Y = 540 - 25;
  RadarTextMsg.message.figureConfig.color = 3;
  RadarTextMsg.message.figureConfig.details_2 = 0;        // String Length
  RadarTextMsg.message.figureConfig.width = 4;            // Line Width
  strcpy(reinterpret_cast<char *>(RadarTextMsg.message.text), "");

  /* Text - Crawler Config */
  crawlerTextMsg.header = CDevReferee::SPkgHeader();
  crawlerTextMsg.header.len = sizeof(crawlerTextMsg) - 9;
  crawlerTextMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  crawlerTextMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&crawlerTextMsg.header), 4);
  crawlerTextMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  crawlerTextMsg.transmitterID += (refereeInfo.robot.robotID);
  crawlerTextMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  crawlerTextMsg.receiverID += (refereeInfo.robot.robotID);
  crawlerTextMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_TEXT;
  crawlerTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  crawlerTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  crawlerTextMsg.message.figureConfig.figureName[2] = 7;    // Figure ID
  crawlerTextMsg.message.figureConfig.operate = 1;
  crawlerTextMsg.message.figureConfig.figureType = 7;
  crawlerTextMsg.message.figureConfig.layerID = 0;
  crawlerTextMsg.message.figureConfig.details_1 = 20;       // Font Size
  crawlerTextMsg.message.figureConfig.posit_X = 1400;
  crawlerTextMsg.message.figureConfig.posit_Y = 840;
  crawlerTextMsg.message.figureConfig.color = 4;
  crawlerTextMsg.message.figureConfig.details_2 = 11;        // String Length
  crawlerTextMsg.message.figureConfig.width = 2;            // Line Width
  strcpy(reinterpret_cast<char *>(crawlerTextMsg.message.text), "Crawler_On:");  

  /* Text - p3LockConfig */
  p3LockTextMsg.header = CDevReferee::SPkgHeader();
  p3LockTextMsg.header.len = sizeof(p3LockTextMsg) - 9;
  p3LockTextMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  p3LockTextMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&p3LockTextMsg.header), 4);
  p3LockTextMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  p3LockTextMsg.transmitterID += (refereeInfo.robot.robotID);
  p3LockTextMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  p3LockTextMsg.receiverID += (refereeInfo.robot.robotID);
  p3LockTextMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_TEXT;
  p3LockTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  p3LockTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  p3LockTextMsg.message.figureConfig.figureName[2] = 9;    // Figure ID
  p3LockTextMsg.message.figureConfig.operate = 1;
  p3LockTextMsg.message.figureConfig.figureType = 7;
  p3LockTextMsg.message.figureConfig.layerID = 0;
  p3LockTextMsg.message.figureConfig.details_1 = 20;       // Font Size
  p3LockTextMsg.message.figureConfig.posit_X = 1400;
  p3LockTextMsg.message.figureConfig.posit_Y = 790;
  p3LockTextMsg.message.figureConfig.color = 4;
  p3LockTextMsg.message.figureConfig.details_2 = 8;        // String Length
  p3LockTextMsg.message.figureConfig.width = 2;            // Line Width
  strcpy(reinterpret_cast<char *>(p3LockTextMsg.message.text), "P3_Enable:");

  p3LockMsg.header = CDevReferee::SPkgHeader();
  p3LockMsg.header.len = sizeof(p3LockMsg) - 9;
  p3LockMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  p3LockMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&p3LockMsg.header), 4);
  p3LockMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  p3LockMsg.transmitterID += (refereeInfo.robot.robotID);
  p3LockMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  p3LockMsg.receiverID += (refereeInfo.robot.robotID);
  p3LockMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_SINGLE;
  p3LockMsg.message.figureConfig[0].figureName[0] = 0;
  p3LockMsg.message.figureConfig[0].figureName[1] = 0;
  p3LockMsg.message.figureConfig[0].figureName[2] = 10;
  p3LockMsg.message.figureConfig[0].operate = 1;
  p3LockMsg.message.figureConfig[0].figureType = 2; // circle
  p3LockMsg.message.figureConfig[0].layerID = 0;
  p3LockMsg.message.figureConfig[0].posit_X = 1620;
  p3LockMsg.message.figureConfig[0].posit_Y = 780;
  p3LockMsg.message.figureConfig[0].color = 7;
  p3LockMsg.message.figureConfig[0].details_3 = 10; // radius
  p3LockMsg.message.figureConfig[0].width = 14;

  /* Text - gripCloseConfig */
  gripCloseTextMsg.header = CDevReferee::SPkgHeader();
  gripCloseTextMsg.header.len = sizeof(gripCloseTextMsg) - 9;
  gripCloseTextMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  gripCloseTextMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&gripCloseTextMsg.header), 4);
  gripCloseTextMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  gripCloseTextMsg.transmitterID += (refereeInfo.robot.robotID);
  gripCloseTextMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  gripCloseTextMsg.receiverID += (refereeInfo.robot.robotID);
  gripCloseTextMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_TEXT;
  gripCloseTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  gripCloseTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  gripCloseTextMsg.message.figureConfig.figureName[2] = 11;    // Figure ID
  gripCloseTextMsg.message.figureConfig.operate = 1;
  gripCloseTextMsg.message.figureConfig.figureType = 7;
  gripCloseTextMsg.message.figureConfig.layerID = 0;
  gripCloseTextMsg.message.figureConfig.details_1 = 20;       // Font Size
  gripCloseTextMsg.message.figureConfig.posit_X = 1400;
  gripCloseTextMsg.message.figureConfig.posit_Y = 740;
  gripCloseTextMsg.message.figureConfig.color = 4;
  gripCloseTextMsg.message.figureConfig.details_2 = 10;        // String Length
  gripCloseTextMsg.message.figureConfig.width = 2;            // Line Width
  strcpy(reinterpret_cast<char *>(gripCloseTextMsg.message.text), "Grip_Close:");

  gripCloseMsg.header = CDevReferee::SPkgHeader();
  gripCloseMsg.header.len = sizeof(gripCloseMsg) - 9;
  gripCloseMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  gripCloseMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&gripCloseMsg.header), 4);
  gripCloseMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  gripCloseMsg.transmitterID += (refereeInfo.robot.robotID);
  gripCloseMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  gripCloseMsg.receiverID += (refereeInfo.robot.robotID);
  gripCloseMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_SINGLE;
  gripCloseMsg.message.figureConfig[0].figureName[0] = 0;
  gripCloseMsg.message.figureConfig[0].figureName[1] = 0;
  gripCloseMsg.message.figureConfig[0].figureName[2] = 12;
  gripCloseMsg.message.figureConfig[0].operate = 1;
  gripCloseMsg.message.figureConfig[0].figureType = 2; // circle
  gripCloseMsg.message.figureConfig[0].layerID = 0;
  gripCloseMsg.message.figureConfig[0].posit_X = 1640;
  gripCloseMsg.message.figureConfig[0].posit_Y = 730;
  gripCloseMsg.message.figureConfig[0].color = 7;
  gripCloseMsg.message.figureConfig[0].details_3 = 10; // radius
  gripCloseMsg.message.figureConfig[0].width = 14;

  /* Text - hipInfo Config */
  hipInfoTextMsg.header = CDevReferee::SPkgHeader();
  hipInfoTextMsg.header.len = sizeof(hipInfoTextMsg) - 9;
  hipInfoTextMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  hipInfoTextMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&hipInfoTextMsg.header), 4);
  hipInfoTextMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  hipInfoTextMsg.transmitterID += (refereeInfo.robot.robotID);
  hipInfoTextMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  hipInfoTextMsg.receiverID += (refereeInfo.robot.robotID);
  hipInfoTextMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_TEXT;
  hipInfoTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  hipInfoTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  hipInfoTextMsg.message.figureConfig.figureName[2] = 4;    // Figure ID
  hipInfoTextMsg.message.figureConfig.operate = 1;
  hipInfoTextMsg.message.figureConfig.figureType = 5;
  hipInfoTextMsg.message.figureConfig.layerID = 0;
  hipInfoTextMsg.message.figureConfig.details_1 = 20;       // Font Size
  hipInfoTextMsg.message.figureConfig.posit_X = 1620;
  hipInfoTextMsg.message.figureConfig.posit_Y = 690,  // 790
  hipInfoTextMsg.message.figureConfig.color = 4;
  // hipInfoTextMsg.message.figureConfig.details_3 = chassis.L_Length * 1000.f;
  hipInfoTextMsg.message.figureConfig.width = 2;            // Line Width
  // sprintf(reinterpret_cast<char *>(hipInfoTextMsg.message.text), "%.2f", chassis.L_Length * 1000);

  /* Text - Pitch Config */
  pitchTextMsg.header = CDevReferee::SPkgHeader();
  pitchTextMsg.header.len = sizeof(pitchTextMsg) - 9;
  pitchTextMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  pitchTextMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&pitchTextMsg.header), 4);
  pitchTextMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  pitchTextMsg.transmitterID += (refereeInfo.robot.robotID);
  pitchTextMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  pitchTextMsg.receiverID += (refereeInfo.robot.robotID);
  pitchTextMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_TEXT;
  pitchTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  pitchTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  pitchTextMsg.message.figureConfig.figureName[2] = 5;    // Figure ID
  pitchTextMsg.message.figureConfig.operate = 1;
  pitchTextMsg.message.figureConfig.figureType = 5;
  pitchTextMsg.message.figureConfig.layerID = 0;
  pitchTextMsg.message.figureConfig.details_1 = 20;       // Font Size
  pitchTextMsg.message.figureConfig.details_3 = chassis.roll_Measure[0] * 1000.f;
  pitchTextMsg.message.figureConfig.posit_X = 1520;
  pitchTextMsg.message.figureConfig.posit_Y = 640,  // 700
  pitchTextMsg.message.figureConfig.color = 4;
  pitchTextMsg.message.figureConfig.width = 2;            // Line Width
  // sprintf(reinterpret_cast<char *>(pitchTextMsg.message.text), "%.2f", chassis.roll_Measure[0] * 1000);

  /* Text - Mode Config */
  pitchStaticTextMsg.header = CDevReferee::SPkgHeader();
  pitchStaticTextMsg.header.len = sizeof(pitchStaticTextMsg) - 9;
  pitchStaticTextMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  pitchStaticTextMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&pitchStaticTextMsg.header), 4);
  pitchStaticTextMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  pitchStaticTextMsg.transmitterID += (refereeInfo.robot.robotID);
  pitchStaticTextMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  pitchStaticTextMsg.receiverID += (refereeInfo.robot.robotID);
  pitchStaticTextMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_TEXT;
  pitchStaticTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  pitchStaticTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  pitchStaticTextMsg.message.figureConfig.figureName[2] = 6;    // Figure ID
  pitchStaticTextMsg.message.figureConfig.operate = 1;
  pitchStaticTextMsg.message.figureConfig.figureType = 7;
  pitchStaticTextMsg.message.figureConfig.layerID = 0;
  pitchStaticTextMsg.message.figureConfig.details_1 = 20;       // Font Size
  pitchStaticTextMsg.message.figureConfig.posit_X = 1400;
  pitchStaticTextMsg.message.figureConfig.posit_Y = 640,  //750
  pitchStaticTextMsg.message.figureConfig.color = 4;
  pitchStaticTextMsg.message.figureConfig.details_2 = 6;        // String Length
  pitchStaticTextMsg.message.figureConfig.width = 2;            // Line Width
  strcpy(reinterpret_cast<char *>(pitchStaticTextMsg.message.text), "PITCH:");

  /* Figure - State Config */
  stateFigureMsg.header = CDevReferee::SPkgHeader();
  stateFigureMsg.header.len = sizeof(stateFigureMsg) - 9;
  stateFigureMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  stateFigureMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&stateFigureMsg.header), 4);
  stateFigureMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  stateFigureMsg.transmitterID += (refereeInfo.robot.robotID);
  stateFigureMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  stateFigureMsg.receiverID += (refereeInfo.robot.robotID);
  stateFigureMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_HEPTA;

  // 履带状态
  stateFigureMsg.message.figureConfig[0].figureName[0] = 0;    // Frame ID
  stateFigureMsg.message.figureConfig[0].figureName[1] = 1;    // Layer ID
  stateFigureMsg.message.figureConfig[0].figureName[2] = 3;    // Figure ID
  stateFigureMsg.message.figureConfig[0].operate = 1;
  stateFigureMsg.message.figureConfig[0].figureType = 2;
  stateFigureMsg.message.figureConfig[0].layerID = 1;
  stateFigureMsg.message.figureConfig[0].posit_X = 1640;
  stateFigureMsg.message.figureConfig[0].posit_Y = 830,
  stateFigureMsg.message.figureConfig[0].color = 7;
  stateFigureMsg.message.figureConfig[0].details_3 = 10;       // Radius
  stateFigureMsg.message.figureConfig[0].width = 14;           // Line Width

  // 取矿六边形（逆时针）
  stateFigureMsg.message.figureConfig[1].figureName[0] = 0;    // Frame ID
  stateFigureMsg.message.figureConfig[1].figureName[1] = 1;    // Layer ID
  stateFigureMsg.message.figureConfig[1].figureName[2] = 4;    // Figure ID
  stateFigureMsg.message.figureConfig[1].operate = 1;
  stateFigureMsg.message.figureConfig[1].figureType = 0;
  stateFigureMsg.message.figureConfig[1].layerID = 1;
  stateFigureMsg.message.figureConfig[1].posit_X = 960 + 60;
  stateFigureMsg.message.figureConfig[1].posit_Y = 540 + 104,
  stateFigureMsg.message.figureConfig[1].color = 8;
  stateFigureMsg.message.figureConfig[1].details_4 = 960 - 60;      // End Posit X
  stateFigureMsg.message.figureConfig[1].details_5 = 540 + 104;      // End Posit Y
  stateFigureMsg.message.figureConfig[1].width = 2;            // Line Width
  
  stateFigureMsg.message.figureConfig[2].figureName[0] = 0;    // Frame ID
  stateFigureMsg.message.figureConfig[2].figureName[1] = 1;    // Layer ID
  stateFigureMsg.message.figureConfig[2].figureName[2] = 5;    // Figure ID
  stateFigureMsg.message.figureConfig[2].operate = 1;
  stateFigureMsg.message.figureConfig[2].figureType = 0;
  stateFigureMsg.message.figureConfig[2].layerID = 1;
  stateFigureMsg.message.figureConfig[2].posit_X = 960 - 60;
  stateFigureMsg.message.figureConfig[2].posit_Y = 540 + 104,
  stateFigureMsg.message.figureConfig[2].color = 8;
  stateFigureMsg.message.figureConfig[2].details_4 = 960 - 120;      // End Posit X
  stateFigureMsg.message.figureConfig[2].details_5 = 540;      // End Posit Y
  stateFigureMsg.message.figureConfig[2].width = 2;            // Line Width

  stateFigureMsg.message.figureConfig[3].figureName[0] = 0;    // Frame ID
  stateFigureMsg.message.figureConfig[3].figureName[1] = 1;    // Layer ID
  stateFigureMsg.message.figureConfig[3].figureName[2] = 6;    // Figure ID
  stateFigureMsg.message.figureConfig[3].operate = 1;
  stateFigureMsg.message.figureConfig[3].figureType = 0;
  stateFigureMsg.message.figureConfig[3].layerID = 1;
  stateFigureMsg.message.figureConfig[3].posit_X = 960 - 120;
  stateFigureMsg.message.figureConfig[3].posit_Y = 540,
  stateFigureMsg.message.figureConfig[3].color = 8;
  stateFigureMsg.message.figureConfig[3].details_4 = 960 - 60;      // End Posit X
  stateFigureMsg.message.figureConfig[3].details_5 = 540 - 104;      // End Posit Y
  stateFigureMsg.message.figureConfig[3].width = 2;            // Line Width

  stateFigureMsg.message.figureConfig[4].figureName[0] = 0;    // Frame ID
  stateFigureMsg.message.figureConfig[4].figureName[1] = 1;    // Layer ID
  stateFigureMsg.message.figureConfig[4].figureName[2] = 7;    // Figure ID
  stateFigureMsg.message.figureConfig[4].operate = 1;
  stateFigureMsg.message.figureConfig[4].figureType = 0;
  stateFigureMsg.message.figureConfig[4].layerID = 1;
  stateFigureMsg.message.figureConfig[4].posit_X = 960 - 300;
  stateFigureMsg.message.figureConfig[4].posit_Y = 540 - 104,
  stateFigureMsg.message.figureConfig[4].color = 0;
  stateFigureMsg.message.figureConfig[4].details_4 = 960 + 300;      // End Posit X
  stateFigureMsg.message.figureConfig[4].details_5 = 540 - 104;      // End Posit Y
  stateFigureMsg.message.figureConfig[4].width = 2;            // Line Width

  stateFigureMsg.message.figureConfig[5].figureName[0] = 0;    // Frame ID
  stateFigureMsg.message.figureConfig[5].figureName[1] = 1;    // Layer ID
  stateFigureMsg.message.figureConfig[5].figureName[2] = 8;    // Figure ID
  stateFigureMsg.message.figureConfig[5].operate = 1;
  stateFigureMsg.message.figureConfig[5].figureType = 0;
  stateFigureMsg.message.figureConfig[5].layerID = 1;
  stateFigureMsg.message.figureConfig[5].posit_X = 960 + 60;
  stateFigureMsg.message.figureConfig[5].posit_Y = 540 - 104,
  stateFigureMsg.message.figureConfig[5].color = 8;
  stateFigureMsg.message.figureConfig[5].details_4 = 960 + 120;      // End Posit X
  stateFigureMsg.message.figureConfig[5].details_5 = 540;      // End Posit Y
  stateFigureMsg.message.figureConfig[5].width = 2;            // Line Width

  stateFigureMsg.message.figureConfig[6].figureName[0] = 0;    // Frame ID
  stateFigureMsg.message.figureConfig[6].figureName[1] = 1;    // Layer ID
  stateFigureMsg.message.figureConfig[6].figureName[2] = 9;    // Figure ID
  stateFigureMsg.message.figureConfig[6].operate = 1;
  stateFigureMsg.message.figureConfig[6].figureType = 0;
  stateFigureMsg.message.figureConfig[6].layerID = 1;
  stateFigureMsg.message.figureConfig[6].posit_X = 960 + 120;
  stateFigureMsg.message.figureConfig[6].posit_Y = 540;
  stateFigureMsg.message.figureConfig[6].color = 8;
  stateFigureMsg.message.figureConfig[6].details_4 = 960 + 60;      // End Posit X
  stateFigureMsg.message.figureConfig[6].details_5 = 540 + 104;      // End Posit Y
  stateFigureMsg.message.figureConfig[6].width = 2;            // Line Width

  /* Figure - Position Config */
  positionFigureMsg.header = CDevReferee::SPkgHeader();
  positionFigureMsg.header.len = sizeof(positionFigureMsg) - 9;
  positionFigureMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  positionFigureMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&positionFigureMsg.header), 4);
  positionFigureMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  positionFigureMsg.transmitterID += (refereeInfo.robot.robotID);
  positionFigureMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  positionFigureMsg.receiverID += (refereeInfo.robot.robotID);
  positionFigureMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_DOUBLE;

  // 车身姿态示意图圆形边框
  positionFigureMsg.message.figureConfig[0].figureName[0] = 0;  // Frame ID
  positionFigureMsg.message.figureConfig[0].figureName[1] = 2;  // Layer ID
  positionFigureMsg.message.figureConfig[0].figureName[2] = 0;  // Figure ID
  positionFigureMsg.message.figureConfig[0].operate = 1;
  positionFigureMsg.message.figureConfig[0].figureType = 2;
  positionFigureMsg.message.figureConfig[0].layerID = 2;
  positionFigureMsg.message.figureConfig[0].posit_X = 960;
  positionFigureMsg.message.figureConfig[0].posit_Y = 200;
  positionFigureMsg.message.figureConfig[0].color = 8;
  positionFigureMsg.message.figureConfig[0].details_3 = 80;      // Radius
  positionFigureMsg.message.figureConfig[0].width = 2;            // Line Width

  // 车身姿态示意图直线
  positionFigureMsg.message.figureConfig[1].figureName[0] = 0;    // Frame ID
  positionFigureMsg.message.figureConfig[1].figureName[1] = 2;    // Layer ID
  positionFigureMsg.message.figureConfig[1].figureName[2] = 1;    // Figure ID
  positionFigureMsg.message.figureConfig[1].operate = 1;
  positionFigureMsg.message.figureConfig[1].figureType = 0;
  positionFigureMsg.message.figureConfig[1].layerID = 1;
  positionFigureMsg.message.figureConfig[1].posit_X = 880;
  positionFigureMsg.message.figureConfig[1].posit_Y = 200;
  positionFigureMsg.message.figureConfig[1].color = 8;
  positionFigureMsg.message.figureConfig[1].details_4 = 1040;      // End Posit X
  positionFigureMsg.message.figureConfig[1].details_5 = 200;      // End Posit Y
  positionFigureMsg.message.figureConfig[1].width = 2;            // Line Width

  // 前进方向示意图直线
  parallelFigureMsg.header = CDevReferee::SPkgHeader();
  parallelFigureMsg.header.len = sizeof(parallelFigureMsg) - 9;
  parallelFigureMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  parallelFigureMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&parallelFigureMsg.header), 4);
  parallelFigureMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  parallelFigureMsg.transmitterID += (refereeInfo.robot.robotID);
  parallelFigureMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  parallelFigureMsg.receiverID += (refereeInfo.robot.robotID);
  parallelFigureMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_DOUBLE;

  parallelFigureMsg.message.figureConfig[0].figureName[0] = 0;    // Frame ID
  parallelFigureMsg.message.figureConfig[0].figureName[1] = 2;    // Layer ID
  parallelFigureMsg.message.figureConfig[0].figureName[2] = 2;    // Figure ID
  parallelFigureMsg.message.figureConfig[0].operate = 1;
  parallelFigureMsg.message.figureConfig[0].figureType = 0;
  parallelFigureMsg.message.figureConfig[0].layerID = 3;
  parallelFigureMsg.message.figureConfig[0].posit_X = 960 - 420;
  parallelFigureMsg.message.figureConfig[0].posit_Y = 0;
  parallelFigureMsg.message.figureConfig[0].color = 8;
  parallelFigureMsg.message.figureConfig[0].details_4 = 960 - 200;      // End Posit X
  parallelFigureMsg.message.figureConfig[0].details_5 = 360;      // End Posit Y
  parallelFigureMsg.message.figureConfig[0].width = 2;            // Line Width

  parallelFigureMsg.message.figureConfig[1].figureName[0] = 0;    // Frame ID
  parallelFigureMsg.message.figureConfig[1].figureName[1] = 2;    // Layer ID
  parallelFigureMsg.message.figureConfig[1].figureName[2] = 3;    // Figure ID
  parallelFigureMsg.message.figureConfig[1].operate = 1;
  parallelFigureMsg.message.figureConfig[1].figureType = 0;
  parallelFigureMsg.message.figureConfig[1].layerID = 4;
  parallelFigureMsg.message.figureConfig[1].posit_X = 960 + 420;
  parallelFigureMsg.message.figureConfig[1].posit_Y = 0;
  parallelFigureMsg.message.figureConfig[1].color = 8;
  parallelFigureMsg.message.figureConfig[1].details_4 = 960 + 200;      // End Posit X
  parallelFigureMsg.message.figureConfig[1].details_5 = 360;      // End Posit Y
  parallelFigureMsg.message.figureConfig[1].width = 2;            // Line Width

  // 臂前三pitch示意图
  armAngleFigureMsg.header = CDevReferee::SPkgHeader();
  armAngleFigureMsg.header.len = sizeof(armAngleFigureMsg) - 9;
  armAngleFigureMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  armAngleFigureMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&armAngleFigureMsg.header), 4);
  armAngleFigureMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  armAngleFigureMsg.transmitterID += (refereeInfo.robot.robotID);
  armAngleFigureMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  armAngleFigureMsg.receiverID += (refereeInfo.robot.robotID);
  armAngleFigureMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_PENTA;

  armAngleFigureMsg.message.figureConfig[0].figureName[0] = 0;    // Frame ID
  armAngleFigureMsg.message.figureConfig[0].figureName[1] = 3;    // Layer ID
  armAngleFigureMsg.message.figureConfig[0].figureName[2] = 1;    // Figure ID
  armAngleFigureMsg.message.figureConfig[0].operate = 1;
  armAngleFigureMsg.message.figureConfig[0].figureType = 0;       // Line
  armAngleFigureMsg.message.figureConfig[0].layerID = 5;
  armAngleFigureMsg.message.figureConfig[0].color = 4;        
  armAngleFigureMsg.message.figureConfig[0].width = 4;            // Line Width
  armAngleFigureMsg.message.figureConfig[0].posit_X = 1500;
  armAngleFigureMsg.message.figureConfig[0].posit_Y = 550;
  armAngleFigureMsg.message.figureConfig[0].details_4 = 1500 + 120;
  armAngleFigureMsg.message.figureConfig[0].details_5 = 550;

  armAngleFigureMsg.message.figureConfig[1].figureName[0] = 0;    // Frame ID
  armAngleFigureMsg.message.figureConfig[1].figureName[1] = 3;    // Layer ID
  armAngleFigureMsg.message.figureConfig[1].figureName[2] = 2;    // Figure ID
  armAngleFigureMsg.message.figureConfig[1].operate = 1;
  armAngleFigureMsg.message.figureConfig[1].figureType = 0;       // Line
  armAngleFigureMsg.message.figureConfig[1].layerID = 5;
  armAngleFigureMsg.message.figureConfig[1].color = 5;        
  armAngleFigureMsg.message.figureConfig[1].width = 4;            // Line Width
  armAngleFigureMsg.message.figureConfig[1].posit_X = 1500 + 120;
  armAngleFigureMsg.message.figureConfig[1].posit_Y = 550;
  armAngleFigureMsg.message.figureConfig[1].details_4 = 1500 + 120 + 80;
  armAngleFigureMsg.message.figureConfig[1].details_5 = 550;

  armAngleFigureMsg.message.figureConfig[2].figureName[0] = 0;    // Frame ID
  armAngleFigureMsg.message.figureConfig[2].figureName[1] = 3;    // Layer ID
  armAngleFigureMsg.message.figureConfig[2].figureName[2] = 3;    // Figure ID
  armAngleFigureMsg.message.figureConfig[2].operate = 1;
  armAngleFigureMsg.message.figureConfig[2].figureType = 0;       // Line
  armAngleFigureMsg.message.figureConfig[2].layerID = 5;
  armAngleFigureMsg.message.figureConfig[2].color = 6;        
  armAngleFigureMsg.message.figureConfig[2].width = 4;            // Line Width
  armAngleFigureMsg.message.figureConfig[2].posit_X = 1500 + 120 + 80;
  armAngleFigureMsg.message.figureConfig[2].posit_Y = 550;
  armAngleFigureMsg.message.figureConfig[2].details_4 = 1500 + 120 + 80 + 50;
  armAngleFigureMsg.message.figureConfig[2].details_5 = 550;

  armAngleFigureMsg.message.figureConfig[3].operate = 0;
  armAngleFigureMsg.message.figureConfig[4].operate = 0;

  // 图传朝向与底盘朝向示意
  armYawFigureMsg.header = CDevReferee::SPkgHeader();
  armYawFigureMsg.header.len = sizeof(armYawFigureMsg) - 9;
  armYawFigureMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  armYawFigureMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&armYawFigureMsg.header), 4);
  armYawFigureMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  armYawFigureMsg.transmitterID += (refereeInfo.robot.robotID);
  armYawFigureMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  armYawFigureMsg.receiverID += (refereeInfo.robot.robotID);
  armYawFigureMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_PENTA;

  armYawFigureMsg.message.figureConfig[0].figureName[0] = 0;    // Frame ID
  armYawFigureMsg.message.figureConfig[0].figureName[1] = 4;    // Layer ID
  armYawFigureMsg.message.figureConfig[0].figureName[2] = 1;    // Figure ID
  armYawFigureMsg.message.figureConfig[0].operate = 1;
  armYawFigureMsg.message.figureConfig[0].figureType = 2;       // Circle
  armYawFigureMsg.message.figureConfig[0].layerID = 6;
  armYawFigureMsg.message.figureConfig[0].color = 8;            // White
  armYawFigureMsg.message.figureConfig[0].width = 2;            // Line Width
  armYawFigureMsg.message.figureConfig[0].posit_X = 200;
  armYawFigureMsg.message.figureConfig[0].posit_Y = 700;
  armYawFigureMsg.message.figureConfig[0].details_3 = 80;       // Radius

  armYawFigureMsg.message.figureConfig[1].figureName[0] = 0;    // Frame ID
  armYawFigureMsg.message.figureConfig[1].figureName[1] = 4;    // Layer ID
  armYawFigureMsg.message.figureConfig[1].figureName[2] = 2;    // Figure ID
  armYawFigureMsg.message.figureConfig[1].operate = 1;
  armYawFigureMsg.message.figureConfig[1].figureType = 0;       // Line
  armYawFigureMsg.message.figureConfig[1].layerID = 6;
  armYawFigureMsg.message.figureConfig[1].color = 4;            // Purplish Red
  armYawFigureMsg.message.figureConfig[1].width = 3;            // Line Width
  armYawFigureMsg.message.figureConfig[1].posit_X = 200;
  armYawFigureMsg.message.figureConfig[1].posit_Y = 700;
  armYawFigureMsg.message.figureConfig[1].details_4 = 200;
  armYawFigureMsg.message.figureConfig[1].details_5 = 700 + 80; // 90 degrees up

  armYawFigureMsg.message.figureConfig[2].figureName[0] = 0;    // Frame ID
  armYawFigureMsg.message.figureConfig[2].figureName[1] = 4;    // Layer ID
  armYawFigureMsg.message.figureConfig[2].figureName[2] = 3;    // Figure ID
  armYawFigureMsg.message.figureConfig[2].operate = 1;
  armYawFigureMsg.message.figureConfig[2].figureType = 0;       // Line
  armYawFigureMsg.message.figureConfig[2].layerID = 6;
  armYawFigureMsg.message.figureConfig[2].color = 8;            // White
  armYawFigureMsg.message.figureConfig[2].width = 3;            // Line Width
  armYawFigureMsg.message.figureConfig[2].posit_X = 200;
  armYawFigureMsg.message.figureConfig[2].posit_Y = 700;
  armYawFigureMsg.message.figureConfig[2].details_4 = 200;
  armYawFigureMsg.message.figureConfig[2].details_5 = 700 + 80;

  armYawFigureMsg.message.figureConfig[3].operate = 0;
  armYawFigureMsg.message.figureConfig[4].operate = 0;

  // /* Figure - Vision Config */ 
  // visionFigureMsg.header = CDevReferee::SPkgHeader();
  // visionFigureMsg.header.len = sizeof(visionFigureMsg) - 9;
  // visionFigureMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  // visionFigureMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&visionFigureMsg.header), 4);
  // visionFigureMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  // visionFigureMsg.transmitterID += (refereeInfo.robot.robotID);
  // visionFigureMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  // visionFigureMsg.receiverID += (refereeInfo.robot.robotID);
  // visionFigureMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_PENTA;

  // visionFigureMsg.message.figureConfig[0].figureName[0] = 0;    // Frame ID
  // visionFigureMsg.message.figureConfig[0].figureName[1] = 2;    // Layer ID
  // visionFigureMsg.message.figureConfig[0].figureName[2] = 10;    // Figure ID
  // visionFigureMsg.message.figureConfig[0].operate = 1;
  // visionFigureMsg.message.figureConfig[0].figureType = 1;
  // visionFigureMsg.message.figureConfig[0].layerID = 2;
  // visionFigureMsg.message.figureConfig[0].posit_X = 960 - 400;
  // visionFigureMsg.message.figureConfig[0].posit_Y = 540 + 300;
  // visionFigureMsg.message.figureConfig[0].color = 7;
  // visionFigureMsg.message.figureConfig[0].width = 2;
  // visionFigureMsg.message.figureConfig[0].details_4 = 960 + 400;        // End Posit X
  // visionFigureMsg.message.figureConfig[0].details_5 = 540 - 300;        // End Posit Y

  // visionFigureMsg.message.figureConfig[1].figureName[0] = 0;    // Frame ID
  // visionFigureMsg.message.figureConfig[1].figureName[1] = 2;    // Layer ID
  // visionFigureMsg.message.figureConfig[1].figureName[2] = 11;    // Figure ID
  // visionFigureMsg.message.figureConfig[1].operate = 1;
  // visionFigureMsg.message.figureConfig[1].figureType = 2;
  // visionFigureMsg.message.figureConfig[1].layerID = 2;
  // visionFigureMsg.message.figureConfig[1].posit_X = 960 + 100;
  // visionFigureMsg.message.figureConfig[1].posit_Y = 540 + 100,
  // visionFigureMsg.message.figureConfig[1].color = 4;
  // visionFigureMsg.message.figureConfig[1].details_3 = 6;       // Radius
  // visionFigureMsg.message.figureConfig[1].width = 8;           // Line Width

  // visionFigureMsg.message.figureConfig[2].figureName[0] = 0;    // Frame ID
  // visionFigureMsg.message.figureConfig[2].figureName[1] = 2;    // Layer ID
  // visionFigureMsg.message.figureConfig[2].figureName[2] = 12;    // Figure ID
  // visionFigureMsg.message.figureConfig[2].operate = 1;
  // visionFigureMsg.message.figureConfig[2].figureType = 2;
  // visionFigureMsg.message.figureConfig[2].layerID = 2;
  // visionFigureMsg.message.figureConfig[2].posit_X = 960 + 100;
  // visionFigureMsg.message.figureConfig[2].posit_Y = 540 - 100,
  // visionFigureMsg.message.figureConfig[2].color = 1;
  // visionFigureMsg.message.figureConfig[2].details_3 = 6;       // Radius
  // visionFigureMsg.message.figureConfig[2].width = 8;           // Line Width

  // visionFigureMsg.message.figureConfig[3].figureName[0] = 0;    // Frame ID
  // visionFigureMsg.message.figureConfig[3].figureName[1] = 2;    // Layer ID
  // visionFigureMsg.message.figureConfig[3].figureName[2] = 13;    // Figure ID
  // visionFigureMsg.message.figureConfig[3].operate = 1;
  // visionFigureMsg.message.figureConfig[3].figureType = 2;
  // visionFigureMsg.message.figureConfig[3].layerID = 2;
  // visionFigureMsg.message.figureConfig[3].posit_X = 960 - 100;
  // visionFigureMsg.message.figureConfig[3].posit_Y = 540 - 100,
  // visionFigureMsg.message.figureConfig[3].color = 2;
  // visionFigureMsg.message.figureConfig[3].details_3 = 6;       // Radius
  // visionFigureMsg.message.figureConfig[3].width = 8;           // Line Width

  // visionFigureMsg.message.figureConfig[4].figureName[0] = 0;    // Frame ID
  // visionFigureMsg.message.figureConfig[4].figureName[1] = 2;    // Layer ID
  // visionFigureMsg.message.figureConfig[4].figureName[2] = 14;    // Figure ID
  // visionFigureMsg.message.figureConfig[4].operate = 1;
  // visionFigureMsg.message.figureConfig[4].figureType = 2;
  // visionFigureMsg.message.figureConfig[4].layerID = 4;
  // visionFigureMsg.message.figureConfig[4].posit_X = 960 + 100;
  // visionFigureMsg.message.figureConfig[4].posit_Y = 540 - 100,
  // visionFigureMsg.message.figureConfig[4].color = 1;
  // visionFigureMsg.message.figureConfig[4].details_3 = 6;       // Radius
  // visionFigureMsg.message.figureConfig[4].width = 8;           // Line Width

}

void CSystemReferee::UI_StartStaticTextDrawing_() {
	hipTextMsg.message.figureConfig.operate = 1;
	hipTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&hipTextMsg), sizeof(hipTextMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&hipTextMsg), sizeof(hipTextMsg));

	proc_waitMs(50);

	modeTextMsg.message.figureConfig.operate = 1;
	modeTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&modeTextMsg), sizeof(modeTextMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&modeTextMsg), sizeof(modeTextMsg));

  proc_waitMs(50);

  hipInfoTextMsg.message.figureConfig.operate = 1;
  hipInfoTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&hipInfoTextMsg), sizeof(hipInfoTextMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&hipInfoTextMsg), sizeof(hipInfoTextMsg));

  proc_waitMs(50);

  pitchStaticTextMsg.message.figureConfig.operate = 1;
  pitchStaticTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&pitchStaticTextMsg), sizeof(pitchStaticTextMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&pitchStaticTextMsg), sizeof(pitchStaticTextMsg));

  proc_waitMs(50);

  pitchTextMsg.message.figureConfig.operate = 1;
  pitchTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&pitchTextMsg), sizeof(pitchTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&pitchTextMsg), sizeof(pitchTextMsg));

  proc_waitMs(50);

  parallelFigureMsg.message.figureConfig[0].operate = 1;
  parallelFigureMsg.message.figureConfig[1].operate = 1;
  parallelFigureMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&parallelFigureMsg), sizeof(parallelFigureMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&parallelFigureMsg), sizeof(parallelFigureMsg));
}

void CSystemReferee::UI_StartCurModeTextDrawing_() {
	curModeTextMsg.message.figureConfig.operate = 1;
	curModeTextMsg.message.figureConfig.details_2 = 4;        // String Length
	curModeTextMsg.message.figureConfig.posit_X = 960 - (25 * 2);
	curModeTextMsg.message.figureConfig.posit_Y = 780;
	strcpy(reinterpret_cast<char *>(curModeTextMsg.message.text), "NONE");
	curModeTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&curModeTextMsg), sizeof(curModeTextMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&curModeTextMsg), sizeof(curModeTextMsg));
}

void CSystemReferee::UI_StartRadarTextDrawing_() {
	RadarTextMsg.message.figureConfig.operate = 1;
	RadarTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&RadarTextMsg), sizeof(RadarTextMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&RadarTextMsg), sizeof(RadarTextMsg));
}

void CSystemReferee::UI_StartStateFigureDrawing_() {
	stateFigureMsg.message.figureConfig[0].operate = 1;
	stateFigureMsg.message.figureConfig[1].operate = 1;
	stateFigureMsg.message.figureConfig[2].operate = 1;
	stateFigureMsg.message.figureConfig[3].operate = 1;
	stateFigureMsg.message.figureConfig[4].operate = 1;
	stateFigureMsg.message.figureConfig[5].operate = 1;
	stateFigureMsg.message.figureConfig[6].operate = 1;
	stateFigureMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&stateFigureMsg), sizeof(stateFigureMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&stateFigureMsg), sizeof(stateFigureMsg));

	proc_waitMs(50);

	crawlerTextMsg.message.figureConfig.operate = 1;
	crawlerTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&crawlerTextMsg), sizeof(crawlerTextMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&crawlerTextMsg), sizeof(crawlerTextMsg));

	proc_waitMs(50);

	p3LockTextMsg.message.figureConfig.operate = 1;
	p3LockTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&p3LockTextMsg), sizeof(p3LockTextMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&p3LockTextMsg), sizeof(p3LockTextMsg));

	proc_waitMs(50);

	p3LockMsg.message.figureConfig[0].operate = 1;
	p3LockMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&p3LockMsg), sizeof(p3LockMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&p3LockMsg), sizeof(p3LockMsg));

	proc_waitMs(50);

	gripCloseTextMsg.message.figureConfig.operate = 1;
	gripCloseTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&gripCloseTextMsg), sizeof(gripCloseTextMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&gripCloseTextMsg), sizeof(gripCloseTextMsg));

	proc_waitMs(50);

	gripCloseMsg.message.figureConfig[0].operate = 1;
	gripCloseMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&gripCloseMsg), sizeof(gripCloseMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&gripCloseMsg), sizeof(gripCloseMsg));
}

void CSystemReferee::UI_StartVisionFigureDrawing_() {
	visionFigureMsg.message.figureConfig[0].operate = 1;
	visionFigureMsg.message.figureConfig[1].operate = 1;
	visionFigureMsg.message.figureConfig[2].operate = 1;
	visionFigureMsg.message.figureConfig[3].operate = 1;
	visionFigureMsg.message.figureConfig[4].operate = 1;
	visionFigureMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&visionFigureMsg), sizeof(visionFigureMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&visionFigureMsg), sizeof(visionFigureMsg));
}

void CSystemReferee::UI_StartHipTextDrawing_() {
  hipInfoTextMsg.message.figureConfig.operate = 1;
  hipInfoTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&hipInfoTextMsg), sizeof(hipInfoTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&hipInfoTextMsg), sizeof(hipInfoTextMsg));
}

void CSystemReferee::UI_StartPitchTextDrawing_() {
  pitchTextMsg.message.figureConfig.operate = 1;
  pitchTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&pitchTextMsg), sizeof(pitchTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&pitchTextMsg), sizeof(pitchTextMsg));
}

void CSystemReferee::UI_StartPositionFigureDrawing_() {
  positionFigureMsg.message.figureConfig[0].operate = 1;
  positionFigureMsg.message.figureConfig[1].operate = 1;
  positionFigureMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&positionFigureMsg), sizeof(positionFigureMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&positionFigureMsg), sizeof(positionFigureMsg));
}

void CSystemReferee::UI_StartParallelFigureDrawing_() {
  parallelFigureMsg.message.figureConfig[0].operate = 1;
  parallelFigureMsg.message.figureConfig[1].operate = 1;
  parallelFigureMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&parallelFigureMsg), sizeof(parallelFigureMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&parallelFigureMsg), sizeof(parallelFigureMsg));
}

void CSystemReferee::UI_StartArmAngleFigureDrawing_() {
  armAngleFigureMsg.message.figureConfig[0].operate = 1;
  armAngleFigureMsg.message.figureConfig[1].operate = 1;
  armAngleFigureMsg.message.figureConfig[2].operate = 1;
  armAngleFigureMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&armAngleFigureMsg), sizeof(armAngleFigureMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&armAngleFigureMsg), sizeof(armAngleFigureMsg));
}

void CSystemReferee::UI_StartArmYawFigureDrawing_() {
  armYawFigureMsg.message.figureConfig[0].operate = 1;
  armYawFigureMsg.message.figureConfig[1].operate = 1;
  armYawFigureMsg.message.figureConfig[2].operate = 1;
  armYawFigureMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&armYawFigureMsg), sizeof(armYawFigureMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&armYawFigureMsg), sizeof(armYawFigureMsg));
}

// 更新当前状态
void CSystemReferee::UI_UpdateCurModeTextDrawing_() {

	std::fill(&curModeTextMsg.message.text[0], &curModeTextMsg.message.text[29], 0);
  if (SystemCore.use_Controller_) {
    curModeTextMsg.message.figureConfig.details_2 = 6;
    curModeTextMsg.message.figureConfig.posit_X = 960 - (25 * 3);
    curModeTextMsg.message.figureConfig.posit_Y = 780;
    strcpy(reinterpret_cast<char *>(curModeTextMsg.message.text), "CUSTOM");
  }
  else {
    switch (SystemCore.currentAutoCtrlProcess_) {

      case CSystemCore::EAutoCtrlProcess::NONE: {
        curModeTextMsg.message.figureConfig.details_2 = 4;
        curModeTextMsg.message.figureConfig.posit_X = 960 - (25 * 2);
        curModeTextMsg.message.figureConfig.posit_Y = 780;
        strcpy(reinterpret_cast<char *>(curModeTextMsg.message.text), "NONE");
        break;
      }

      case CSystemCore::EAutoCtrlProcess::RETURN_ORIGIN: {
        curModeTextMsg.message.figureConfig.details_2 = 6;
        curModeTextMsg.message.figureConfig.posit_X = 960 - (25 * 3);
        curModeTextMsg.message.figureConfig.posit_Y = 780;
        strcpy(reinterpret_cast<char *>(curModeTextMsg.message.text), "ORIGIN");
        break;
      }

      case CSystemCore::EAutoCtrlProcess::CLIMBING: {
        curModeTextMsg.message.figureConfig.details_2 = 8;
        curModeTextMsg.message.figureConfig.posit_X = 960 - (25 * 3.5);
        curModeTextMsg.message.figureConfig.posit_Y = 780;
        strcpy(reinterpret_cast<char *>(curModeTextMsg.message.text), "CLIMBING");
        break;
      }

      case CSystemCore::EAutoCtrlProcess::GROUND_ORE: {
        curModeTextMsg.message.figureConfig.details_2 = 6;
        curModeTextMsg.message.figureConfig.posit_X = 960 - (25 * 3);
        curModeTextMsg.message.figureConfig.posit_Y = 780;
        strcpy(reinterpret_cast<char *>(curModeTextMsg.message.text), "GROUND");
        break;
      }

      case CSystemCore::EAutoCtrlProcess::STORE_ORE: {
        curModeTextMsg.message.figureConfig.details_2 = 5;
        curModeTextMsg.message.figureConfig.posit_X = 960 - (25 * 2.5);
        curModeTextMsg.message.figureConfig.posit_Y = 780;
        strcpy(reinterpret_cast<char *>(curModeTextMsg.message.text), "STORE");
        break;
      }

      case CSystemCore::EAutoCtrlProcess::EXCHANGE_ORE: {
        curModeTextMsg.message.figureConfig.details_2 = 8;
        curModeTextMsg.message.figureConfig.posit_X = 960 - (25 * 5.5);
        curModeTextMsg.message.figureConfig.posit_Y = 780;
        strcpy(reinterpret_cast<char *>(curModeTextMsg.message.text), "EXCHANGE");
        break;
      }

      case CSystemCore::EAutoCtrlProcess::ENERGY_UNIT: {
        curModeTextMsg.message.figureConfig.details_2 = 7;
        curModeTextMsg.message.figureConfig.posit_X = 960 - (25 * 3.5);
        curModeTextMsg.message.figureConfig.posit_Y = 780;
        strcpy(reinterpret_cast<char *>(curModeTextMsg.message.text), "GET ORE");
        break;
      }

      case CSystemCore::EAutoCtrlProcess::DOWN_STAIR: {
        curModeTextMsg.message.figureConfig.details_2 = 7;
        curModeTextMsg.message.figureConfig.posit_X = 960 - (25 * 3.5);
        curModeTextMsg.message.figureConfig.posit_Y = 780;
        strcpy(reinterpret_cast<char *>(curModeTextMsg.message.text), "DOWNSTAIR");
        break;
      }
    }
  }

	curModeTextMsg.message.figureConfig.operate = 2;
	curModeTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&curModeTextMsg), sizeof(curModeTextMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&curModeTextMsg), sizeof(curModeTextMsg));
}

void CSystemReferee::UI_UpdateStateFigureDrawing_() {

  static auto &chassis_info = reinterpret_cast<CModChassis *>(ModuleIDMap.at(EModuleID::MOD_CHASSIS))->chassisInfo;

	stateFigureMsg.message.figureConfig[0].operate = 2;
	stateFigureMsg.message.figureConfig[0].color = (chassis_info.crawler_on) ? 3 : 7;

	stateFigureMsg.message.figureConfig[0].operate = 2;
	stateFigureMsg.message.figureConfig[1].operate = 2;
	stateFigureMsg.message.figureConfig[2].operate = 2;
	stateFigureMsg.message.figureConfig[3].operate = 2;
	stateFigureMsg.message.figureConfig[4].operate = 2;
	stateFigureMsg.message.figureConfig[5].operate = 2;
	stateFigureMsg.message.figureConfig[6].operate = 2;

	stateFigureMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&stateFigureMsg), sizeof(stateFigureMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&stateFigureMsg), sizeof(stateFigureMsg));

  crawlerTextMsg.message.figureConfig.operate = 2;
  if(chassis_info.crawler_on) {
    crawlerTextMsg.message.figureConfig.color = 2;
  }
  else {
    crawlerTextMsg.message.figureConfig.color = 7;
  }
  crawlerTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&crawlerTextMsg), sizeof(crawlerTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&crawlerTextMsg), sizeof(crawlerTextMsg));

  proc_waitMs(50);

  p3LockTextMsg.message.figureConfig.operate = 2;
  if(SysControllerLink.robotInfo.p3_lock) {
    p3LockTextMsg.message.figureConfig.color = 2;
  }
  else {
    p3LockTextMsg.message.figureConfig.color = 7;
  }
  p3LockTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&p3LockTextMsg), sizeof(p3LockTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&p3LockTextMsg), sizeof(p3LockTextMsg));

  proc_waitMs(50);

  p3LockMsg.message.figureConfig[0].operate = 2;
  p3LockMsg.message.figureConfig[0].color = (SysControllerLink.robotInfo.p3_lock) ? 3 : 7;
  p3LockMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&p3LockMsg), sizeof(p3LockMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&p3LockMsg), sizeof(p3LockMsg));

  proc_waitMs(50);

  gripCloseTextMsg.message.figureConfig.operate = 2;
  if(true) {    // 这里留出来，等后面有了夹爪标志位加上
    gripCloseTextMsg.message.figureConfig.color = 2;
  }
  else {
    gripCloseTextMsg.message.figureConfig.color = 7;
  }
  gripCloseTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&gripCloseTextMsg), sizeof(gripCloseTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&gripCloseTextMsg), sizeof(gripCloseTextMsg));

  proc_waitMs(50);

  gripCloseMsg.message.figureConfig[0].operate = 2;
  gripCloseMsg.message.figureConfig[0].color = (true) ? 3 : 7;
  gripCloseMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&gripCloseMsg), sizeof(gripCloseMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&gripCloseMsg), sizeof(gripCloseMsg));
}

void CSystemReferee::UI_UpdateVisionFigureDrawing_() {

	visionFigureMsg.message.figureConfig[0].operate = 2;
	visionFigureMsg.message.figureConfig[1].operate = 2;
	visionFigureMsg.message.figureConfig[2].operate = 2;
	visionFigureMsg.message.figureConfig[3].operate = 2;
	visionFigureMsg.message.figureConfig[4].operate = 2;

	if (SysVision.systemStatus != APP_OK) {
		visionFigureMsg.message.figureConfig[0].color = 7;
	} else {
		visionFigureMsg.message.figureConfig[0].color = (SysVision.visionInfo.oreTank.isFoundOreTank) ? 2 : 1;
	}

	if (SysVision.systemStatus == APP_OK && SysVision.visionInfo.uiPoint.isFoundOreTank) {
		visionFigureMsg.message.figureConfig[1].posit_X = 960 + SysVision.visionInfo.uiPoint.pointPosit_X[0] * 1.1;
		visionFigureMsg.message.figureConfig[1].posit_Y = 510 + SysVision.visionInfo.uiPoint.pointPosit_Y[0] * 1.1;
		visionFigureMsg.message.figureConfig[2].posit_X = 960 + SysVision.visionInfo.uiPoint.pointPosit_X[1] * 1.1;
		visionFigureMsg.message.figureConfig[2].posit_Y = 510 + SysVision.visionInfo.uiPoint.pointPosit_Y[1] * 1.1;
		visionFigureMsg.message.figureConfig[3].posit_X = 960 + SysVision.visionInfo.uiPoint.pointPosit_X[2] * 1.1;
		visionFigureMsg.message.figureConfig[3].posit_Y = 510 + SysVision.visionInfo.uiPoint.pointPosit_Y[2] * 1.1;
		visionFigureMsg.message.figureConfig[4].posit_X = 960 + SysVision.visionInfo.uiPoint.pointPosit_X[3] * 1.1;
		visionFigureMsg.message.figureConfig[4].posit_Y = 510 + SysVision.visionInfo.uiPoint.pointPosit_Y[3] * 1.1;
	} else {
		visionFigureMsg.message.figureConfig[1].posit_X = 960 + 400;
		visionFigureMsg.message.figureConfig[1].posit_Y = 540 + 300;
		visionFigureMsg.message.figureConfig[2].posit_X = 960 + 400;
		visionFigureMsg.message.figureConfig[2].posit_Y = 540 - 300;
		visionFigureMsg.message.figureConfig[3].posit_X = 960 - 400;
		visionFigureMsg.message.figureConfig[3].posit_Y = 540 - 300;
		visionFigureMsg.message.figureConfig[4].posit_X = 960 - 400;
		visionFigureMsg.message.figureConfig[4].posit_Y = 540 + 300;
	}

	visionFigureMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&visionFigureMsg), sizeof(visionFigureMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&visionFigureMsg), sizeof(visionFigureMsg));
}

void CSystemReferee::UI_UpdateHipTextDrawing_() {
  static auto &chassis_info = reinterpret_cast<CModChassis *>(ModuleIDMap.at(EModuleID::MOD_CHASSIS))->chassisInfo;
  std::fill(&hipInfoTextMsg.message.text[0], &hipInfoTextMsg.message.text[29], 0);
  hipInfoTextMsg.message.figureConfig.operate = 2;
  int32_t int_val = (int32_t)(chassis_info.L_Length * 1000.f);
  hipInfoTextMsg.message.figureConfig.details_3 = int_val & 0x3FF;
  hipInfoTextMsg.message.figureConfig.details_4 = (int_val >> 10) & 0x7FF;
  hipInfoTextMsg.message.figureConfig.details_5 = (int_val >> 21) & 0x7FF;
  // hipInfoTextMsg.message.figureConfig.details_3 = chassis_info.L_Length * 1000.f;
  // hipInfoTextMsg.message.figureConfig.details_4 = chassis.L_Length * 1000;
  // hipInfoTextMsg.message.figureConfig.details_5 = chassis.L_Length * 1000;
  hipInfoTextMsg.message.figureConfig.posit_X = 1620;
  hipInfoTextMsg.message.figureConfig.posit_Y = 690;
  // sprintf(reinterpret_cast<char *>(hipInfoTextMsg.message.text), "%.2f", chassis.L_Length * 1000);
  hipInfoTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&hipInfoTextMsg), sizeof(hipInfoTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&hipInfoTextMsg), sizeof(hipInfoTextMsg));
}

void CSystemReferee::UI_UpdatePitchTextDrawing_() {
  static auto &chassis_info = reinterpret_cast<CModChassis *>(ModuleIDMap.at(EModuleID::MOD_CHASSIS))->chassisInfo;
  std::fill(&pitchTextMsg.message.text[0], &pitchTextMsg.message.text[29], 0);
  pitchTextMsg.message.figureConfig.operate = 2;
  int32_t int_val = (int32_t)(chassis_info.roll_Measure[0] * 1000.f);
  pitchTextMsg.message.figureConfig.details_3 = int_val & 0x3FF;
  pitchTextMsg.message.figureConfig.details_4 = (int_val >> 10) & 0x7FF;
  pitchTextMsg.message.figureConfig.details_5 = (int_val >> 21) & 0x7FF;  ///< 分为高中低11 11 10位发送
  // pitchTextMsg.message.figureConfig.details_3 = chassis_info.roll_Measure[0] * 1000.f;
  pitchTextMsg.message.figureConfig.posit_X = 1520;
  pitchTextMsg.message.figureConfig.posit_Y = 640;
  // sprintf(reinterpret_cast<char *>(pitchTextMsg.message.text), "%.2f", chassis.roll_Measure[0] * 1000);
  pitchTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&pitchTextMsg), sizeof(pitchTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&pitchTextMsg), sizeof(pitchTextMsg));
}

void CSystemReferee::UI_UpdatePositionFigureDrawing_() {
  static auto &chassis_info = reinterpret_cast<CModChassis *>(ModuleIDMap.at(EModuleID::MOD_CHASSIS))->chassisInfo;
  positionFigureMsg.message.figureConfig[0].operate = 2;
  positionFigureMsg.message.figureConfig[1].operate = 2;
  positionFigureMsg.message.figureConfig[1].posit_X = 960 - (uint32_t)(80 * cos(fabs(chassis_info.roll_Measure[0]) * 2 * PI / 180.f));
  positionFigureMsg.message.figureConfig[1].posit_Y = 200 + (uint32_t)(80 * sin(fabs(chassis_info.roll_Measure[0]) * 2 * PI / 180.f));
  positionFigureMsg.message.figureConfig[1].details_4 = 960 + (uint32_t)(80 * cos(fabs(chassis_info.roll_Measure[0]) * 2 * PI / 180.f));
  positionFigureMsg.message.figureConfig[1].details_5 = 200 - (uint32_t)(80 * sin(fabs(chassis_info.roll_Measure[0]) * 2 * PI / 180.f));
  positionFigureMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&positionFigureMsg), sizeof(positionFigureMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&positionFigureMsg), sizeof(positionFigureMsg));
}

void CSystemReferee::UI_UpdateArmYawFigureDrawing_() {
  if (ModuleIDMap.find(EModuleID::MOD_ARM) == ModuleIDMap.end()) return;
  static auto &arm_info = reinterpret_cast<CModArm *>(ModuleIDMap.at(EModuleID::MOD_ARM))->armInfo;
  static auto &gimbal_info = reinterpret_cast<CModGimbal *>(ModuleIDMap.at(EModuleID::MOD_GIMBAL))->gimbalInfo;

  float rad = (arm_info.angle_Yaw + gimbal_info.angle_visualyaw) * 2 * PI / 360.0f;
  
  armYawFigureMsg.message.figureConfig[0].operate = 2; // circle static
    
  //底盘示意直线 随臂的yaw变而变
  armYawFigureMsg.message.figureConfig[1].operate = 2;
  armYawFigureMsg.message.figureConfig[1].details_4 = (uint32_t)(200.0f - 80.0f * sin(rad));
  armYawFigureMsg.message.figureConfig[1].details_5 = (uint32_t)(700.0f + 80.0f * cos(rad));
    
  //臂示意直线 保持 90° 不动
  armYawFigureMsg.message.figureConfig[2].operate = 2;
  armYawFigureMsg.message.figureConfig[2].details_4 = 200;
  armYawFigureMsg.message.figureConfig[2].details_5 = 700 + 80;
  armYawFigureMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&armYawFigureMsg), sizeof(armYawFigureMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&armYawFigureMsg), sizeof(armYawFigureMsg));
}

void CSystemReferee::UI_UpdateArmAngleFigureDrawing_() {
  if (ModuleIDMap.find(EModuleID::MOD_ARM) == ModuleIDMap.end()) return;
  static auto &arm_info = reinterpret_cast<CModArm *>(ModuleIDMap.at(EModuleID::MOD_ARM))->armInfo;

  float origin_x = 1500.0f;
  float origin_y = 420.0f; // 300.f
  float len1 = 120.0f;
  float len2 = 80.0f;
  float len3 = 50.0f;

  float rad1 = (arm_info.angle_Pitch1 - 0.0f) * 2 * PI / 360.0f;
  float rad2 = (arm_info.angle_Pitch2 - 70.f) * 2 * PI / 360.0f;   
  float rad3 = (arm_info.angle_Pitch3 - 0.0f) * 2 * PI / 360.0f;  

  float p1_x = origin_x + len1 * cos(rad1);
  float p1_y = origin_y + len1 * sin(rad1);
  float p2_x = p1_x + len2 * cos(rad2);
  float p2_y = p1_y + len2 * sin(rad2);
  float p3_x = p2_x + len3 * cos(rad3);
  float p3_y = p2_y + len3 * sin(rad3);

  armAngleFigureMsg.message.figureConfig[0].operate = 2;
  armAngleFigureMsg.message.figureConfig[0].posit_X = (uint32_t)origin_x;
  armAngleFigureMsg.message.figureConfig[0].posit_Y = (uint32_t)origin_y;
  armAngleFigureMsg.message.figureConfig[0].details_4 = (uint32_t)p1_x;
  armAngleFigureMsg.message.figureConfig[0].details_5 = (uint32_t)p1_y;

  armAngleFigureMsg.message.figureConfig[1].operate = 2;
  armAngleFigureMsg.message.figureConfig[1].posit_X = (uint32_t)p1_x;
  armAngleFigureMsg.message.figureConfig[1].posit_Y = (uint32_t)p1_y;
  armAngleFigureMsg.message.figureConfig[1].details_4 = (uint32_t)p2_x;
  armAngleFigureMsg.message.figureConfig[1].details_5 = (uint32_t)p2_y;

  armAngleFigureMsg.message.figureConfig[2].operate = 2;
  armAngleFigureMsg.message.figureConfig[2].posit_X = (uint32_t)p2_x;
  armAngleFigureMsg.message.figureConfig[2].posit_Y = (uint32_t)p2_y;
  armAngleFigureMsg.message.figureConfig[2].details_4 = (uint32_t)p3_x;
  armAngleFigureMsg.message.figureConfig[2].details_5 = (uint32_t)p3_y;

  armAngleFigureMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&armAngleFigureMsg), sizeof(armAngleFigureMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&armAngleFigureMsg), sizeof(armAngleFigureMsg));
}

void CSystemReferee::UI_RADAR_WARNING_TextDrawing_() {
  std::fill(&RadarTextMsg.message.text[0], &RadarTextMsg.message.text[29], 0);
  RadarTextMsg.message.figureConfig.operate = 2;
  RadarTextMsg.message.figureConfig.details_2 = 7;
  RadarTextMsg.message.figureConfig.posit_X = 960 - (25 * 3.5);
  RadarTextMsg.message.figureConfig.posit_Y = 540 - 25;
  strcpy(reinterpret_cast<char *>(RadarTextMsg.message.text), "WARNING");
  RadarTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&RadarTextMsg), sizeof(RadarTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&RadarTextMsg), sizeof(RadarTextMsg));
}

void CSystemReferee::UI_RADAR_WARNING_TextClearing_() {
  RadarTextMsg.message.figureConfig.operate = 2;
  RadarTextMsg.message.figureConfig.details_2 = 1;
  RadarTextMsg.message.figureConfig.posit_X = 960 - (25 * 0.5);
  RadarTextMsg.message.figureConfig.posit_Y = 540 - 25;
  std::fill(&RadarTextMsg.message.text[0], &RadarTextMsg.message.text[29], 0);
  // strcpy(reinterpret_cast<char *>(RadarTextMsg.message.text), ".");
  RadarTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&RadarTextMsg), sizeof(RadarTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&RadarTextMsg), sizeof(RadarTextMsg));
}

void CSystemReferee::StartSysRefereeUiTask(void *arg) {
	proc_waitUntil(SysReferee.refereeInfo.robot.robotCamp != 0 );

	// 初始化UI
	SysReferee.UI_InitDrawing();

  // 静态UI
	SysReferee.UI_StartStaticTextDrawing_();
	proc_waitMs(50);
	SysReferee.UI_StartCurModeTextDrawing_();
	proc_waitMs(50);
	SysReferee.UI_StartStateFigureDrawing_();
	proc_waitMs(50);
	// SysReferee.UI_StartVisionFigureDrawing_();
	// proc_waitMs(100);

  SysReferee.UI_StartRadarTextDrawing_();
  proc_waitMs(50);

  SysReferee.UI_StartPositionFigureDrawing_();  
  proc_waitMs(50);

  SysReferee.UI_StartParallelFigureDrawing_();
  proc_waitMs(50);

  SysReferee.UI_StartArmAngleFigureDrawing_();
  proc_waitMs(50);

  SysReferee.UI_StartArmYawFigureDrawing_();
  proc_waitMs(50);

	// 进入UI绘制循环
	while (true) {

    // 在这里面更新动态UI
		SysReferee.UI_UpdateCurModeTextDrawing_();
		proc_waitMs(50);

		// SysReferee.UI_UpdateVisionFigureDrawing_();
		// proc_waitMs(100);

		SysReferee.UI_UpdateStateFigureDrawing_();
		proc_waitMs(50);

    SysReferee.UI_UpdateHipTextDrawing_();
    proc_waitMs(50);

    SysReferee.UI_UpdatePitchTextDrawing_();
    proc_waitMs(50);

    SysReferee.UI_UpdatePositionFigureDrawing_();
    proc_waitMs(50);

    SysReferee.UI_UpdateArmAngleFigureDrawing_();
    proc_waitMs(50);

    SysReferee.UI_UpdateArmYawFigureDrawing_();
    proc_waitMs(50);

		// SysReferee.UI_UpdateVisionFigureDrawing_();

		// proc_waitMs(100);

    if (SysReferee.refereeInfo.radar.if_dart_comming == 1) {
      SysReferee.UI_RADAR_WARNING_TextDrawing_();
      proc_waitMs(50);
    } else {
      SysReferee.UI_RADAR_WARNING_TextClearing_();
      proc_waitMs(50);
    }
	}

	proc_return();
}

} // namespace my_engineer
