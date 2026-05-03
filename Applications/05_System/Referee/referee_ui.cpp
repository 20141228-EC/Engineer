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
  // hipTextMsg.header = CDevReferee::SPkgHeader();
  // hipTextMsg.header.len = sizeof(hipTextMsg) - 9;
  // hipTextMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  // hipTextMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&hipTextMsg.header), 4);
  // hipTextMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  // hipTextMsg.transmitterID += (refereeInfo.robot.robotID);
  // hipTextMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  // hipTextMsg.receiverID += (refereeInfo.robot.robotID);
  // hipTextMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_TEXT;
  // hipTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  // hipTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  // hipTextMsg.message.figureConfig.figureName[2] = 0;    // Figure ID
  // hipTextMsg.message.figureConfig.operate = 1;
  // hipTextMsg.message.figureConfig.figureType = 7;
  // hipTextMsg.message.figureConfig.layerID = 0;
  // hipTextMsg.message.figureConfig.details_1 = 20;       // Font Size
  // hipTextMsg.message.figureConfig.posit_X = 1400; //1400
  // hipTextMsg.message.figureConfig.posit_Y = 790,
  // hipTextMsg.message.figureConfig.color = 4;
  // hipTextMsg.message.figureConfig.details_2 = 11;        // String Length
  // hipTextMsg.message.figureConfig.width = 2;            // Line Width
  // strcpy(reinterpret_cast<char *>(hipTextMsg.message.text), "Hip_Length:");

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

  /* Text - Spin Config */
  spinTextMsg.header = CDevReferee::SPkgHeader();
  spinTextMsg.header.len = sizeof(spinTextMsg) - 9;
  spinTextMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  spinTextMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&spinTextMsg.header), 4);
  spinTextMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  spinTextMsg.transmitterID += (refereeInfo.robot.robotID);
  spinTextMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  spinTextMsg.receiverID += (refereeInfo.robot.robotID);
  spinTextMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_TEXT;
  spinTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  spinTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  spinTextMsg.message.figureConfig.figureName[2] = 7;    // Figure ID
  spinTextMsg.message.figureConfig.operate = 1;
  spinTextMsg.message.figureConfig.figureType = 7;
  spinTextMsg.message.figureConfig.layerID = 0;
  spinTextMsg.message.figureConfig.details_1 = 20;       // Font Size
  spinTextMsg.message.figureConfig.posit_X = 1400;
  spinTextMsg.message.figureConfig.posit_Y = 840;
  spinTextMsg.message.figureConfig.color = 4;
  spinTextMsg.message.figureConfig.details_2 = 8;        // String Length
  spinTextMsg.message.figureConfig.width = 2;            // Line Width
  strcpy(reinterpret_cast<char *>(spinTextMsg.message.text), "SPIN:OFF");

  /* Text - Grip Config */
  gripTextMsg.header = CDevReferee::SPkgHeader();
  gripTextMsg.header.len = sizeof(gripTextMsg) - 9;
  gripTextMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  gripTextMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&gripTextMsg.header), 4);
  gripTextMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  gripTextMsg.transmitterID += (refereeInfo.robot.robotID);
  gripTextMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  gripTextMsg.receiverID += (refereeInfo.robot.robotID);
  gripTextMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_TEXT;
  gripTextMsg.message.figureConfig.figureName[0] = 0;
  gripTextMsg.message.figureConfig.figureName[1] = 0;
  gripTextMsg.message.figureConfig.figureName[2] = 10;
  gripTextMsg.message.figureConfig.operate = 1;
  gripTextMsg.message.figureConfig.figureType = 7;
  gripTextMsg.message.figureConfig.layerID = 0;
  gripTextMsg.message.figureConfig.details_1 = 20;
  gripTextMsg.message.figureConfig.posit_X = 1400;
  gripTextMsg.message.figureConfig.posit_Y = 880;
  gripTextMsg.message.figureConfig.color = 4;
  gripTextMsg.message.figureConfig.details_2 = 8;
  gripTextMsg.message.figureConfig.width = 2;
  strcpy(reinterpret_cast<char *>(gripTextMsg.message.text), "GRIP:OFF");

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
  hipInfoTextMsg.message.figureConfig.posit_Y = 790,
  hipInfoTextMsg.message.figureConfig.color = 4;
  // hipInfoTextMsg.message.figureConfig.details_3 = chassis.L_Length * 1000.f;
  hipInfoTextMsg.message.figureConfig.width = 2;            // Line Width
  // sprintf(reinterpret_cast<char *>(hipInfoTextMsg.message.text), "%.2f", chassis.L_Length * 1000);

  /* Text - Yaw Config */
  yawTextMsg.header = CDevReferee::SPkgHeader();
  yawTextMsg.header.len = sizeof(yawTextMsg) - 9;
  yawTextMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  yawTextMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&yawTextMsg.header), 4);
  yawTextMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  yawTextMsg.transmitterID += (refereeInfo.robot.robotID);
  yawTextMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  yawTextMsg.receiverID += (refereeInfo.robot.robotID);
  yawTextMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_TEXT;
  yawTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  yawTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  yawTextMsg.message.figureConfig.figureName[2] = 5;    // Figure ID
  yawTextMsg.message.figureConfig.operate = 1;
  yawTextMsg.message.figureConfig.figureType = 5;
  yawTextMsg.message.figureConfig.layerID = 0;
  yawTextMsg.message.figureConfig.details_1 = 20;       // Font Size
  // yawTextMsg.message.figureConfig.details_3 = chassis.roll_Measure[0] * 1000.f;
  yawTextMsg.message.figureConfig.posit_X = 1520;
  yawTextMsg.message.figureConfig.posit_Y = 790,
  yawTextMsg.message.figureConfig.color = 4;
  yawTextMsg.message.figureConfig.width = 2;            // Line Width
  // sprintf(reinterpret_cast<char *>(yawTextMsg.message.text), "%.2f", chassis.roll_Measure[0] * 1000);

  /* Text - Speed Config */
  speedTextMsg.header = CDevReferee::SPkgHeader();
  speedTextMsg.header.len = sizeof(speedTextMsg) - 9;
  speedTextMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  speedTextMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&speedTextMsg.header), 4);
  speedTextMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  speedTextMsg.transmitterID += (refereeInfo.robot.robotID);
  speedTextMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  speedTextMsg.receiverID += (refereeInfo.robot.robotID);
  speedTextMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_TEXT;
  speedTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  speedTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  speedTextMsg.message.figureConfig.figureName[2] = 6;    // Figure ID
  speedTextMsg.message.figureConfig.operate = 1;
  speedTextMsg.message.figureConfig.figureType = 5;
  speedTextMsg.message.figureConfig.layerID = 0;
  speedTextMsg.message.figureConfig.details_1 = 20;       // Font Size
  speedTextMsg.message.figureConfig.posit_X = 1520;
  speedTextMsg.message.figureConfig.posit_Y = 830,
  speedTextMsg.message.figureConfig.color = 4;
  speedTextMsg.message.figureConfig.width = 2;            // Line Width

  /* Text - Mode Config */
  yawStaticTextMsg.header = CDevReferee::SPkgHeader();
  yawStaticTextMsg.header.len = sizeof(yawStaticTextMsg) - 9;
  yawStaticTextMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  yawStaticTextMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&yawStaticTextMsg.header), 4);
  yawStaticTextMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  yawStaticTextMsg.transmitterID += (refereeInfo.robot.robotID);
  yawStaticTextMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  yawStaticTextMsg.receiverID += (refereeInfo.robot.robotID);
  yawStaticTextMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_TEXT;
  yawStaticTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  yawStaticTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  yawStaticTextMsg.message.figureConfig.figureName[2] = 8;    // Figure ID
  yawStaticTextMsg.message.figureConfig.operate = 1;
  yawStaticTextMsg.message.figureConfig.figureType = 7;
  yawStaticTextMsg.message.figureConfig.layerID = 0;
  yawStaticTextMsg.message.figureConfig.details_1 = 20;       // Font Size
  yawStaticTextMsg.message.figureConfig.posit_X = 1400;
  yawStaticTextMsg.message.figureConfig.posit_Y = 790,
  yawStaticTextMsg.message.figureConfig.color = 4;
  yawStaticTextMsg.message.figureConfig.details_2 = 4;        // String Length
  yawStaticTextMsg.message.figureConfig.width = 2;            // Line Width
  strcpy(reinterpret_cast<char *>(yawStaticTextMsg.message.text), "YAW:");

  speedStaticTextMsg.header = CDevReferee::SPkgHeader();
  speedStaticTextMsg.header.len = sizeof(speedStaticTextMsg) - 9;
  speedStaticTextMsg.header.cmdId = CDevReferee::ECommandID::ID_ROBOT_MSG;
  speedStaticTextMsg.header.CRC8 = CCrcValidator::Crc8Calculate(reinterpret_cast<uint8_t *>(&speedStaticTextMsg.header), 4);
  speedStaticTextMsg.transmitterID = (refereeInfo.robot.robotCamp == 2) ? 100 : 0;
  speedStaticTextMsg.transmitterID += (refereeInfo.robot.robotID);
  speedStaticTextMsg.receiverID = (refereeInfo.robot.robotCamp == 2) ? 0x164 : 0x100;
  speedStaticTextMsg.receiverID += (refereeInfo.robot.robotID);
  speedStaticTextMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_TEXT;
  speedStaticTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  speedStaticTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  speedStaticTextMsg.message.figureConfig.figureName[2] = 9;    // Figure ID
  speedStaticTextMsg.message.figureConfig.operate = 1;
  speedStaticTextMsg.message.figureConfig.figureType = 7;
  speedStaticTextMsg.message.figureConfig.layerID = 0;
  speedStaticTextMsg.message.figureConfig.details_1 = 20;       // Font Size
  speedStaticTextMsg.message.figureConfig.posit_X = 1400;
  speedStaticTextMsg.message.figureConfig.posit_Y = 750,
  speedStaticTextMsg.message.figureConfig.color = 4;
  speedStaticTextMsg.message.figureConfig.details_2 = 5;        // String Length
  speedStaticTextMsg.message.figureConfig.width = 2;            // Line Width
  strcpy(reinterpret_cast<char *>(speedStaticTextMsg.message.text), "SPEED:");

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
  positionFigureMsg.messageID = CDevReferee::EMessageID::ID_UI_DRAW_HEPTA;

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
  positionFigureMsg.message.figureConfig[1].posit_X = 960;
  positionFigureMsg.message.figureConfig[1].posit_Y = 120;
  positionFigureMsg.message.figureConfig[1].color = 8;
  positionFigureMsg.message.figureConfig[1].details_4 = 960;       // End Posit X
  positionFigureMsg.message.figureConfig[1].details_5 = 280;       // End Posit Y
  positionFigureMsg.message.figureConfig[1].width = 2;            // Line Width

  // 四个轮子的方向示意线（LF, RF, LB, RB）
  for (uint8_t i = 0; i < 4; ++i) {
    const uint8_t idx = static_cast<uint8_t>(2 + i);
    positionFigureMsg.message.figureConfig[idx].figureName[0] = 0;
    positionFigureMsg.message.figureConfig[idx].figureName[1] = 2;
    positionFigureMsg.message.figureConfig[idx].figureName[2] = idx;
    positionFigureMsg.message.figureConfig[idx].operate = 1;
    positionFigureMsg.message.figureConfig[idx].figureType = 0;
    positionFigureMsg.message.figureConfig[idx].layerID = 2;
    positionFigureMsg.message.figureConfig[idx].color = 2;
    positionFigureMsg.message.figureConfig[idx].width = 2;
  }

  // 预留一个图元占位，保持Hepta结构完整
  positionFigureMsg.message.figureConfig[6].figureName[0] = 0;
  positionFigureMsg.message.figureConfig[6].figureName[1] = 2;
  positionFigureMsg.message.figureConfig[6].figureName[2] = 6;
  positionFigureMsg.message.figureConfig[6].operate = 1;
  positionFigureMsg.message.figureConfig[6].figureType = 2;
  positionFigureMsg.message.figureConfig[6].layerID = 2;
  positionFigureMsg.message.figureConfig[6].color = 7;
  positionFigureMsg.message.figureConfig[6].width = 1;
  positionFigureMsg.message.figureConfig[6].posit_X = 960;
  positionFigureMsg.message.figureConfig[6].posit_Y = 200;
  positionFigureMsg.message.figureConfig[6].details_3 = 1;

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
	// hipTextMsg.message.figureConfig.operate = 1;
	// hipTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&hipTextMsg), sizeof(hipTextMsg) - 2);
	// pInterface_->Transmit(reinterpret_cast<uint8_t *>(&hipTextMsg), sizeof(hipTextMsg));

	// proc_waitMs(200);

	modeTextMsg.message.figureConfig.operate = 1;
	modeTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&modeTextMsg), sizeof(modeTextMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&modeTextMsg), sizeof(modeTextMsg));

  proc_waitMs(200);

  spinTextMsg.message.figureConfig.operate = 1;
  spinTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&spinTextMsg), sizeof(spinTextMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&spinTextMsg), sizeof(spinTextMsg));

  proc_waitMs(200);

  gripTextMsg.message.figureConfig.operate = 1;
  gripTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&gripTextMsg), sizeof(gripTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&gripTextMsg), sizeof(gripTextMsg));

  proc_waitMs(200);

  // hipInfoTextMsg.message.figureConfig.operate = 1;
  // hipInfoTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&hipInfoTextMsg), sizeof(hipInfoTextMsg) - 2);
	// pInterface_->Transmit(reinterpret_cast<uint8_t *>(&hipInfoTextMsg), sizeof(hipInfoTextMsg));

  // proc_waitMs(200);

  yawStaticTextMsg.message.figureConfig.operate = 1;
  yawStaticTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&yawStaticTextMsg), sizeof(yawStaticTextMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&yawStaticTextMsg), sizeof(yawStaticTextMsg));

  proc_waitMs(200);

  yawTextMsg.message.figureConfig.operate = 1;
  yawTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&yawTextMsg), sizeof(yawTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&yawTextMsg), sizeof(yawTextMsg));

  proc_waitMs(200);

  speedStaticTextMsg.message.figureConfig.operate = 1;
  speedStaticTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&speedStaticTextMsg), sizeof(speedStaticTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&speedStaticTextMsg), sizeof(speedStaticTextMsg));

  proc_waitMs(200);

  speedTextMsg.message.figureConfig.operate = 1;
  speedTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&speedTextMsg), sizeof(speedTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&speedTextMsg), sizeof(speedTextMsg));

  proc_waitMs(200);
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
void CSystemReferee::UI_StartSpeedTextDrawing_() {
  speedTextMsg.message.figureConfig.operate = 1;
  speedTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&speedTextMsg), sizeof(speedTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&speedTextMsg), sizeof(speedTextMsg));
}

void CSystemReferee::UI_StartSpinTextDrawing_() {
  spinTextMsg.message.figureConfig.operate = 1;
  spinTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&spinTextMsg), sizeof(spinTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&spinTextMsg), sizeof(spinTextMsg));
}

void CSystemReferee::UI_StartGripTextDrawing_() {
  gripTextMsg.message.figureConfig.operate = 1;
  gripTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&gripTextMsg), sizeof(gripTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&gripTextMsg), sizeof(gripTextMsg));
}

void CSystemReferee::UI_StartYawTextDrawing_() {
  yawTextMsg.message.figureConfig.operate = 1;
  yawTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&yawTextMsg), sizeof(yawTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&yawTextMsg), sizeof(yawTextMsg));
}

void CSystemReferee::UI_StartPositionFigureDrawing_() {
  positionFigureMsg.message.figureConfig[0].operate = 1;
  positionFigureMsg.message.figureConfig[1].operate = 1;
  positionFigureMsg.message.figureConfig[2].operate = 1;
  positionFigureMsg.message.figureConfig[3].operate = 1;
  positionFigureMsg.message.figureConfig[4].operate = 1;
  positionFigureMsg.message.figureConfig[5].operate = 1;
  positionFigureMsg.message.figureConfig[6].operate = 1;
  positionFigureMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&positionFigureMsg), sizeof(positionFigureMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&positionFigureMsg), sizeof(positionFigureMsg));
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

      // case CSystemCore::EAutoCtrlProcess::STORE_ORE: {
      //   curModeTextMsg.message.figureConfig.details_2 = 5;
      //   curModeTextMsg.message.figureConfig.posit_X = 960 - (25 * 2.5);
      //   curModeTextMsg.message.figureConfig.posit_Y = 780;
      //   strcpy(reinterpret_cast<char *>(curModeTextMsg.message.text), "STORE");
      //   break;
      // }

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

      // case CSystemCore::EAutoCtrlProcess::DOWN_STAIR: {
      //   curModeTextMsg.message.figureConfig.details_2 = 7;
      //   curModeTextMsg.message.figureConfig.posit_X = 960 - (25 * 3.5);
      //   curModeTextMsg.message.figureConfig.posit_Y = 780;
      //   strcpy(reinterpret_cast<char *>(curModeTextMsg.message.text), "DOWNSTAIR");
      //   break;
      // }
    }
  }

	curModeTextMsg.message.figureConfig.operate = 2;
	curModeTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&curModeTextMsg), sizeof(curModeTextMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&curModeTextMsg), sizeof(curModeTextMsg));
}

void CSystemReferee::UI_UpdateStateFigureDrawing_() {

	static auto &chassis = reinterpret_cast<CModChassis *>(ModuleIDMap.at(EModuleID::MOD_CHASSIS))->chassisInfo;
  stateFigureMsg.message.figureConfig[0].operate = 2;
  const bool boardSpinValid = (SysBoardLink.otherInfo.pack_id == CDevBoardLink::PKT_OTHER_INFOS);
  const bool spinOn = boardSpinValid ? (SysBoardLink.otherInfo.is_spin_on != 0)
                                     : (reinterpret_cast<CModChassis *>(ModuleIDMap.at(EModuleID::MOD_CHASSIS))->spin_on);
  stateFigureMsg.message.figureConfig[0].color = spinOn ? 3 : 7;

	stateFigureMsg.message.figureConfig[0].operate = 2;
	stateFigureMsg.message.figureConfig[1].operate = 2;
	stateFigureMsg.message.figureConfig[2].operate = 2;
	stateFigureMsg.message.figureConfig[3].operate = 2;
	stateFigureMsg.message.figureConfig[4].operate = 2;
	stateFigureMsg.message.figureConfig[5].operate = 2;
	stateFigureMsg.message.figureConfig[6].operate = 2;

	stateFigureMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&stateFigureMsg), sizeof(stateFigureMsg) - 2);
	pInterface_->Transmit(reinterpret_cast<uint8_t *>(&stateFigureMsg), sizeof(stateFigureMsg));
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
  // static auto &chassis_info = reinterpret_cast<CModChassis *>(ModuleIDMap.at(EModuleID::MOD_CHASSIS))->chassisInfo;
  // std::fill(&hipInfoTextMsg.message.text[0], &hipInfoTextMsg.message.text[29], 0);
  // hipInfoTextMsg.message.figureConfig.operate = 2;
  // int32_t int_val = (int32_t)(chassis_info.L_Length * 1000.f);
  // hipInfoTextMsg.message.figureConfig.details_3 = int_val & 0x3FF;
  // hipInfoTextMsg.message.figureConfig.details_4 = (int_val >> 10) & 0x7FF;
  // hipInfoTextMsg.message.figureConfig.details_5 = (int_val >> 21) & 0x7FF;
  // // hipInfoTextMsg.message.figureConfig.details_3 = chassis_info.L_Length * 1000.f;
  // // hipInfoTextMsg.message.figureConfig.details_4 = chassis.L_Length * 1000;
  // // hipInfoTextMsg.message.figureConfig.details_5 = chassis.L_Length * 1000;
  // hipInfoTextMsg.message.figureConfig.posit_X = 1620;
  // hipInfoTextMsg.message.figureConfig.posit_Y = 790;
  // // sprintf(reinterpret_cast<char *>(hipInfoTextMsg.message.text), "%.2f", chassis.L_Length * 1000);
  // hipInfoTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&hipInfoTextMsg), sizeof(hipInfoTextMsg) - 2);
  // pInterface_->Transmit(reinterpret_cast<uint8_t *>(&hipInfoTextMsg), sizeof(hipInfoTextMsg));
}

void CSystemReferee::UI_UpdateYawTextDrawing_() {
  std::fill(&yawTextMsg.message.text[0], &yawTextMsg.message.text[29], 0);
  yawTextMsg.message.figureConfig.operate = 2;
  float yawValue = 0.0f;
  if (SysBoardLink.otherInfo.pack_id == CDevBoardLink::PKT_OTHER_INFOS) {
    yawValue = static_cast<float>(SysBoardLink.otherInfo.yaw_gyro);
  }

  int32_t int_val = static_cast<int32_t>(yawValue * 1000.f);
  yawTextMsg.message.figureConfig.details_3 = int_val & 0x3FF;
  yawTextMsg.message.figureConfig.details_4 = (int_val >> 10) & 0x7FF;
  yawTextMsg.message.figureConfig.details_5 = (int_val >> 21) & 0x7FF;
  yawTextMsg.message.figureConfig.posit_X = 1520;
  yawTextMsg.message.figureConfig.posit_Y = 790;
  yawTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&yawTextMsg), sizeof(yawTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&yawTextMsg), sizeof(yawTextMsg));
}

void CSystemReferee::UI_UpdateSpeedTextDrawing_() {
  speedTextMsg.message.figureConfig.operate = 2;

  float vy = 0.0f;
  if (SysBoardLink.ctrlInfos.pack_id == CDevBoardLink::PKT_CTRL_INFOS) {
    vy = static_cast<float>(SysBoardLink.ctrlInfos.speed_y);
  }
  const float kSpeedWarn = 60.0f;
  speedTextMsg.message.figureConfig.color = (std::fabs(vy) > kSpeedWarn) ? 3 : 2;

  // 按你现有 yaw 的打包方式发 float
  const int32_t int_val = static_cast<int32_t>(vy * 1000.f);
  speedTextMsg.message.figureConfig.details_3 = int_val & 0x3FF;
  speedTextMsg.message.figureConfig.details_4 = (int_val >> 10) & 0x7FF;
  speedTextMsg.message.figureConfig.details_5 = (int_val >> 21) & 0x7FF;
  speedTextMsg.message.figureConfig.posit_X = 1520;
  speedTextMsg.message.figureConfig.posit_Y = 750;
  speedTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&speedTextMsg), sizeof(speedTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&speedTextMsg), sizeof(speedTextMsg));
}

void CSystemReferee::UI_UpdateSpinTextDrawing_() {
  spinTextMsg.message.figureConfig.operate = 2;
  const bool spinOn = (SysBoardLink.otherInfo.pack_id == CDevBoardLink::PKT_OTHER_INFOS)
      && (SysBoardLink.otherInfo.is_spin_on != 0);
  spinTextMsg.message.figureConfig.details_2 = spinOn ? 7 : 8;
  strcpy(reinterpret_cast<char *>(spinTextMsg.message.text), spinOn ? "SPIN:ON" : "SPIN:OFF");
  spinTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&spinTextMsg), sizeof(spinTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&spinTextMsg), sizeof(spinTextMsg));
}

void CSystemReferee::UI_UpdateGripTextDrawing_() {
  gripTextMsg.message.figureConfig.operate = 2;
  const bool gripOn = (SysBoardLink.angleInfo.pack_id == CDevBoardLink::PKT_JOINT_INFOS)
      && (SysBoardLink.angleInfo.grip_close != 0);
  gripTextMsg.message.figureConfig.details_2 = gripOn ? 7 : 8;
  strcpy(reinterpret_cast<char *>(gripTextMsg.message.text), gripOn ? "GRIP:ON" : "GRIP:OFF");
  gripTextMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&gripTextMsg), sizeof(gripTextMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&gripTextMsg), sizeof(gripTextMsg));
}
void CSystemReferee::UI_UpdatePositionFigureDrawing_() {
  auto *chassisModule = reinterpret_cast<CModChassis *>(ModuleIDMap.at(EModuleID::MOD_CHASSIS));
  float chassisYawDeg = 0.0f;
  if (chassisModule && chassisModule->filter && chassisModule->filter->Imu_Ave_Info.is_initialized) {
    chassisYawDeg = chassisModule->filter->Imu_Ave_Info.imu_ave_yaw;
  }

  float gimbalYawDeg = 0.0f;
  if (SysBoardLink.ctrlInfos.pack_id == CDevBoardLink::PKT_CTRL_INFOS
      && SysBoardLink.ctrlInfos.remote_is_online == 1) {
    gimbalYawDeg = chassisYawDeg;
  }

  float deltaYawDeg = chassisYawDeg - gimbalYawDeg;
  while (deltaYawDeg > 180.f) deltaYawDeg -= 360.f;
  while (deltaYawDeg < -180.f) deltaYawDeg += 360.f;

  constexpr float kCenterX = 960.0f;
  constexpr float kCenterY = 200.0f;
  constexpr float kRadius = 80.0f;
  constexpr float kWheelLineHalf = 24.0f;
  const float deltaYawRad = (deltaYawDeg + 90.0f) * PI / 180.0f;
  constexpr float kEcdToDeg = 360.0f / 8192.0f;

  // 轮子在示意圆中的固定位置（LF, RF, LB, RB）
  const float wheelPosX[4] = {920.0f, 1000.0f, 920.0f, 1000.0f};
  const float wheelPosY[4] = {240.0f, 240.0f, 160.0f, 160.0f};
  const float wheelSteerRawDeg[4] = {steer_angle_lf_deg, steer_angle_rf_deg, steer_angle_lb_deg, steer_angle_rb_deg};
  const float wheelMechMidDeg[4] = {
      600.0f * kEcdToDeg,
      7302.0f * kEcdToDeg,
      4700.0f * kEcdToDeg,
      7450.0f * kEcdToDeg,
  };

  auto normDeg = [](float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
  };

  positionFigureMsg.message.figureConfig[0].operate = 2;
  positionFigureMsg.message.figureConfig[1].operate = 2;
  positionFigureMsg.message.figureConfig[1].posit_X = static_cast<uint16_t>(kCenterX - kRadius * cosf(deltaYawRad));
  positionFigureMsg.message.figureConfig[1].posit_Y = static_cast<uint16_t>(kCenterY - kRadius * sinf(deltaYawRad));
  positionFigureMsg.message.figureConfig[1].details_4 = static_cast<uint16_t>(kCenterX + kRadius * cosf(deltaYawRad));
  positionFigureMsg.message.figureConfig[1].details_5 = static_cast<uint16_t>(kCenterY + kRadius * sinf(deltaYawRad));

  for (uint8_t i = 0; i < 4; ++i) {
    const uint8_t idx = static_cast<uint8_t>(2 + i);
    const float wheelSteerDeg = -normDeg(wheelSteerRawDeg[i] - wheelMechMidDeg[i]);
    const float wheelDirRad = deltaYawRad + wheelSteerDeg * PI / 180.0f;
    const float dx = kWheelLineHalf * cosf(wheelDirRad);
    const float dy = kWheelLineHalf * sinf(wheelDirRad);
    positionFigureMsg.message.figureConfig[idx].operate = 2;
    positionFigureMsg.message.figureConfig[idx].posit_X = static_cast<uint16_t>(wheelPosX[i] - dx);
    positionFigureMsg.message.figureConfig[idx].posit_Y = static_cast<uint16_t>(wheelPosY[i] - dy);
    positionFigureMsg.message.figureConfig[idx].details_4 = static_cast<uint16_t>(wheelPosX[i] + dx);
    positionFigureMsg.message.figureConfig[idx].details_5 = static_cast<uint16_t>(wheelPosY[i] + dy);
  }

  positionFigureMsg.message.figureConfig[6].operate = 2;
  positionFigureMsg.CRC16 = CCrcValidator::Crc16Calculate(reinterpret_cast<uint8_t *>(&positionFigureMsg), sizeof(positionFigureMsg) - 2);
  pInterface_->Transmit(reinterpret_cast<uint8_t *>(&positionFigureMsg), sizeof(positionFigureMsg));
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
	proc_waitMs(100);
	SysReferee.UI_StartCurModeTextDrawing_();
	proc_waitMs(100);
	SysReferee.UI_StartStateFigureDrawing_();
	proc_waitMs(100);
	// SysReferee.UI_StartVisionFigureDrawing_();
	// proc_waitMs(100);

  SysReferee.UI_StartRadarTextDrawing_();
  proc_waitMs(100);

  SysReferee.UI_StartPositionFigureDrawing_();  
  proc_waitMs(100);

	// 进入UI绘制循环
	while (true) {

    // 在这里面更新动态UI
		SysReferee.UI_UpdateCurModeTextDrawing_();
		proc_waitMs(50);

		// SysReferee.UI_UpdateVisionFigureDrawing_();
		// proc_waitMs(100);

		SysReferee.UI_UpdateStateFigureDrawing_();
		proc_waitMs(50);

    // SysReferee.UI_UpdateHipTextDrawing_();
    // proc_waitMs(50);

    SysReferee.UI_UpdateYawTextDrawing_();
    proc_waitMs(50);

    SysReferee.UI_UpdateSpeedTextDrawing_();
    proc_waitMs(50);

    SysReferee.UI_UpdateGripTextDrawing_();
    proc_waitMs(50);

    SysReferee.UI_UpdateSpinTextDrawing_();
    proc_waitMs(50);

    SysReferee.UI_UpdatePositionFigureDrawing_();
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
