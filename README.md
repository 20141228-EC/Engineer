# RP_2026_ENGINEER_ROBOT

## 版本修改记录

### v1.0.0：

- 2025.11.13 ：
  
  1. 补充添加了部分的注释
  2. 修改了com_joint.cpp文件中115行：pitch2的初始化角度为90
  
- 2025.11.15

  1. 补充了空指针的判断 ，同时对当前赛季删除的内容也增加了判断和维护，保证后续赛季的可持续性
  2. 将查找方式从 .at() 改为使用 .find()，并检查迭代器有效性和指针非空。避免使用at时候异常的抛出。并且编译通过。
  3. 添加了部分注释

  | 修改的文件              | 修改内容                                                     | 修改段落数 |
  | ----------------------- | ------------------------------------------------------------ | ---------- |
  | Core.cpp                | InitSystemCore模块指针获取、HeartbeatHandler_、RESET_SYSTEM、Print语句、气泵控制 | 3          |
  | sys_remote.cpp          | InitSystem、HeartbeatHandler_、UpdateRemote_、UpdateKeyboard_ | 23         |
  | sys_esp32.cpp           | InitSystem、UpdateHandler_（包含大量电机状态检查）           | 2          |
  | sys_vison.cpp           | InitSystem、UpdateHandler、HeartbeatHandler、UpdateOreTankInfo、UpdateUiPointInfo | 2          |
  | control.cpp             | 对所有的遥控器控制前加了空指针的判断                         | 2          |
  | sys_controller_link.cpp | InitSystem、UpdateHandler、UpdateControllerLinkInfo、UpdateRobotInfo、UpdateRobotDataPkg、UpdateControllerDataPkg | 2          |
  | sys_referee.cpp         | InitSystem、HeartbeatHandler、UpdateRaceInfo、UpdateRobotInfo_、UpdateRadarInfo | 2          |
