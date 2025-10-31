/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-03-22
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-03-22   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#ifndef RP_SYS_COMMON_HPP
#define RP_SYS_COMMON_HPP

#include "Configuration.hpp"

namespace robotpilots {

class CSysInstance {
  friend void StartSystemUpdateTask(void *arg);
  friend void StartHeartbeatTask(void *arg);
protected:

  TaskHandle_t systemTaskHandle = nullptr;

  virtual void UpdateHandler_() { }

  virtual void HeartbeatHandler_() { }

  ERpStatus RegisterSystem_();

  ERpStatus UnregisterSystem_();

public:

  struct SSystemInitParam {
    ESystemID systemId = ESystemID::SYS_NULL;
  };

  ESystemID systemID = ESystemID::SYS_NULL;

  ERpStatus systemState = RP_RESET;

  CSysInstance() = default;

  virtual ~CSysInstance() { UnregisterSystem_(); }

  virtual ERpStatus InitSystem(SSystemInitParam *pStruct) = 0;

};

extern std::map<ESystemID, CSysInstance *> SystemMap;

}

#endif // RP_SYS_COMMON_HPP
