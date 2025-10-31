/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-03-19
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-03-19   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#ifndef RP_MOD_COMMON_HPP
#define RP_MOD_COMMON_HPP

#include "Configuration.hpp"
#include "Algorithm.hpp"
#include "Interface.hpp"
#include "Device.hpp"

namespace robotpilots {

/**
 * @brief Module Instance Class
 */
class CModInstance {
  friend void StartUpdateTask(void *arg);
  friend void StartHeartbeatTask(void *arg);
protected:

  struct SModInitParam {
    EModuleID moduleId = EModuleID::MOD_NULL;
  };

  int16_t processFlag_ = 0;

  virtual void UpdateHandler_() { }

  virtual void HeartbeatHandler_() { }

  virtual ERpStatus CreateModuleTask_() { return RP_ERROR; }

  ERpStatus RegisterModule_();

  ERpStatus UnregisterModule_();

public:

  /**
   * @brief Module Component Class
   */
  class CComponent {
  public:

    ERpStatus componentState = RP_RESET;

    CComponent() = default;

    virtual ~CComponent() = default;

    virtual ERpStatus InitComponent(SModInitParam &param) = 0;

    virtual ERpStatus StartComponent();

    virtual ERpStatus StopComponent();

    virtual ERpStatus UpdateComponent() { return RP_ERROR; }

    virtual ERpStatus HeartbeatComponent() { return RP_ERROR; }

  protected:

    int16_t processFlag_ = 0;

  };

  EModuleID moduleId = EModuleID::MOD_NULL;

  ERpStatus moduleState = RP_RESET;

  TaskHandle_t  moduleTaskHandle = nullptr;

  CModInstance() = default;

  virtual ~CModInstance() { UnregisterModule_(); };

  virtual ERpStatus InitModule(SModInitParam &param) = 0;

  virtual ERpStatus StartModule();

  virtual ERpStatus StopModule();

};

extern std::map<EModuleID, CModInstance *> ModuleMap;

} // namespace robotpilots

#endif // RP_MOD_COMMON_HPP
