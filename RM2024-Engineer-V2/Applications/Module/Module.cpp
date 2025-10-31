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

#include "Module.hpp"

namespace robotpilots {

/**
 * @brief
 */
std::map<EModuleID, CModInstance *> ModuleMap;


/**
 * @brief
 * @return
 */
ERpStatus CModInstance::StartModule() {

  if (moduleState == RP_RESET) return RP_ERROR;

  processFlag_ = 1;

  moduleState = RP_BUSY;
  return RP_OK;
}


/**
 * @brief
 * @return
 */
ERpStatus CModInstance::StopModule() {

  if (moduleState == RP_RESET) return RP_ERROR;

  processFlag_ = 0;
  CreateModuleTask_();

  moduleState = RP_OK;
  return RP_OK;
}


/**
 * @brief
 * @return
 */
ERpStatus CModInstance::RegisterModule_() {

  if (moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  ModuleMap.insert(std::make_pair(moduleId, this));
  return RP_OK;
}

/**
 * @brief
 * @return
 */
ERpStatus CModInstance::UnregisterModule_() {

  if (moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  ModuleMap.erase(moduleId);
  return RP_OK;
}


/**
 * @brief
 * @return
 */
ERpStatus CModInstance::CComponent::StartComponent() {

  if (componentState == RP_RESET) return RP_ERROR;

  processFlag_ = 1;

  componentState = RP_BUSY;
  return RP_OK;
}


/**
 * @brief
 * @return
 */
ERpStatus CModInstance::CComponent::StopComponent() {

  if (componentState == RP_RESET) return RP_ERROR;

  processFlag_ = 0;

  componentState = RP_OK;
  return RP_OK;
}

} // namespace robotpilots
