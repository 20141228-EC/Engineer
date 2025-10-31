/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-18
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-18   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "Core.hpp"

#define deg2rad (3.1415926f / 180.0f)

namespace robotpilots {

/**
 * @brief Convert Joint Angle to Euler Angle
 * @param jointAngle[in] Pointer to Joint Angle (Unit: Rad, Joint_Yaw, Joint_Roll, End_Pitch)
 * @param eulerAngle[out] Pointer to Euler Angle (Unit: Rad, Yaw, Pitch, Roll)
 */
void JointAngleToEulerAngle (const float32_t *jointAngle, float32_t *eulerAngle) {

if (jointAngle == nullptr || eulerAngle == nullptr) return;

  /* Output Rotation Matrix */
  arm_matrix_instance_f32 rotationMatrix[2];
  float32_t rotationMatrixBuffer[2][9] = {{0}, {0}};
  arm_mat_init_f32(&rotationMatrix[0], 3, 3, rotationMatrixBuffer[0]);
  arm_mat_init_f32(&rotationMatrix[1], 3, 3, rotationMatrixBuffer[1]);

  /* Joint Rotation Matrix */
  arm_matrix_instance_f32 jointRotationMatrix[3];
  float32_t jointRotationMatrixBuffer[3][9] = {
    {   // Joint 1, Rotate around Y-Axis
      arm_cos_f32(jointAngle[0]), 0.0f, arm_sin_f32(jointAngle[0]),
      0.0f, 1.0f, 0.0f,
      -arm_sin_f32(jointAngle[0]), 0.0f, arm_cos_f32(jointAngle[0])
    },
    {   // Joint 2, Rotate around Z-Axis
      arm_cos_f32(jointAngle[1]), -arm_sin_f32(jointAngle[1]), 0.0f,
      arm_sin_f32(jointAngle[1]), arm_cos_f32(jointAngle[1]), 0.0f,
      0.0f, 0.0f, 1.0f
    },
    {   // Joint 3, Rotate around X-Axis
      1.0f, 0.0f, 0.0f,
      0.0f, arm_cos_f32(jointAngle[2]), -arm_sin_f32(jointAngle[2]),
      0.0f, arm_sin_f32(jointAngle[2]), arm_cos_f32(jointAngle[2])
    },
  };
  arm_mat_init_f32(&jointRotationMatrix[0], 3, 3, jointRotationMatrixBuffer[0]);
  arm_mat_init_f32(&jointRotationMatrix[1], 3, 3, jointRotationMatrixBuffer[1]);
  arm_mat_init_f32(&jointRotationMatrix[2], 3, 3, jointRotationMatrixBuffer[2]);

  /* Calculate Rotation Matrix */
  arm_mat_mult_f32(&jointRotationMatrix[1], &jointRotationMatrix[0], &rotationMatrix[0]);
  arm_mat_mult_f32(&jointRotationMatrix[2], &rotationMatrix[0], &rotationMatrix[1]);

  /* Calculate Euler Angle */
  arm_atan2_f32(rotationMatrixBuffer[1][6], rotationMatrixBuffer[1][8], &eulerAngle[0]);
  eulerAngle[1] = asinf(-rotationMatrixBuffer[1][7]);
  arm_atan2_f32(rotationMatrixBuffer[1][3], rotationMatrixBuffer[1][4], &eulerAngle[2]);

//  eulerAngle[0] = atan2f(rotationMatrixBuffer[1][6], rotationMatrixBuffer[1][8]) / deg2rad;
//  eulerAngle[1] = asinf(-rotationMatrixBuffer[1][7]) / deg2rad;
//  eulerAngle[2] = atan2f(rotationMatrixBuffer[1][3], rotationMatrixBuffer[1][4]) / deg2rad;
}


/**
 * @brief
 * @param arg
 */
void CSystemCore::StartExchangeTask(void *arg) {

  if (arg == nullptr) proc_return();

  /* Get System Core Handle */
  auto &core = *reinterpret_cast<CSystemCore *>(arg);
  auto cnt = 0;
  const auto timeout = 60000 / 5;    // unit: ms
  CSysVision::SOreTankInfo oreTankInfo;

  /* Set Auto Control Flag */
  core.gantry_->gantryCmd.isAutoCtrl = true;
  core.subgantry_->subGantryCmd.isAutoCtrl = true;

  while (SysRemote.remoteInfo.keyboard.key_Ctrl) {

    /* Manual Exchange - Left */
    if (SysRemote.remoteInfo.keyboard.mouse_L) {

      /* Step 1 */
      core.gimbal_->gimbalCmd.setPosit_Lift = 0.0f;
      core.gimbal_->gimbalCmd.setStep_Pitch = 1;

      core.subgantry_->subGantryCmd.setPosit_Lift_L = 0.0f;
      core.subgantry_->subGantryCmd.setPosit_Lift_R = 0.0f;
      core.subgantry_->subGantryCmd.setPosit_Stretch_L = 0.0f;
      core.subgantry_->subGantryCmd.setPosit_Stretch_R = 0.0f;

      core.gantry_->gantryCmd.setPumpOn_C = true;
      core.gantry_->gantryCmd.setPosit_Lift = 560.0f;
      core.gantry_->gantryCmd.setPosit_Stretch = 400.0f;
      core.gantry_->gantryCmd.setPosit_Traverse = 0.0f;
      core.gantry_->gantryCmd.setAngle_Joint_Yaw = 0.0f;
      core.gantry_->gantryCmd.setAngle_Joint_Roll = -90.0f;
      core.gantry_->gantryCmd.setAngle_End_Pitch = 0.0f;
      core.gantry_->gantryCmd.setAngle_End_Roll -= 90.0f;
      if (core.gantry_->gantryInfo.posit_Lift < 300.0f)
        proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Stretch);

      /* Wait for User Confirmation */
      cnt = 60000*3;
      core.gantry_->gantryCmd.isAutoCtrl = false;      // Unlock Gantry
      proc_waitMs(500);
      while (cnt--) {
        if (SysRemote.remoteInfo.keyboard.key_Ctrl) {
          if (SysRemote.remoteInfo.keyboard.mouse_L) break;
          if (SysRemote.remoteInfo.keyboard.mouse_R) goto proc_exit;
        }
        proc_waitMs(1);
      }
      if (cnt == 0) goto proc_exit;

      /* Step 2 */
      core.gantry_->gantryCmd.setAngle_End_Pitch = -90.0f;
      core.gantry_->gantryCmd.setAngle_Joint_Yaw = -20.0f;
      core.subgantry_->subGantryCmd.setPosit_Lift_L = 120.0f;
      core.subgantry_->subGantryCmd.setPosit_Lift_R = 120.0f;

      /* Wait for User Confirmation */
      cnt = 60000*3;
      core.gantry_->gantryCmd.isAutoCtrl = false;      // Unlock Gantry
      proc_waitMs(500);
      while (cnt--) {
        if (SysRemote.remoteInfo.keyboard.key_Ctrl) {
          if (SysRemote.remoteInfo.keyboard.mouse_L) break;
          if (SysRemote.remoteInfo.keyboard.mouse_R) goto proc_exit;
          if (SysRemote.remoteInfo.keyboard.mouse_Thumb != 0) {
            float32_t sign = SysRemote.remoteInfo.keyboard.mouse_Thumb > 0 ? 0.5f : -0.5f;
            float32_t jointAngle[3] = {0};
            float32_t eulerAngle[3] = {0};
            jointAngle[0] = core.gantry_->gantryInfo.angle_Joint_Yaw * deg2rad;
            jointAngle[1] = core.gantry_->gantryInfo.angle_Joint_Roll * deg2rad;
            jointAngle[2] = core.gantry_->gantryInfo.angle_End_Pitch * deg2rad;
            JointAngleToEulerAngle(jointAngle, eulerAngle);
            core.gantry_->gantryCmd.setPosit_Lift -=
              sign * arm_sin_f32(eulerAngle[1]);
            core.gantry_->gantryCmd.setPosit_Stretch +=
              sign * arm_cos_f32(eulerAngle[0]) * arm_cos_f32(eulerAngle[1]);
            core.gantry_->gantryCmd.setPosit_Traverse +=
              sign * arm_sin_f32(eulerAngle[0]) * arm_cos_f32(eulerAngle[1]);
          }
        }
        proc_waitMs(1);
      }
      if (cnt == 0) goto proc_exit;

      /* Step 3 */
      core.gantry_->gantryCmd.setPumpOn_C = false;

      /* Wait for User Confirmation */
      cnt = 60 * 1000 * 3;
      core.gantry_->gantryCmd.isAutoCtrl = false;      // Unlock Gantry
      proc_waitMs(500);
      while (cnt--) {
        if (SysRemote.remoteInfo.keyboard.key_Ctrl) {
          if (SysRemote.remoteInfo.keyboard.mouse_L) break;
          if (SysRemote.remoteInfo.keyboard.mouse_R) goto proc_exit;
        }
        proc_waitMs(1);
      }
      if (cnt == 0) goto proc_exit;

      /* Step 4 */
      core.gimbal_->gimbalCmd.setPosit_Lift = 150.0f;
      core.gimbal_->gimbalCmd.setStep_Pitch = 0;
      core.subgantry_->subGantryCmd.setPosit_Lift_L = 0.0f;
      core.subgantry_->subGantryCmd.setPosit_Lift_R = 0.0f;
      core.gantry_->gantryCmd.setPosit_Traverse = 190.0f / 2;
      core.gantry_->gantryCmd.setPosit_Stretch = 0.0f;
      core.gantry_->gantryCmd.setPosit_Lift = 200.0f;
      core.gantry_->gantryCmd.setAngle_Joint_Yaw = 0.0f;
      core.gantry_->gantryCmd.setAngle_Joint_Roll = 0.0f;
      core.gantry_->gantryCmd.setAngle_End_Pitch = 0.0f;
      core.gantry_->gantryCmd.setAngle_End_Roll = 0.0f;

      goto proc_exit;
    }

    /* Manual Exchange - Right */
    if (SysRemote.remoteInfo.keyboard.mouse_R) {

      /* Step 1 */
      core.gimbal_->gimbalCmd.setPosit_Lift = 0.0f;
      core.gimbal_->gimbalCmd.setStep_Pitch = 1;

      core.subgantry_->subGantryCmd.setPosit_Lift_L = 0.0f;
      core.subgantry_->subGantryCmd.setPosit_Lift_R = 0.0f;
      core.subgantry_->subGantryCmd.setPosit_Stretch_L = 0.0f;
      core.subgantry_->subGantryCmd.setPosit_Stretch_R = 0.0f;

      core.gantry_->gantryCmd.setPumpOn_C = true;
      core.gantry_->gantryCmd.setPosit_Lift = 560.0f;
      core.gantry_->gantryCmd.setPosit_Stretch = 400.0f;
      core.gantry_->gantryCmd.setPosit_Traverse = 190.0f;
      core.gantry_->gantryCmd.setAngle_Joint_Yaw = 0.0f;
      core.gantry_->gantryCmd.setAngle_Joint_Roll = 90.0f;
      core.gantry_->gantryCmd.setAngle_End_Pitch = 0.0f;
      core.gantry_->gantryCmd.setAngle_End_Roll += 90.0f;
      if (core.gantry_->gantryInfo.posit_Lift < 300.0f)
        proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Stretch);

      /* Wait for User Confirmation */
      cnt = 60000*3;
      core.gantry_->gantryCmd.isAutoCtrl = false;      // Unlock Gantry
      proc_waitMs(500);
      while (cnt--) {
        if (SysRemote.remoteInfo.keyboard.key_Ctrl) {
          if (SysRemote.remoteInfo.keyboard.mouse_L) break;
          if (SysRemote.remoteInfo.keyboard.mouse_R) goto proc_exit;
        }
        proc_waitMs(1);
      }
      if (cnt == 0) goto proc_exit;

      /* Step 2 */
      core.gantry_->gantryCmd.setAngle_End_Pitch = -90.0f;
      core.gantry_->gantryCmd.setAngle_Joint_Yaw = 20.0f;
      core.subgantry_->subGantryCmd.setPosit_Lift_L = 120.0f;
      core.subgantry_->subGantryCmd.setPosit_Lift_R = 120.0f;

      /* Wait for User Confirmation */
      cnt = 60000*3;
      core.gantry_->gantryCmd.isAutoCtrl = false;      // Unlock Gantry
      proc_waitMs(500);
      while (cnt--) {
        if (SysRemote.remoteInfo.keyboard.key_Ctrl) {
          if (SysRemote.remoteInfo.keyboard.mouse_L) break;
          if (SysRemote.remoteInfo.keyboard.mouse_R) goto proc_exit;
          if (SysRemote.remoteInfo.keyboard.mouse_Thumb != 0) {
            float32_t sign = SysRemote.remoteInfo.keyboard.mouse_Thumb > 0 ? 0.5f : -0.5f;
            float32_t jointAngle[3] = {0};
            float32_t eulerAngle[3] = {0};
            jointAngle[0] = core.gantry_->gantryInfo.angle_Joint_Yaw * deg2rad;
            jointAngle[1] = core.gantry_->gantryInfo.angle_Joint_Roll * deg2rad;
            jointAngle[2] = core.gantry_->gantryInfo.angle_End_Pitch * deg2rad;
            JointAngleToEulerAngle(jointAngle, eulerAngle);
            core.gantry_->gantryCmd.setPosit_Lift -=
              sign * arm_sin_f32(eulerAngle[1]);
            core.gantry_->gantryCmd.setPosit_Stretch +=
              sign * arm_cos_f32(eulerAngle[0]) * arm_cos_f32(eulerAngle[1]);
            core.gantry_->gantryCmd.setPosit_Traverse +=
              sign * arm_sin_f32(eulerAngle[0]) * arm_cos_f32(eulerAngle[1]);
          }
        }
        if (core.gantry_->gantryCmd.setPosit_Lift < 350.0f)
          core.gimbal_->gimbalCmd.setPosit_Lift = 280.0f;
        proc_waitMs(1);
      }
      if (cnt == 0) goto proc_exit;

      /* Step 3 */
      core.gantry_->gantryCmd.setPumpOn_C = false;

      /* Wait for User Confirmation */
      cnt = 60 * 1000 * 3;
      core.gantry_->gantryCmd.isAutoCtrl = false;      // Unlock Gantry
      proc_waitMs(500);
      while (cnt--) {
        if (SysRemote.remoteInfo.keyboard.key_Ctrl) {
          if (SysRemote.remoteInfo.keyboard.mouse_L) break;
          if (SysRemote.remoteInfo.keyboard.mouse_R) goto proc_exit;
        }
        proc_waitMs(1);
      }
      if (cnt == 0) goto proc_exit;

      /* Step 4 */
      core.gimbal_->gimbalCmd.setPosit_Lift = 150.0f;
      core.gimbal_->gimbalCmd.setStep_Pitch = 0;
      core.subgantry_->subGantryCmd.setPosit_Lift_L = 0.0f;
      core.subgantry_->subGantryCmd.setPosit_Lift_R = 0.0f;
      core.gantry_->gantryCmd.setPosit_Traverse = 190.0f / 2;
      core.gantry_->gantryCmd.setPosit_Stretch = 0.0f;
      core.gantry_->gantryCmd.setPosit_Lift = 200.0f;
      core.gantry_->gantryCmd.setAngle_Joint_Yaw = 0.0f;
      core.gantry_->gantryCmd.setAngle_Joint_Roll = 0.0f;
      core.gantry_->gantryCmd.setAngle_End_Pitch = 0.0f;
      core.gantry_->gantryCmd.setAngle_End_Roll = 0.0f;

      goto proc_exit;
    }

    proc_waitMs(5);
  }

  /* Auto Exchange */

  /* Step 1 */
  core.gimbal_->gimbalCmd.setPosit_Lift = 280.0f;
  core.gimbal_->gimbalCmd.setStep_Pitch = 3;
  core.subgantry_->subGantryCmd.setPosit_Lift_L = 0.0f;
  core.subgantry_->subGantryCmd.setPosit_Lift_R = 0.0f;
  core.subgantry_->subGantryCmd.setPosit_Stretch_L = 0.0f;
  core.subgantry_->subGantryCmd.setPosit_Stretch_R = 0.0f;
  core.gantry_->gantryCmd.setPosit_Lift = 250.0f;
  core.gantry_->gantryCmd.setPosit_Stretch = 0.0f;
  core.gantry_->gantryCmd.setPosit_Traverse = 190.0f / 2;
  core.gantry_->gantryCmd.setAngle_Joint_Yaw = 0.0f;
  core.gantry_->gantryCmd.setAngle_Joint_Roll = 0.0f;
  core.gantry_->gantryCmd.setAngle_End_Pitch = -90.0f;
  proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Joint_Yaw
                 && core.gantry_->gantryInfo.isPositArrived_Joint_Roll);
  proc_waitMs(500);
  core.gantry_->gantryCmd.setAngle_End_Pitch = -90.0f;

  /* Wait for Vision Identification & User Confirmation */
  cnt = 60 * 1000 * 2;    // 120s
  core.gantry_->gantryCmd.isAutoCtrl = false;
  while (cnt--) {
    if (SysRemote.remoteInfo.keyboard.key_Ctrl
        && SysRemote.remoteInfo.keyboard.mouse_L
        && SysVision.visionInfo.oreTank.isFoundOreTank) {
      oreTankInfo = SysVision.visionInfo.oreTank;
      break;
    }
    if (SysRemote.remoteInfo.keyboard.key_Ctrl
        && SysRemote.remoteInfo.keyboard.mouse_R) goto proc_exit;
    if (SysVision.systemState != RP_OK) goto proc_exit;
    proc_waitMs(5);
  }
  if (cnt == 0) goto proc_exit;

  core.gantry_->gantryCmd.isAutoCtrl = true;
  if (abs(oreTankInfo.atti_YAW) < 80.0f) {    // Level 4 Exchange
    /* Step 2 */
    core.gimbal_->gimbalCmd.setPosit_Lift = 0.0f;
    core.gimbal_->gimbalCmd.setStep_Pitch = 1;
//    core.gantry_->gantryCmd.setPosit_Lift = 550.0f + arm_sin_f32(oreTankInfo.atti_PITCH * deg2rad) * 150.0f;
    core.gantry_->gantryCmd.setPosit_Lift = 660.0f;
    core.gantry_->gantryCmd.setPosit_Stretch = 200.0f;
    core.gantry_->gantryCmd.setPosit_Traverse   = (oreTankInfo.atti_YAW > 0.0f) ? 0.0f : 190.0f;
    core.gantry_->gantryCmd.setAngle_Joint_Yaw  = 0.0f;
    core.gantry_->gantryCmd.setAngle_Joint_Roll = -180.0f;
    proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Lift);
    core.subgantry_->subGantryCmd.setPosit_Lift_L = 120.0f;
    core.subgantry_->subGantryCmd.setPosit_Lift_R = 120.0f;


    /* Wait for User Confirmation */
    cnt = 60 * 1000 * 3;
    core.gantry_->gantryCmd.isAutoCtrl = false;      // Unlock Gantry
    proc_waitMs(500);
    while (cnt--) {
      if (SysRemote.remoteInfo.keyboard.key_Ctrl) {
        if (SysRemote.remoteInfo.keyboard.mouse_L) break;
        if (SysRemote.remoteInfo.keyboard.mouse_R) goto proc_exit;
      }
      proc_waitMs(1);
    }
    if (cnt == 0) goto proc_exit;

    /* Step 3 */
    core.gantry_->gantryCmd.setAngle_Joint_Yaw = -oreTankInfo.atti_YAW;
    core.gantry_->gantryCmd.setAngle_End_Pitch = -(oreTankInfo.atti_PITCH - 3.0f);
//    core.gantry_->gantryCmd.setAngle_End_Roll += oreTankInfo.atti_ROLL;
    if (oreTankInfo.atti_YAW > 0.0f)
      core.gantry_->gantryCmd.setAngle_End_Roll -= oreTankInfo.atti_ROLL + 180.0f;
    else
      core.gantry_->gantryCmd.setAngle_End_Roll -= oreTankInfo.atti_ROLL + 180.0f;

  } else {    // Level 5 Exchange
    /* Step 2 */
    core.gimbal_->gimbalCmd.setPosit_Lift = 0.0f;
    core.gimbal_->gimbalCmd.setStep_Pitch = 1;
//    core.gantry_->gantryCmd.setPosit_Lift = 660.0f;
    core.gantry_->gantryCmd.setPosit_Lift =
      480.0f - oreTankInfo.posit_Y - arm_sin_f32(oreTankInfo.atti_PITCH * deg2rad) * 330.0f;
    core.gantry_->gantryCmd.setPosit_Lift =
      std::clamp(core.gantry_->gantryCmd.setPosit_Lift, 300.0f, 660.0f);
    core.gantry_->gantryCmd.setPosit_Traverse   = (oreTankInfo.atti_YAW > 0.0f) ? 0.0f : 190.0f;
    core.gantry_->gantryCmd.setAngle_Joint_Yaw  = 0.0f;
    core.gantry_->gantryCmd.setAngle_Joint_Roll = (oreTankInfo.atti_YAW > 0.0f) ? -90.0f : 90.0f;
    core.gantry_->gantryCmd.setAngle_End_Pitch  = 0.0f;
    proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_Lift
                   && core.gantry_->gantryInfo.isPositArrived_Traverse);
    core.gantry_->gantryCmd.setPosit_Stretch    = 400.0f;
    core.subgantry_->subGantryCmd.setPosit_Lift_L = 120.0f;
    core.subgantry_->subGantryCmd.setPosit_Lift_R = 120.0f;

    /* Wait for User Confirmation */
    cnt = 60 * 1000 * 3;
    core.gantry_->gantryCmd.isAutoCtrl = false;      // Unlock Gantry
    proc_waitMs(500);
    while (cnt--) {
      if (SysRemote.remoteInfo.keyboard.key_Ctrl) {
        if (SysRemote.remoteInfo.keyboard.mouse_L) break;
        if (SysRemote.remoteInfo.keyboard.mouse_R) goto proc_exit;
      }
      proc_waitMs(1);
    }
    if (cnt == 0) goto proc_exit;

    /* Step 3 */
    core.gantry_->gantryCmd.setAngle_End_Pitch = -90.0f;
    proc_waitUntil(core.gantry_->gantryInfo.isPositArrived_End_Pitch);
    proc_waitMs(200);
    if (oreTankInfo.atti_YAW > 0.0f) {    // Level 5 Exchange (Left)
      core.gantry_->gantryCmd.setAngle_Joint_Yaw = -(oreTankInfo.atti_YAW - 90.0f) * 0.8f;
      core.gantry_->gantryCmd.setAngle_Joint_Roll = -(oreTankInfo.atti_PITCH - 3.0f) - 90.0f;
      core.gantry_->gantryCmd.setAngle_End_Roll -= 90.0f + oreTankInfo.atti_ROLL;
    } else {    // Level 5 Exchange (Right)
      core.gantry_->gantryCmd.setAngle_Joint_Yaw = -(oreTankInfo.atti_YAW + 90.0f) * 0.8f;
      core.gantry_->gantryCmd.setAngle_Joint_Roll = (oreTankInfo.atti_PITCH - 3.0f) + 90.0f;
      core.gantry_->gantryCmd.setAngle_End_Roll += 90.0f - oreTankInfo.atti_ROLL;
    }
  }

  /* Wait for User Confirmation */
  cnt = 60 * 1000 * 3;    // 180s
  core.gantry_->gantryCmd.isAutoCtrl = false;      // Unlock Gantry
  proc_waitMs(500);
  while (cnt--) {
    if (SysRemote.remoteInfo.keyboard.key_Ctrl
        && SysRemote.remoteInfo.keyboard.mouse_L) break;
    if (SysRemote.remoteInfo.keyboard.key_Ctrl
        && SysRemote.remoteInfo.keyboard.mouse_R) goto proc_exit;
    if (SysRemote.remoteInfo.keyboard.key_Ctrl
        && SysRemote.remoteInfo.keyboard.mouse_Thumb != 0) {
      float32_t sign          = SysRemote.remoteInfo.keyboard.mouse_Thumb > 0 ? 0.5f : -0.5f;
      float32_t jointAngle[3] = {0};
      float32_t eulerAngle[3] = {0};
      jointAngle[0]           = core.gantry_->gantryInfo.angle_Joint_Yaw * deg2rad;
      jointAngle[1]           = core.gantry_->gantryInfo.angle_Joint_Roll * deg2rad;
      jointAngle[2]           = core.gantry_->gantryInfo.angle_End_Pitch * deg2rad;
      JointAngleToEulerAngle(jointAngle, eulerAngle);
      core.gantry_->gantryCmd.setPosit_Lift -=
        sign * arm_sin_f32(eulerAngle[1]);
      core.gantry_->gantryCmd.setPosit_Stretch +=
        sign * arm_cos_f32(eulerAngle[0]) * arm_cos_f32(eulerAngle[1]);
      core.gantry_->gantryCmd.setPosit_Traverse +=
        sign * arm_sin_f32(eulerAngle[0]) * arm_cos_f32(eulerAngle[1]);
    }
    if (core.gantry_->gantryCmd.setPosit_Lift < 350.0f)
      core.gimbal_->gimbalCmd.setPosit_Lift = 280.0f;
    proc_waitMs(1);
  }
  if (cnt == 0) goto proc_exit;

  /* Step 4 */
  core.gantry_->gantryCmd.setPumpOn_C = false;

  /* Wait for User Confirmation */
  cnt = 60 * 1000 * 3;
  core.gantry_->gantryCmd.isAutoCtrl = false;      // Unlock Gantry
  proc_waitMs(500);
  while (cnt--) {
    if (SysRemote.remoteInfo.keyboard.key_Ctrl) {
      if (SysRemote.remoteInfo.keyboard.mouse_L) break;
      if (SysRemote.remoteInfo.keyboard.mouse_R) goto proc_exit;
    }
    proc_waitMs(1);
  }
  if (cnt == 0) goto proc_exit;

  /* Step 5 */
  core.gimbal_->gimbalCmd.setPosit_Lift = 150.0f;
  core.gimbal_->gimbalCmd.setStep_Pitch = 0;
  core.subgantry_->subGantryCmd.setPosit_Lift_L = 0.0f;
  core.subgantry_->subGantryCmd.setPosit_Lift_R = 0.0f;
  core.gantry_->gantryCmd.setPosit_Traverse = 190.0f / 2;
  core.gantry_->gantryCmd.setPosit_Stretch = 0.0f;
  core.gantry_->gantryCmd.setPosit_Lift = 200.0f;
  core.gantry_->gantryCmd.setAngle_Joint_Yaw = 0.0f;
  core.gantry_->gantryCmd.setAngle_Joint_Roll = 0.0f;
  core.gantry_->gantryCmd.setAngle_End_Pitch = 0.0f;
  core.gantry_->gantryCmd.setAngle_End_Roll = 0.0f;

  /* Process Exit */
proc_exit:
  core.gimbal_->gimbalCmd.isAutoCtrl = false;
  core.gantry_->gantryCmd.isAutoCtrl = false;
  core.subgantry_->subGantryCmd.isAutoCtrl = false;
  core.autoCtrlTaskHandle_ = nullptr;
  core.currentAutoCtrlProcess_ = EAutoCtrlProcess::NONE;
  proc_return();
}

} // namespace robotpilots
