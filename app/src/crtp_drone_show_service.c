/*
 * Crazyflie CRTP protocol extension for supporting Skybrush-specific functionality.
 *
 * This file is part of the Skybrush compatibility layer for the Crazyflie firmware.
 *
 * Copyright 2020-2022 CollMot Robotics Ltd.
 *
 * This app is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This app is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "FreeRTOS.h"
#include "timers.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include "arming.h"
#include "commander.h"
#include "crtp.h"
#include "crtp_drone_show_service.h"
#include "drone_show.h"
#include "estimator_kalman.h"
#include "fence.h"
#include "gcs_light_effects.h"
#include "light_program.h"
#include "log.h"
#include "param.h"
#include "pm.h"
#include "preflight.h"
#include "supervisor.h"
#include "system.h"

#include "stabilizer_types.h"
#include "quatcompress.h"
#include "crtp_localization_service.h"
#include "controller_lqr_1dof.h"
#include "controller_lqr_2dof.h"

#define DEBUG_MODULE "SHOW"
#include "debug.h"

#define CONTROL_CH 0
#define LOAD_POSE_CH 1

#define CMD_START  0
#define CMD_PAUSE  1
#define CMD_STOP   2
#define CMD_STATUS 3
#define CMD_DEFINE_LIGHT_PROGRAM 4
#define CMD_RESTART 5
#define CMD_TRIGGER_GCS_LIGHT_EFFECT 6
#define CMD_ARM_OR_DISARM 7
#define CMD_DEFINE_FENCE 8


struct data_arm_or_disarm {
  uint8_t arm;  /* 0 = disarm, 1 = arm, 2 = force-disarm, 3 = force-arm (not used yet) */
} __attribute__((packed));

struct data_define_fence {
  struct fenceLocationDescription description;
} __attribute__((packed));

struct data_define_program {
  uint8_t programId;
  struct lightProgramDescription description;
} __attribute__((packed));

struct data_trigger_light_effect {
  uint8_t effect;
  uint8_t color[3];
} __attribute__((packed));

struct data_load_pose_packed {
  uint8_t id; // 
  int16_t x; // mm
  int16_t y; // mm
  int16_t z; // mm
  uint32_t quat; // compressed quaternion, see quatcompress.h
} __attribute__((packed));  // similar to extPosePackedItem in crtp_localization_service

struct data_load_pose {
  float x; // in m
  float y; // in m
  float z; // in m
  float qx;
  float qy;
  float qz;
  float qw;
} __attribute__((packed));  // similar to CrtpExtPose in crtp_localization_service

static bool isInit = false;
static poseMeasurement_t load_pose;
static uint8_t payload_id = 12;
static uint8_t cur_id = 0;
static uint32_t now_ms;
static uint32_t load_pose_ctr;

static struct {
  logVarId_t stateEstimateYaw;
  logVarId_t ledColorRed;
  logVarId_t ledColorGreen;
  logVarId_t ledColorBlue;
} logIds;

static void droneShowSrvCrtpCB(CRTPPacket* pk);
static void handleArmOrDisarmCommandPacket(CRTPPacket* pk);
static void handleDefineFencePacket(CRTPPacket* pk);
static void handleDefineLightProgramPacket(CRTPPacket* pk);
static void handleTriggerGcsLightEffectPacket(CRTPPacket* pk);
static void updatePacketWithStatusInformation(CRTPPacket* pk);
static void droneShowSrvLoadPosePacket(CRTPPacket* pk);
static void handleLoadPosePackedPacket(CRTPPacket* pk);
static void handleLoadPosePacket(CRTPPacket* pk);

void droneShowSrvInit() {
  if (isInit) {
    return;
  }

  /* Retrieve the IDs of the log variables and parameters that we will need */
  logIds.ledColorRed = logGetVarId("show", "ledColorRed");
  logIds.ledColorGreen = logGetVarId("show", "ledColorGreen");
  logIds.ledColorBlue = logGetVarId("show", "ledColorBlue");
  logIds.stateEstimateYaw = logGetVarId("stateEstimate", "yaw");

  if (!logVarIdIsValid(logIds.stateEstimateYaw)) {
    return;
  }

  crtpRegisterPortCB(CRTP_PORT_APP, droneShowSrvCrtpCB);

  isInit = true;
}

bool droneShowSrvTest(void) {
  return isInit;
}

static void droneShowSrvProcessControlPacket(CRTPPacket* pk) {
  if (pk->size < 1) {
    return;
  }

  /* commands are acknowledged by sending the same packet back.
   * status requests are responded to in the same packet */

  switch (pk->data[0]) {
    case CMD_START:
      if (pk->size > 2) {
        droneShowDelayedStart(*(int16_t*)(pk->data + 1));
      } else {
        droneShowStart();
      }
      pk->size = 1;
      break;

    case CMD_PAUSE:
      droneShowPause();
      break;

    case CMD_STOP:
      droneShowStop();
      break;

    case CMD_STATUS:
      updatePacketWithStatusInformation(pk);
      break;

    case CMD_RESTART:
      droneShowRestart();
      break;

    case CMD_DEFINE_LIGHT_PROGRAM:
      handleDefineLightProgramPacket(pk);
      break;

    case CMD_TRIGGER_GCS_LIGHT_EFFECT:
      handleTriggerGcsLightEffectPacket(pk);
      break;

    case CMD_ARM_OR_DISARM:
      handleArmOrDisarmCommandPacket(pk);
      break;

    case CMD_DEFINE_FENCE:
      handleDefineFencePacket(pk);
      break;

    default:
      return;
  }

  crtpSendPacket(pk);
}

static void droneShowSrvLoadPosePacket(CRTPPacket* pk) {
  if (pk->size < 1) {
    return;
  }

  /* commands are acknowledged by sending the same packet back.
   * status requests are responded to in the same packet */
  if (pk->data[0] == EXT_POSE_PACKED){
    handleLoadPosePackedPacket(pk);
  }
  else if (pk->data[0] == EXT_POSE){
    handleLoadPosePacket(pk);
  }

  crtpSendPacket(pk);
}

static void droneShowSrvCrtpCB(CRTPPacket* pk) {
  if (pk->channel == CONTROL_CH) {
    droneShowSrvProcessControlPacket(pk);
  } else if (pk->channel == LOAD_POSE_CH) {
    droneShowSrvLoadPosePacket(pk);
  }
}

static void handleLoadPosePackedPacket(CRTPPacket* pk) {
  // similar to extPosePackedHandler in crtp_localization_service
  struct data_load_pose_packed data = *((struct data_load_pose_packed*)(pk->data + 1));
  uint32_t now_ms_new = T2M(xTaskGetTickCount());
  if ((data.id == payload_id + cur_id) && (now_ms_new - now_ms > 16)) {
    load_pose_ctr++;
    load_pose.x = data.x / 1000.0f;
    load_pose.y = data.y / 1000.0f;
    load_pose.z = data.z / 1000.0f;
    quatdecompress(data.quat, (float *)&load_pose.quat.q0);
    // load_pose.stdDevPos = 1;  // not needed yet
    // load_pose.stdDevQuat = 1;
    // estimatorEnqueuePose(&ext_pose);
    // setLoadState1Dof(&load_pose, now_ms_new-now_ms);
    setLoadState2Dof(&load_pose, now_ms_new-now_ms);
    now_ms = now_ms_new;
    cur_id = 1 - cur_id;
  }
}


static void handleLoadPosePacket(CRTPPacket* pk) {
  struct data_load_pose data = *((struct data_load_pose*)(pk->data +1));
  uint32_t now_ms_new = T2M(xTaskGetTickCount());
  if (now_ms_new - now_ms > 16) {
    load_pose_ctr++;
    load_pose.x = data.x;
    load_pose.y = data.y;
    load_pose.z = data.z;
    load_pose.quat.q0 = data.qx;
    load_pose.quat.q1 = data.qy;
    load_pose.quat.q2 = data.qz;
    load_pose.quat.q3 = data.qw;
    // load_pose.stdDevPos = extPosStdDev;
    // load_pose.stdDevQuat = extQuatStdDev;
    // setLoadState1Dof(&load_pose, now_ms_new-now_ms);
    setLoadState2Dof(&load_pose, now_ms_new-now_ms);
    now_ms = now_ms_new;
  }
}


static void handleArmOrDisarmCommandPacket(CRTPPacket* pk) {
  struct data_arm_or_disarm data = *((struct data_arm_or_disarm*)(pk->data + 1));
  
  /* put the response in the packet and trim it */
  if (pk->size >= sizeof(struct data_arm_or_disarm) + 1) {
    bool shouldBeArmed = data.arm & 1;
    bool forced = data.arm & 2;

    systemSetArmed(shouldBeArmed);
    if (!shouldBeArmed && forced) {
      armingForceDisarm();
    }

    pk->size = 3;
    pk->data[2] = 0;
  } else {
    pk->size = 3;
    pk->data[2] = EINVAL;
  }
}

static void handleDefineFencePacket(CRTPPacket* pk) {
  struct data_define_fence data = *((struct data_define_fence*)(pk->data + 1));

  /* put the response in the packet and trim it */
  if (pk->size >= sizeof(struct data_define_fence) + 1) {
    pk->size = 3;
    pk->data[2] = fenceSetup(&data.description);
  } else {
    pk->size = 3;
    pk->data[2] = EINVAL;
  }
}

static void handleDefineLightProgramPacket(CRTPPacket* pk) {
  struct data_define_program data = *((struct data_define_program*)(pk->data + 1));

  /* put the response in the packet and trim it */
  if (pk->size >= sizeof(struct data_define_program) + 1) {
    pk->size = 3;
    pk->data[2] = lightProgramPlayerDefineProgram(data.programId, data.description);
  } else {
    pk->size = 3;
    pk->data[2] = EINVAL;
  }
}

static void handleTriggerGcsLightEffectPacket(CRTPPacket* pk) {
  struct data_trigger_light_effect data = *((struct data_trigger_light_effect*)(pk->data + 1));
  uint8_t color[3];

  /* put the response in the packet and trim it */
  if (pk->size >= sizeof(struct data_trigger_light_effect) + 1) {
    color[0] = data.color[0];
    color[1] = data.color[1];
    color[2] = data.color[2];

    pk->size = 3;
    pk->data[2] = gcsLightEffectTrigger(data.effect, color);
  } else {
    pk->size = 3;
    pk->data[2] = EINVAL;
  }
}

static void updatePacketWithStatusInformation(CRTPPacket* pk) {
  static logVarId_t setpointXVarId = 0xffffu, setpointYVarId, setpointZVarId;
  uint16_t voltage = ((int)round(pmGetBatteryVoltage() * 10));
  uint16_t colorAsRGB565;
  const uint8_t* lastColor;
  point_t pos;
  show_state_t show_state;

  pk->size = 16;

  /* clear the packet body */
  memset(pk->data + 1, 0, pk->size - 1);

  /* version number and state */
  show_state = droneShowGetState();
  pk->data[1] = (0 << 4) | (show_state & 0x0F);

  /* battery voltage level, in 1/10 volts */
  pk->data[2] = (voltage > 0xFF) ? 0xFF : voltage;

  /* status flags */
  pk->data[3] = (
    /* is the battery charging? */
    (pmIsCharging() ? 1 : 0) |
    /* is the high level commander being used now? */
    (commanderGetActivePriority() == COMMANDER_PRIORITY_HIGHLEVEL ? (1 << 1) : 0) |
    /* is the drone show mode turned on? */
    (droneShowIsEnabled() ? (1 << 2) : 0) |
    /* is the drone flying? */
    (supervisorIsFlying() ? (1 << 3) : 0) |
    /* is the drone show in testing mode? */
    (droneShowIsInTestingMode() ? (1 << 4) : 0) |
    /* is the drone _disarmed_? (backwards compatibility) */
    (systemIsArmed() ? 0 : (1 << 5)) |
    /* is the fence breached? */
    (fenceIsBreached() ? (1 << 6) : 0)
  );

  /* preflight check status */
  *((preflight_check_status_t*)(pk->data + 4)) = getPreflightCheckStatus();

  /* position */
  if (droneShowIsProbablyAirborne() && droneShowIsInTestingMode()) {
    /* in testing mode, report the position where the drone is supposed to be
     * instead of the current position, for debugging purposes */
    if (setpointXVarId == 0xffffu) {
      setpointXVarId = logGetVarId("ctrltarget", "x");
      setpointYVarId = logGetVarId("ctrltarget", "y");
      setpointZVarId = logGetVarId("ctrltarget", "z");
    }

    pos.x = logGetFloat(setpointXVarId);
    pos.y = logGetFloat(setpointYVarId);
    pos.z = logGetFloat(setpointZVarId);
  } else {
    estimatorKalmanGetEstimatedPos(&pos);
  }
  *((int16_t*)(pk->data + 6)) = (int16_t) (pos.x * 1000);
  *((int16_t*)(pk->data + 8)) = (int16_t) (pos.y * 1000);
  *((int16_t*)(pk->data + 10)) = (int16_t) (pos.z * 1000);

  /* yaw */
  *((int16_t*)(pk->data + 12)) = (int16_t) (logGetFloat(logIds.stateEstimateYaw) * 10);

  /* color of the LED ring, encoded in RGB565 */
  lastColor = droneShowGetLastColor();
  colorAsRGB565 = (
    (((lastColor[0] >> 3) & 0x1f) << 11) |
    (((lastColor[1] >> 2) & 0x3f) << 5) |
    ((lastColor[2] >> 3) & 0x1f)
  );
  *((uint16_t*)(pk->data + 14)) = colorAsRGB565;
}

PARAM_GROUP_START(show_crtp)
PARAM_ADD(PARAM_UINT8, payload_id, &payload_id)
PARAM_GROUP_STOP(show_crtp)

LOG_GROUP_START(load_pose)
LOG_ADD(LOG_FLOAT, x, &load_pose.x)
LOG_ADD(LOG_FLOAT, y, &load_pose.y)
LOG_ADD(LOG_FLOAT, z, &load_pose.z)
LOG_ADD(LOG_FLOAT, qx, &load_pose.quat.x)
LOG_ADD(LOG_FLOAT, qy, &load_pose.quat.y)
LOG_ADD(LOG_FLOAT, qz, &load_pose.quat.z)
LOG_ADD(LOG_FLOAT, qw, &load_pose.quat.w)
LOG_ADD(LOG_UINT32, counter, &load_pose_ctr)
LOG_GROUP_STOP(load_pose)