/*
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

#define DEBUG_MODULE "APP"

#include <stdbool.h>

#include "autoconf.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app.h"
#include "debug.h"
#include "system.h"
#include "static_mem.h"

#include "arming.h"
#include "crtp_drone_show_service.h"
#include "custom_params.h"
#include "drone_show.h"
#include "fence.h"
#include "gcs_light_effects.h"
#include "light_program.h"
#include "preflight.h"
#include "controller_lqr_2dof.h"

#define DRONE_SHOW_APP_STACKSIZE 300

static bool isInit = false;

STATIC_MEM_TASK_ALLOC(appTask, DRONE_SHOW_APP_STACKSIZE);

static bool appTest();
static void appTask(void *param);

void appInit()
{
  if (isInit) {
    return;
  }

  // The light program player has to be initialized here, before the system
  // starts because it registers a new memory handler
  lightProgramPlayerInit();

  // The fence module also has to be initialized here for the same reasons
  fenceInit();

  // The lqr controller also has to be initialized here for the same reasons
  lqr2RegisterMemoryHandler();

  STATIC_MEM_TASK_CREATE(appTask, appTask, "app", NULL, CONFIG_APP_PRIORITY);

  isInit = true;
}

static bool appTest()
{
  bool pass = isInit;

  pass &= armingTest();
  pass &= fenceTest();
  pass &= preflightTest();
  pass &= gcsLightEffectsTest();
  pass &= lightProgramPlayerTest();
  pass &= droneShowSrvTest();
  pass &= droneShowTest();

  pass &= droneShowApplyCustomParameters();

  return pass;
}

static void appTask(void *param)
{
  systemWaitStart();

  armingInit();
  preflightInit();
  gcsLightEffectsInit();
  droneShowSrvInit();
  droneShowInit();

  if (appTest()) {
    appMain();
  } else {
    DEBUG_PRINT("Failure during drone show app startup.\n");
  }

  while(1) {
    vTaskDelay(portMAX_DELAY);
  }
}

void appMain() {
  DEBUG_PRINT("Drone show app started.\n");
}
