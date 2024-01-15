/*
Smooth controller switching from LQR 1 DoF to Geometric 
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "stabilizer.h"
#include "physicalConstants.h"
#include "controller_geom.h"
#include "controller_lqr_1dof.h"
#include "pm.h"

static float progress = 0;  // Progress variable: 0 -> Lqr1Dof, 1-> Geom
static float switch_duration = 2.91;  // Switch duration in seconds
static float cmd_roll, cmd_pitch, cmd_yaw, cmd_thrust;


void controllerSwitchReset(void)
{
  progress = 0;
}

void controllerSwitchInit(void)
{
  progress = 0;
}

bool controllerSwitchTest(void)
{
  return true;
}

void controllerSwitch(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  float dt = 1.0f / (float)ATTITUDE_RATE;
  
  // Compute inputs of both controllers, and blend them together
  // DANGER: real_mass should be identical in both controllers in order to work properly
  controllerGeom(control, setpoint, sensors, state, tick);  // sets the control variable
  float geom_cmd_roll = control->torqueX;
  float geom_cmd_pitch = control->torqueY;
  float geom_cmd_yaw = control->torqueZ;
  float geom_cmd_thrust = control->thrustSi;
  
  controllerLqr1Dof(control, setpoint, sensors, state, tick);  // sets the control variable
  float lqr_cmd_roll = control->torqueX;
  float lqr_cmd_pitch = control->torqueY;
  float lqr_cmd_yaw = control->torqueZ;
  float lqr_cmd_thrust = control->thrustSi;

  // Blend control inputs
  cmd_roll = progress * geom_cmd_roll + (1.0f - progress) * lqr_cmd_roll;
  cmd_pitch = progress * geom_cmd_pitch + (1.0f - progress) * lqr_cmd_pitch;
  cmd_yaw = progress * geom_cmd_yaw + (1.0f - progress) * lqr_cmd_yaw;
  cmd_thrust = progress * geom_cmd_thrust + (1.0f - progress) * lqr_cmd_thrust;

  control->thrustSi = cmd_thrust;
  if (control->thrustSi > 0) {
    control->torqueX = cmd_roll;
    control->torqueY = cmd_pitch;
    control->torqueZ = cmd_yaw;

  } else {
    control->torqueX = 0;
    control->torqueY = 0;
    control->torqueZ = 0;
  }

  if (progress < 1.0f) {
    progress += 1.0f / switch_duration * dt;
  }
}

LOG_GROUP_START(ctrlSwitch)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, progress, &progress)
LOG_GROUP_STOP(ctrlSwitch)