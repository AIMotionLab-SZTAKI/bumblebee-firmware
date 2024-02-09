/*
LQR controller.
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "stabilizer.h"
#include "physicalConstants.h"
#include "controller_lqr.h"
#include "pm.h"
#include "debug.h"
#include "controller.h"
#include "stdlib.h"


// Logging variables

static float cmd_thrust_N;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

static float drone_mass = 0.605;

static float K[4][12];

static uint32_t K_timestamp;
static uint32_t traj_timestamp;
static int32_t delay;
static int32_t max_delay = 200;
static uint32_t delay_ctr = 0;
static uint32_t max_delay_time_ms = 200;

static float ex, ey, ez; // evx, evy, evz, eyaw;

// struct vec setpoint_rpy;

void setLqrParams(float params[], int param_num, uint16_t timestamp) {
  for (int i=0; i<4; i++) {
    for (int j=0; j<12; j++) {
      int cur_idx = 12*i+j;
      if (cur_idx < param_num) {
        K[i][j] = params[12*i+j];
      }
    }
  }
  K_timestamp = timestamp;
}

void controllerLqrReset(void)
{
  for (int i=0; i<4; i++) {
    for (int j=0; j<12; j++) {
      K[i][j] = 0.0f;
    }
  }
  K[0][2] = 4.3726;
  K[0][5] = 2.7058;
  K[1][1] = -0.2917;
  K[1][4] = -0.2222;
  K[1][6] = 0.6766;
  K[1][9] = 0.1055;
  K[2][0] = 0.287;
  K[2][3] = 0.2186;
  K[2][7] = 0.6653;
  K[2][10] = 0.1037;
  K[3][8] = 0.0372;
  K[3][11] = 0.1188;

  delay = 0;
  delay_ctr=0;
  K_timestamp = 0;
  traj_timestamp = 0;
}

void controllerLqrInit(void)
{
  controllerLqrReset();
}

bool controllerLqrTest(void)
{
  return true;
}

void controllerLqr(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
      return;
    }

  //Enter force-torque control
  control->controlMode = controlModeForceTorque;

  // float vehicleWeight_N = drone_mass * GRAVITY_MAGNITUDE; //in N
 
  float setpoint_arr[12] = {setpoint->position.x, setpoint->position.y, setpoint->position.z, setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z,
                            radians(setpoint->attitude.roll), radians(setpoint->attitude.pitch), radians(setpoint->attitude.yaw), 
                            radians(setpoint->attitudeRate.roll), radians(setpoint->attitudeRate.pitch), radians(setpoint->attitudeRate.yaw)};

  traj_timestamp = setpoint->t_traj;
  delay = (int32_t)traj_timestamp - (int32_t)K_timestamp;  
  if (setpoint->mode.z == modeDisable) {
    delay = 0;
  }
  uint32_t delay_ctr_max = ATTITUDE_RATE * max_delay_time_ms / 1000;
  if (abs(delay) > max_delay) {
    delay_ctr++;
    if (delay_ctr > delay_ctr_max) {
      forceControllerType(ControllerTypeGeom);
    }
  } else {
    delay_ctr=0;
  }
  float wx = radians(sensors->gyro.x);
  float wy = radians(sensors->gyro.y);
  float wz = radians(sensors->gyro.z);
  float state_arr[12] = {state->position.x, state->position.y, state->position.z, state->velocity.x, state->velocity.y, state->velocity.z,
                         radians(state->attitude.roll), -radians(state->attitude.pitch), radians(state->attitude.yaw), wx, wy, wz};

  float err_arr[12];
  for (int i=0; i < 12; i++) {
    err_arr[i] = state_arr[i] - setpoint_arr[i];
  }

  ex = err_arr[0];
  ey = err_arr[1];
  ez = err_arr[2];

  float eyaw = err_arr[8];
  while (eyaw > M_PI_F) {
    eyaw -= 2 * M_PI_F;
  }
  while (eyaw < -M_PI_F) {
    eyaw += 2 * M_PI_F;
  }
  err_arr[8] = eyaw;

  // Compute control inputs: u = -K * (x - x_r) + u_r
  cmd_thrust_N = setpoint->thrust;
  cmd_roll = setpoint->torques.x;
  cmd_pitch = setpoint->torques.y;
  cmd_yaw = setpoint->torques.z;

  for (int i=0; i < 12; i++) {
    cmd_thrust_N += - K[0][i] * err_arr[i];
    cmd_roll += - K[1][i] * err_arr[i];
    cmd_pitch += - K[2][i] * err_arr[i];
    cmd_yaw += - K[3][i] * err_arr[i];
  }

  if (cmd_thrust_N > 8.0f) {
    cmd_thrust_N = 8.0f;
  }
  control->thrustSi = cmd_thrust_N;
  if (setpoint->mode.z == modeDisable) {
    control->thrustSi = 0;
  }
  if(control->thrustSi > 0){
    control->torqueX = cmd_roll;
    control->torqueY = cmd_pitch;
    control->torqueZ = cmd_yaw;
  } else {
    control->torqueX = 0;
    control->torqueY = 0;
    control->torqueZ = 0;
  }
}


PARAM_GROUP_START(Lqr)
PARAM_ADD(PARAM_FLOAT, drone_mass, &drone_mass)
PARAM_ADD(PARAM_INT32, max_delay, &max_delay)
PARAM_ADD(PARAM_UINT32, max_delay_time_ms, &max_delay_time_ms)
PARAM_GROUP_STOP(Lqr)


LOG_GROUP_START(Lqr)
LOG_ADD(LOG_FLOAT, cmd_thrust_N, &cmd_thrust_N)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_UINT32, traj_timestamp, &traj_timestamp)
//LOG_ADD(LOG_FLOAT, K_0_0, &K[0][0])
//LOG_ADD(LOG_FLOAT, K_3_11, &K[3][11])
LOG_ADD(LOG_UINT32, K_timestamp, &K_timestamp)
LOG_ADD(LOG_INT32, delay, &delay)
LOG_ADD(LOG_UINT32, delay_ctr, &delay_ctr)
LOG_ADD(LOG_FLOAT, ex, &ex)
// LOG_ADD(LOG_FLOAT, evx, &evx)
LOG_ADD(LOG_FLOAT, ey, &ey)
// LOG_ADD(LOG_FLOAT, evy, &evy)
LOG_ADD(LOG_FLOAT, ez, &ez)
// LOG_ADD(LOG_FLOAT, evz, &evz)
// LOG_ADD(LOG_FLOAT, eyaw, &eyaw)
// LOG_ADD(LOG_FLOAT, sroll, &setpoint->att.x)
// LOG_ADD(LOG_FLOAT, spitch, &setpoint_rpy.y)
// LOG_ADD(LOG_FLOAT, syaw, &setpoint_rpy.z)
LOG_GROUP_STOP(Lqr)
