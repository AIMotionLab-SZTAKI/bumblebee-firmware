/*
LQR payload stabilizing controller.
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "stabilizer.h"
#include "physicalConstants.h"
#include "controller_lqr_2dof.h"
#include "pm.h"
#include "stdlib.h"
#include "mem.h"

#define LQR_N 240 // 30kbyte=15360 uint16 param
#define INT16_LQR_SCALING 32766

static uint32_t duration = 0;
float K_lim_memory[128+1]; //1 = 4 bytes for checksum
int16_t K_memory[LQR_N*64+2]; //2 = 4 bytes for checksum

static uint32_t handleMemGetSize(void) {return sizeof(K_memory);}
static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
static bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);
static const MemoryHandlerDef_t memDef = {
  .type = MEM_TYPE_LQR,
  .getSize = handleMemGetSize,
  .read = handleMemRead,
  .write = handleMemWrite,
};

static uint32_t handleBoundsGetSize(void) {return sizeof(K_lim_memory);}
static bool handleBoundsRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
static bool handleBoundsWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);
static const MemoryHandlerDef_t memDefBounds = {
  .type = MEM_TYPE_LQR_BOUNDS,
  .getSize = handleBoundsGetSize,
  .read = handleBoundsRead,
  .write = handleBoundsWrite,
};

void lqr2RegisterMemoryHandler(void) {
  memoryRegisterHandler(&memDef);
  memoryRegisterHandler(&memDefBounds);
}

// Logging variables

static float cmd_thrust_N;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static uint8_t ctrlMode;


static struct quat q;

static float drone_mass = 0.625;
static float payload_mass = 0.05;
static float rod_length_safety = 5.0;
static float rod_length = 0.52;

// static float real_mass = 0.625;

// static float measured_mass = 0;

static float batt_comp_a = -0.1205;
static float batt_comp_b = 2.6802;

static float K[4][16];

static uint32_t K_timestamp;
static uint32_t traj_timestamp;
static int32_t delay;
static int32_t max_delay = 200;
static uint32_t delay_ctr = 0;
static uint32_t max_delay_time_ms = 200;

static float dt;
static float ex, ey, ez;
// evx, evy, evz, 
static float alpha, dalpha, beta, dbeta;
static float alpha_ref, beta_ref, dalpha_ref, dbeta_ref;

static poseMeasurement_t load_pose;

static struct vec load_vel;
static struct vec load_rpy;
static struct vec load_ang_vel;
static struct vec load_pos_error;

static float average_weight_pos = 0.5;
static float average_weight_att = 0.4;

static const state_t* cur_state;

static float K1, K2, K3, K4, K5, K6, K7, K8; // only for logging
static uint16_t hook_control_on;

void setLqr2DofParams(float params[], int param_num, uint16_t timestamp) {
  for (int i=0; i<4; i++) {
    for (int j=0; j<16; j++) {
      int cur_idx = 16*i+j;
      if (cur_idx < param_num) {
        K[i][j] = params[16*i+j];
      }
    }
  }
  K_timestamp = timestamp;
}

void setLoadState2Dof(const poseMeasurement_t *measurement, uint32_t dt_ms)
{
  dt = (float)dt_ms / 1000.0f;
  load_pose.quat = measurement->quat;
  q.x = load_pose.quat.q0;
  q.y = load_pose.quat.q1;
  q.z = load_pose.quat.q2;
  q.w = load_pose.quat.q3;
  struct vec load_rpy_body = quat2rpy(q);
  float yaw = load_rpy_body.z;
  struct vec load_rpy_new = quat2rpy(qqmul(rpy2quat(mkvec(0, 0, -yaw)), q));
  load_vel.x = (1 - average_weight_pos) * load_vel.x + average_weight_pos * (measurement->x - load_pose.x) / dt;
  load_vel.y = (1 - average_weight_pos) * load_vel.y + average_weight_pos * (measurement->y - load_pose.y) / dt;
  load_vel.z = (1 - average_weight_pos) * load_vel.z + average_weight_pos * (measurement->z - load_pose.z) / dt;
  load_ang_vel.x = (1 - average_weight_att) * load_ang_vel.x + average_weight_att * (load_rpy_new.x - load_rpy.x) / dt;
  load_ang_vel.y = (1 - average_weight_att) * load_ang_vel.y + average_weight_att * (load_rpy_new.y - load_rpy.y) / dt;
  load_ang_vel.z = (1 - average_weight_att) * load_ang_vel.z + average_weight_att * (load_rpy_new.z - load_rpy.z) / dt;
  load_pose.x = measurement->x;
  load_pose.y = measurement->y;
  load_pose.z = measurement->z;
  load_rpy = load_rpy_new;
}

void controllerLqr2DofReset(void)
{
  for (int i=0; i<4; i++) {
    for (int j=0; j<16; j++) {
      K[i][j] = 0.0f;
    }
  }
  K[0][2] = 4.3746;
  K[0][5] = 2.739;
  K[1][1] = -0.2899;
  K[1][4] = -0.2343;
  K[1][6] = 0.8058;
  K[1][9] = 0.1073;
  K[1][12] = -0.0302;
  K[1][14] = 0.024;
  K[2][0] = 0.2853;
  K[2][3] = 0.2305;
  K[2][7] = 0.7922;
  K[2][10] = 0.1054;
  K[2][13] = -0.0296;
  K[2][15] = 0.0236;
  K[3][8] = 0.0372;
  K[3][11] = 0.1188;

  delay_ctr=0;
  K_timestamp = 0;
  traj_timestamp = 0;
}

void controllerLqr2DofInit(void)
{
  controllerLqr2DofReset();
}

bool controllerLqr2DofTest(void)
{
  return true;
}

void controllerLqr2Dof(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
      return;
    }

  traj_timestamp = setpoint->t_traj;
  // Compute the current LQR gain matrix by linear interpolation between adjacent values,
  // Then uncompress the compressed matrix using the bounds in K_lim_memory
  float param_progress = (float)traj_timestamp / (float)duration * LQR_N;
  int16_t ta = (int16_t)param_progress * 64;
  int16_t tb = ta + 64;
  for (int i=0; i<4; i++) {
    for (int j=0; j<16; j++) {
      float wa = (param_progress - (float)ta / 64.0f) / LQR_N;
      float K_elem_normed =  wa * (float)K_memory[2 + ta + 16*i + j] + (1-wa) * (float)K_memory[2 + tb + 16*i + j]; //2+... because of crc
      float K_lb = K_lim_memory[1+16*i+j]; //1+... because of crc
      float K_ub = K_lim_memory[1+64 + 16*i+j]; //1+... because of crc
      K[i][j] = (K_ub - K_lb) / 2 * K_elem_normed / (float)INT16_LQR_SCALING + (K_ub + K_lb) / 2;
    }
  }

  delay = (int32_t)traj_timestamp - (int32_t)K_timestamp;  
  if (setpoint->mode.z == modeDisable) {
    delay = 0;
  }
  uint32_t delay_ctr_max = ATTITUDE_RATE * max_delay_time_ms / 1000;
  if (abs(delay) > max_delay) {
    delay_ctr++;
    if (delay_ctr > delay_ctr_max && !duration) {
      forceControllerType(ControllerTypeGeom);
    }
  } else {
    delay_ctr=0;
  }


  cur_state = state;

  if (payload_mass < 0.0f) {
    payload_mass = 0.0f;
  } else if (payload_mass > 0.1f) {
    payload_mass = 0.1f;
  }

  // Only for logging
  K1 = K[0][2];
  K2 = K[0][5];
  K3 = K[1][1];
  K4 = K[1][12];
  K5 = K[1][14];
  K6 = K[1][9];
  K7 = K[2][13];
  K8 = K[2][15];

  //Enter force-torque control
  control->controlMode = controlModeForceTorque;
  //Log variable
  ctrlMode = control->controlMode;
  //After this, we ought to work only with SI units, as force-torque control takes SI inputs
  
  float supplyVoltage = pmGetBatteryVoltage();  
  float mass_ratio = batt_comp_a * supplyVoltage + batt_comp_b;  
  //float g_vehicleMass = real_mass * mass_ratio;
  //float vehicleWeight_N = g_vehicleMass * GRAVITY_MAGNITUDE; //in N
  

  float setpoint_arr[16] = {setpoint->position.x, setpoint->position.y, setpoint->position.z, setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z,
                            radians(setpoint->attitude.roll), radians(setpoint->attitude.pitch), radians(setpoint->attitude.yaw), 
                            radians(setpoint->attitudeRate.roll), radians(setpoint->attitudeRate.pitch), radians(setpoint->attitudeRate.yaw), 
                            setpoint->alpha, setpoint->beta, setpoint->dalpha, setpoint->dbeta};
  alpha_ref = setpoint->alpha;
  beta_ref = setpoint->beta;
  dalpha_ref = setpoint->dalpha;
  dbeta_ref = setpoint->dbeta;

  struct vec load_pos_ref = vsub(mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z), mvmul(quat2rotmat(rpy2quat(mkvec(alpha_ref, beta_ref, 0))), mkvec(0, 0, rod_length)));
  load_pos_error = vsub(mkvec(load_pose.x, load_pose.y, load_pose.z), load_pos_ref);

  float wx = radians(sensors->gyro.x);
  float wy = radians(sensors->gyro.y);
  float wz = radians(sensors->gyro.z);
  alpha = load_rpy.x;
  dalpha = load_ang_vel.x;
  beta = load_rpy.y;
  dbeta = load_ang_vel.y;
  float state_arr[16] = {state->position.x, state->position.y, state->position.z, state->velocity.x, state->velocity.y, state->velocity.z,
                         radians(state->attitude.roll), -radians(state->attitude.pitch), radians(state->attitude.yaw), wx, wy, wz,
                         alpha, beta, dalpha, dbeta};

  float err_arr[16];
  for (int i=0; i < 16; i++) {
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

  /*
  // measured mass
  measured_mass = cmd_thrust_N / (GRAVITY_MAGNITUDE) / mass_ratio;
  if (measured_mass > 0.7f){
    real_mass = drone_mass + payload_mass;
  }
  else if (measured_mass < 0.67f) {
    real_mass = drone_mass;
  }
  */

  // Compute control inputs: u = -K * (x - x_r) + u_r
  cmd_thrust_N = setpoint->thrust * mass_ratio;
  cmd_roll = setpoint->torques.x;
  cmd_pitch = setpoint->torques.y;
  cmd_yaw = setpoint->torques.z;

  int num_states_to_control = 16;
  hook_control_on = 1;
  if (state->position.z < rod_length_safety) {
    num_states_to_control = 12;
    hook_control_on = 0;
  }
  for (int i=0; i < num_states_to_control; i++) {
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

static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
  bool result = false;
  if (memAddr + readLen <= sizeof(K_memory)) {
    uint8_t* K_memory_uint8 = (uint8_t*)(K_memory);
    memcpy(buffer, K_memory_uint8+memAddr, readLen);
    result = true;
  }

  return result;
}

static bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer) {
  bool result = false;
  if ((memAddr + writeLen) <= sizeof(K_memory)) {
    uint8_t* K_memory_uint8 = (uint8_t*)(K_memory);
    memcpy(K_memory_uint8+memAddr, buffer, writeLen);
    result = true;
  }
  return result;
}

static bool handleBoundsRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
  bool result = false;
  if (memAddr + readLen <= sizeof(K_lim_memory)) {
    uint8_t* K_lim_memory_uint8 = (uint8_t*)K_lim_memory;
    memcpy(buffer, K_lim_memory_uint8+memAddr, readLen);
    result = true;
  }

  return result;
}

static bool handleBoundsWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer) {
  bool result = false;
  if ((memAddr + writeLen) <= sizeof(K_lim_memory)) {
    uint8_t* K_lim_memory_uint8 = (uint8_t*)K_lim_memory;
    memcpy(K_lim_memory_uint8 + memAddr, buffer, writeLen);
    result = true;
  }
  return result;
}

PARAM_GROUP_START(Lqr2)
PARAM_ADD(PARAM_FLOAT, drone_mass, &drone_mass)
PARAM_ADD(PARAM_FLOAT, payload_mass, &payload_mass)
PARAM_ADD(PARAM_INT32, max_delay, &max_delay)
PARAM_ADD(PARAM_UINT32, max_delay_time_ms, &max_delay_time_ms)
PARAM_ADD(PARAM_FLOAT, batt_comp_a, &batt_comp_a)
PARAM_ADD(PARAM_FLOAT, batt_comp_b, &batt_comp_b)
PARAM_ADD(PARAM_FLOAT, w_pos, &average_weight_pos)
PARAM_ADD(PARAM_FLOAT, w_att, &average_weight_att)
PARAM_ADD(PARAM_FLOAT, rod_length_safety, &rod_length_safety)
PARAM_ADD(PARAM_FLOAT, rod_length, &rod_length)
PARAM_ADD(PARAM_UINT32, duration, &duration)
PARAM_GROUP_STOP(Lqr2)


LOG_GROUP_START(Lqr2)
LOG_ADD(LOG_UINT32, traj_timestamp, &traj_timestamp)
LOG_ADD(LOG_UINT32, K_timestamp, &K_timestamp)
LOG_ADD(LOG_INT32, delay, &delay)
LOG_ADD(LOG_UINT32, delay_ctr, &delay_ctr)
LOG_ADD(LOG_FLOAT, cmd_thrust_N, &cmd_thrust_N)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, ex, &ex)
// LOG_ADD(LOG_FLOAT, evx, &evx)
LOG_ADD(LOG_FLOAT, ey, &ey)
// LOG_ADD(LOG_FLOAT, evy, &evy)
LOG_ADD(LOG_FLOAT, ez, &ez)
// LOG_ADD(LOG_FLOAT, evz, &evz)
LOG_ADD(LOG_FLOAT, ex_load, &load_pos_error.x)
LOG_ADD(LOG_FLOAT, ey_load, &load_pos_error.y)
LOG_ADD(LOG_FLOAT, ez_load, &load_pos_error.z)
LOG_ADD(LOG_FLOAT, alpha, &alpha)
LOG_ADD(LOG_FLOAT, dalpha, &dalpha)
LOG_ADD(LOG_FLOAT, beta, &beta)
LOG_ADD(LOG_FLOAT, dbeta, &dbeta)
LOG_ADD(LOG_FLOAT, alpha_ref, &alpha_ref)
LOG_ADD(LOG_FLOAT, dalpha_ref, &dalpha_ref)
LOG_ADD(LOG_FLOAT, beta_ref, &beta_ref)
LOG_ADD(LOG_FLOAT, dbeta_ref, &dbeta_ref)
LOG_ADD(LOG_FLOAT, dt, &dt)
LOG_ADD(LOG_UINT8, ctrlMode, &ctrlMode)
// LOG_ADD(LOG_FLOAT, measured_mass, &measured_mass)
// LOG_ADD(LOG_FLOAT, real_mass, &real_mass)
LOG_ADD(LOG_FLOAT, K1, &K1)
LOG_ADD(LOG_FLOAT, K2, &K2)
LOG_ADD(LOG_FLOAT, K3, &K3)
LOG_ADD(LOG_FLOAT, K4, &K4)
LOG_ADD(LOG_FLOAT, K5, &K5)
LOG_ADD(LOG_FLOAT, K6, &K6)
LOG_ADD(LOG_FLOAT, K7, &K7)
LOG_ADD(LOG_FLOAT, K8, &K8)
LOG_ADD(LOG_UINT16, hook_control_on, &hook_control_on)
LOG_GROUP_STOP(Lqr2)
