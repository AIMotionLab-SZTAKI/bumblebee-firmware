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


// Logging variables

static float cmd_thrust_N;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static uint8_t setPointMode_x;
static uint8_t setPointMode_y;
static uint8_t setPointMode_z;
static uint8_t setPointMode_roll;
static uint8_t setPointMode_pitch;
static uint8_t setPointMode_yaw;
static uint8_t setPointMode_quat;
static uint8_t ctrlMode;


static struct quat q;

static float drone_mass = 0.605;
static float payload_mass = 0.05;

static float real_mass = 0.605;

static float measured_mass = 0;

static float batt_comp_a = -0.1205;  // with kR = 0.6: -0.1205
static float batt_comp_b = 2.6802;  // with kR = 0.6: 2.6802

static float K13 = 5.8055;
static float K16 = 4.8982;
static float K22 = -0.0991;
static float K25 = -0.1213;
static float K27 = 0.5053;
static float K210 = 0.0799;
static float K213 = -0.0199;
static float K215 = 0.0106;
static float K31 = 0.0958;
static float K34 = 0.1173;
static float K38 = 0.4885;
static float K311 = 0.0773;
static float K314 = -0.0193;
static float K316 = 0.0102;
static float K49 = 0.2906;
static float K412 = 0.1358;


static float dt;
static float ex, ey, ez, evx, evy, evz, alpha, dalpha, beta, dbeta;
// static float alpha2, dalpha2, beta2, dbeta2;

static poseMeasurement_t load_pose;

static struct vec load_vel;
static struct vec load_rpy;
static struct vec load_ang_vel;

static float average_weight_pos = 0.5;
static float average_weight_att = 0.4;

//static float L1 = 0.1; // distance from drone markers to hook joint
//static float L2 = 0.365; // distance from hook joint to hook markers

static const state_t* cur_state;

void setLoadState2Dof(const poseMeasurement_t *measurement, uint32_t dt_ms)
{
  dt = (float)dt_ms / 1000.0f;
  load_pose.quat = measurement->quat;
  q.x = load_pose.quat.q0;
  q.y = load_pose.quat.q1;
  q.z = load_pose.quat.q2;
  q.w = load_pose.quat.q3;
  struct vec load_rpy_new = quat2rpy(q);
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
  //float alpha2_prev = alpha2;
  //float beta2_prev = beta2;
  //alpha2 = -asinf((cur_state->position.y - load_pose.y + L1*sinf(radians(cur_state->attitude.roll)))/L2);
  //beta2 = -asinf((load_pose.x - cur_state->position.x + L1*cosf(radians(cur_state->attitude.roll))*sinf(-radians(cur_state->attitude.pitch)))/(L2*cosf(alpha2)));
  //dalpha2 = (1 - average_weight_pos) * dalpha2 + average_weight_pos * (alpha2 - alpha2_prev) / dt;
  //dbeta2 = (1 - average_weight_pos) * dbeta2 + average_weight_pos * (beta2 - beta2_prev) / dt;
}

void controllerLqr2DofReset(void)
{
  //No integral part to reset
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

  cur_state = state;

  if (payload_mass < 0.0f) {
    payload_mass = 0.0f;
  } else if (payload_mass > 0.1f) {
    payload_mass = 0.1f;
  }

  //Enter force-torque control
  control->controlMode = controlModeForceTorque;
  //Log variable
  ctrlMode = control->controlMode;
  //After this, we ought to work only with SI units, as force-torque control takes SI inputs
  float supplyVoltage = pmGetBatteryVoltage();  
  float mass_ratio = batt_comp_a * supplyVoltage + batt_comp_b;  
  float g_vehicleMass = real_mass * mass_ratio;
  float vehicleWeight_N = g_vehicleMass * GRAVITY_MAGNITUDE; //in N

  //Log variables
  setPointMode_x = setpoint->mode.x;
  setPointMode_y = setpoint->mode.y;
  setPointMode_z = setpoint->mode.z;
  setPointMode_roll = setpoint->mode.roll;
  setPointMode_pitch = setpoint->mode.pitch;
  setPointMode_yaw = setpoint->mode.yaw;
  setPointMode_quat = setpoint->mode.quat;

 
  //Position in m, velocity in m/s
  struct vec setpointPos = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  struct vec setpointVel = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
  /*struct vec setpointPos = mkvec(x, y, z);
  struct vec setpointVel = vzero();*/

  // Compute error vector: payload position, swing angle
  ex = state->position.x - setpointPos.x;
  ey = state->position.y - setpointPos.y;
  ez = state->position.z - setpointPos.z;
  evx = state->velocity.x - setpointVel.x;
  evy = state->velocity.y - setpointVel.y;
  evz = state->velocity.z - setpointVel.z;
  // alpha = load_pitch - quad_pitch
  alpha = load_rpy.x;
  dalpha = load_ang_vel.x;
  beta = load_rpy.y;
  dbeta = load_ang_vel.y;

  

  //alpha2 = -asinf((state->position.y - load_pose.y + L1*sinf(radians(state->attitude.roll)))/L2);
  //dalpha2 = -(state->velocity.y - load_vel.y + L1*cosf(radians(state->attitude.roll))*sensors->gyro.x)/(L2*sqrtf(1.0f - (powf(state->velocity.y - load_vel.y + L1*sinf(radians(state->attitude.roll)), 2))/L2/L2));
  //beta2 = -asinf((load_pose.x - state->position.x + L1*cosf(radians(state->attitude.roll))*sinf(-radians(state->attitude.pitch)))/(L2*cosf(alpha2)));
  //dbeta2 = ((state->velocity.x - load_vel.x + L1*sinf(radians(state->attitude.roll))*sinf(-radians(state->attitude.pitch))*sensors->gyro.y - L1*cosf(radians(state->attitude.roll))*cosf(-radians(state->attitude.pitch))*sensors->gyro.y)/(L2*cosf(alpha2)) - 
  //(sinf(alpha2)*dalpha2*(load_pose.x - state->position.x + L1*cosf(radians(state->attitude.roll))*sinf(-radians(state->attitude.pitch))))/(L2*cosf(alpha2)*cosf(alpha)))/sqrtf(1.0f - powf(load_pose.x - state->position.x + L1*cosf(radians(state->attitude.roll))*sinf(-radians(state->attitude.pitch)), 2)/(L2*L2*cosf(alpha2)*cosf(alpha2)));

  // Compute control inputs: u = -K * (x - x_r) + u_r
  cmd_thrust_N = -K13 * ez - K16 * evz + vehicleWeight_N; 
  cmd_roll = -K22 * ey - K25 * evy - K27 * radians(state->attitude.roll) - K210 * radians(sensors->gyro.x) - K213 * alpha - K215 * dalpha;
  cmd_pitch = -K31 * ex - K34 * evx - K38 * (-radians(state->attitude.pitch)) - K311 * (radians(sensors->gyro.y)) - K314 * beta - K316 * dbeta;
  cmd_yaw = -K49 * radians(state->attitude.yaw) - K412 * radians(sensors->gyro.z);
  // TODO: thrust = cmd_thrust_N, M.y = cmd_pitch


  // measured mass
  measured_mass = cmd_thrust_N / (GRAVITY_MAGNITUDE) / mass_ratio;
  if (measured_mass > 0.68f){
    real_mass = drone_mass + payload_mass;
  }
  else if (measured_mass < 0.65f) {
    real_mass = drone_mass;
  }

  control->thrustSi = cmd_thrust_N;
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


PARAM_GROUP_START(Lqr2)
/*PARAM_ADD(PARAM_FLOAT, K13, &K13)
PARAM_ADD(PARAM_FLOAT, K16, &K14)
PARAM_ADD(PARAM_FLOAT, K22, &K21)
PARAM_ADD(PARAM_FLOAT, K25, &K22)
PARAM_ADD(PARAM_FLOAT, K27, &K25)
PARAM_ADD(PARAM_FLOAT, K210, &K26)
PARAM_ADD(PARAM_FLOAT, K213, &K27)
PARAM_ADD(PARAM_FLOAT, K28, &K28)*/
PARAM_ADD(PARAM_FLOAT, drone_mass, &drone_mass)
PARAM_ADD(PARAM_FLOAT, payload_mass, &payload_mass)
PARAM_ADD(PARAM_FLOAT, batt_comp_a, &batt_comp_a)
PARAM_ADD(PARAM_FLOAT, batt_comp_b, &batt_comp_b)
PARAM_ADD(PARAM_FLOAT, w_pos, &average_weight_pos)
PARAM_ADD(PARAM_FLOAT, w_att, &average_weight_att)
PARAM_GROUP_STOP(Lqr2)


LOG_GROUP_START(Lqr2)
LOG_ADD(LOG_FLOAT, cmd_thrust_N, &cmd_thrust_N)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, ex, &ex)
LOG_ADD(LOG_FLOAT, evx, &evx)
LOG_ADD(LOG_FLOAT, ey, &ey)
LOG_ADD(LOG_FLOAT, evy, &evy)
LOG_ADD(LOG_FLOAT, ez, &ez)
LOG_ADD(LOG_FLOAT, evz, &evz)
LOG_ADD(LOG_FLOAT, alpha, &alpha)
LOG_ADD(LOG_FLOAT, dalpha, &dalpha)
LOG_ADD(LOG_FLOAT, beta, &beta)
LOG_ADD(LOG_FLOAT, dbeta, &dbeta)
//LOG_ADD(LOG_FLOAT, beta2, &beta2)
//LOG_ADD(LOG_FLOAT, dbeta2, &dbeta2)
//LOG_ADD(LOG_FLOAT, alpha2, &alpha2)
//LOG_ADD(LOG_FLOAT, dalpha2, &dalpha2)
LOG_ADD(LOG_FLOAT, dt, &dt)
LOG_ADD(LOG_UINT8, x_Mode, &setPointMode_x)
LOG_ADD(LOG_UINT8, y_Mode, &setPointMode_y)
LOG_ADD(LOG_UINT8, z_Mode, &setPointMode_z)
LOG_ADD(LOG_UINT8, roll_Mode, &setPointMode_roll)
LOG_ADD(LOG_UINT8, pitch_Mode, &setPointMode_pitch)
LOG_ADD(LOG_UINT8, yaw_Mode, &setPointMode_yaw)
LOG_ADD(LOG_UINT8, quat_Mode, &setPointMode_quat)
LOG_ADD(LOG_UINT8, ctrlMode, &ctrlMode)
LOG_ADD(LOG_FLOAT, measured_mass, &measured_mass)
LOG_ADD(LOG_FLOAT, real_mass, &real_mass)
LOG_GROUP_STOP(Lqr2)
