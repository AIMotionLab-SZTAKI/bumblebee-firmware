/*
LQR payload stabilizing controller.
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "stabilizer.h"
#include "physicalConstants.h"
#include "controller_lqr_1dof.h"
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

static float drone_mass = 0.65;
static float payload_mass = 0.0736;

static float real_mass = 0.65;

static float measured_mass = 0;

static float batt_comp_a = -0.1245; 
static float batt_comp_b = 2.768; 

static float K13 = 6;
static float K16 = 3;
static float K22 = -0.5;
static float K25 = -0.3491;
static float K27 = 1.1709;
static float K210 = 0.204;
static float K31 = 0.5;
static float K34 = 0.3477;
static float K38 = 1.1616;
static float K311 = 0.1962;
static float K313 = -0.0937;
static float K314 = 0.0229;
static float K49 = 0.5;
static float K412 = 0.0539;


static float dt;
static float ex, ey, ez, evx, evy, evz, alpha, dalpha;

static poseMeasurement_t load_pose;

static struct vec load_vel;
static struct vec load_rpy;
static struct vec load_ang_vel;

static float average_weight_pos = 0.5;
static float average_weight_att = 0.4;

static const state_t* cur_state;

void setLoadState1Dof(const poseMeasurement_t *measurement, uint32_t dt_ms)
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
}

void controllerLqr1DofReset(void)
{
  //No integral part to reset
}

void controllerLqr1DofInit(void)
{
  controllerLqr1DofReset();
}

bool controllerLqr1DofTest(void)
{
  return true;
}

void controllerLqr1Dof(control_t *control, const setpoint_t *setpoint,
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
  } else if (payload_mass > 0.15f) {
    payload_mass = 0.15f;
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

  // Angle setpoints are not calculated in pptraj.c
  struct vec zB = vnormalize(mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z + GRAVITY_MAGNITUDE));
  struct vec xC = mkvec(cosf(radians(setpoint->attitude.yaw)), sinf(radians(setpoint->attitude.yaw)), 0);
  struct vec yB = vnormalize(vcross(zB, xC));
  struct vec xB = vcross(yB, zB);
  struct mat33 Rd = mcolumns(xB, yB, zB);
  struct vec setpoint_rpy = quat2rpy(mat2quat(Rd));

  // Compute error vector: drone position, swing angle
  ex = state->position.x - setpointPos.x;
  ey = state->position.y - setpointPos.y;
  ez = state->position.z - setpointPos.z;

  evx = state->velocity.x - setpointVel.x;
  evy = state->velocity.y - setpointVel.y;
  evz = state->velocity.z - setpointVel.z;

  float yaw = radians(state->attitude.yaw);
  float ex_tr = cosf(yaw) * ex + sinf(yaw) * ey;
  float ey_tr = -sinf(yaw) * ex + cosf(yaw) * ey;
  float evx_tr = cosf(yaw) * evx + sinf(yaw) * evy;
  float evy_tr = -sinf(yaw) * evx + cosf(yaw) * evy;

  alpha = load_rpy.y;
  dalpha = load_ang_vel.y;

  float eroll = radians(state->attitude.roll) ;//- setpoint_rpy.x;
  float epitch = -radians(state->attitude.pitch);// - setpoint_rpy.y;
  float eyaw = radians(state->attitude.yaw) - setpoint_rpy.z;
  while (eyaw > M_PI_F) {
    eyaw -= 2 * M_PI_F;
  }
  while (eyaw < -M_PI_F) {
    eyaw += 2 * M_PI_F;
  }

  // Angular velocity setpoints are calculated in pptraj.c
  float ewx = radians(sensors->gyro.x); //- setpoint->attitudeRate.roll;
  float ewy = radians(sensors->gyro.y); //- setpoint->attitudeRate.pitch;
  float ewz = radians(sensors->gyro.z) - radians(setpoint->attitudeRate.yaw);

  // Compute control inputs: u = -K * (x - x_r) + u_r
  cmd_thrust_N = -K13 * ez - K16 * evz + vehicleWeight_N; 
  cmd_roll = -K22 * ey_tr - K25 * evy_tr - K27 * eroll - K210 * ewx;
  cmd_pitch = -K31 * ex_tr - K34 * evx_tr - K38 * epitch - K311 * ewy - K313 * alpha - K314 * dalpha;
  cmd_yaw = -K49 * eyaw - K412 * ewz;

  // measured mass
  measured_mass = cmd_thrust_N / (GRAVITY_MAGNITUDE) / mass_ratio;
  if (measured_mass > 0.7f){
    real_mass = drone_mass + payload_mass;
  }
  else if (measured_mass < 0.67f) {
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


PARAM_GROUP_START(Lqr1)
PARAM_ADD(PARAM_FLOAT, drone_mass, &drone_mass)
PARAM_ADD(PARAM_FLOAT, payload_mass, &payload_mass)
PARAM_ADD(PARAM_FLOAT, batt_comp_a, &batt_comp_a)
PARAM_ADD(PARAM_FLOAT, batt_comp_b, &batt_comp_b)
PARAM_ADD(PARAM_FLOAT, w_pos, &average_weight_pos)
PARAM_ADD(PARAM_FLOAT, w_att, &average_weight_att)
PARAM_GROUP_STOP(Lqr1)


LOG_GROUP_START(Lqr1)
LOG_ADD(LOG_FLOAT, cmd_thrust_N, &cmd_thrust_N)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, ex, &ex)
LOG_ADD(LOG_FLOAT, evx, &evx)
LOG_ADD(LOG_FLOAT, ey, &ey)
LOG_ADD(LOG_FLOAT, evy, &evy)
LOG_ADD(LOG_FLOAT, ez, &ez)
LOG_ADD(LOG_FLOAT, evz, &evz)
LOG_ADD(LOG_FLOAT, alpha, &load_rpy.y)
LOG_ADD(LOG_FLOAT, dalpha, &load_ang_vel.y)
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
LOG_GROUP_STOP(Lqr1)
