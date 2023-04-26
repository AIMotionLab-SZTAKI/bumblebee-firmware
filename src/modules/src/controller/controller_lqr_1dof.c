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
static float cmd_pitch;
static float r_pitch;
static float thrust;
static uint8_t setPointMode_x;
static uint8_t setPointMode_y;
static uint8_t setPointMode_z;
static uint8_t setPointMode_roll;
static uint8_t setPointMode_pitch;
static uint8_t setPointMode_yaw;
static uint8_t setPointMode_quat;
static uint8_t ctrlMode;

static float x = 0;
static float y = 0;
static float z = 0;

static float ctrl_thrust = 0;

static struct quat q;

static float drone_mass = 0.605;
static float payload_mass = 0.0736;

static float real_mass = 0.605;

static float measured_mass = 0;

static float psi = 0;

static float batt_comp_a = -0.1205;  // with kR = 0.6: -0.1205
static float batt_comp_b = 2.6802;  // with kR = 0.6: 2.6802

static float K13 = 3.06;
static float K14 = 2.24;
static float K21 = 0.211;
static float K22 = 0.209;
static float K25 = 0.814;
static float K26 = 0.0825;
static float K27 = 0.0982;
static float K28 = 0.150;

static float dt;
static float ex, ez, evx, evz, alpha, dalpha;

static poseMeasurement_t load_pose;

static struct vec load_vel;
static struct vec load_rpy;
static struct vec load_ang_vel;

void setLoadState(const poseMeasurement_t *measurement, const uint32_t dt_ms)
{
  dt = (float)dt_ms / 1000.0f;
  struct vec load_rpy_new = quat2rpy(load_pose.quat);
  load_vel.x = (measurement->x - load_pose.x) / dt;
  load_vel.y = (measurement->y - load_pose.y) / dt;
  load_vel.z = (measurement->z - load_pose.z) / dt;
  load_ang_vel.x = (load_rpy_new.x - load_rpy.x) / dt;
  load_ang_vel.y = (load_rpy_new.y - load_rpy.y) / dt;
  load_ang_vel.z = (load_rpy_new.z - load_rpy.z) / dt;
  load_pose.x = measurement->x;
  load_pose.y = measurement->y;
  load_pose.z = measurement->z;
  load_pose.quat = measurement->quat;
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
  struct vec setpointPos = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z-0.4f);
  struct vec setpointVel = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
  /*struct vec setpointPos = mkvec(x, y, z);
  struct vec setpointVel = vzero();*/

  // Compute error vector: payload position, swing angle
  ex = load_pose.x - setpointPos.x;
  ez = load_pose.z - setpointPos.z;
  evx = load_vel.x - setpointVel.x;
  evz = load_vel.z - setpointVel.z;
  // alpha = load_pitch - quad_pitch
  alpha = load_rpy.y - (-radians(state->attitude.pitch));
  dalpha = load_ang_vel.y - (-radians(sensors->gyro.y));  // negative sign inside parenthesis due to CF coordinate system

  // Compute control inputs: u = -K * (x - x_r) + u_r
  cmd_thrust_N = -K13 * ez - K14 * evz + vehicleWeight_N; 
  cmd_pitch = -K21 * ex - K22 * evx - K25 * (-radians(state->attitude.pitch)) - K26 * (-radians(sensors->gyro.y)) - K27 * alpha - K28 * dalpha;
  // TODO: thrust = cmd_thrust_N, M.y = cmd_pitch


  // measured mass
  measured_mass = cmd_thrust_N / (GRAVITY_MAGNITUDE) / mass_ratio;
  if (measured_mass > 0.68f){
    real_mass = drone_mass + payload_mass;
  }
  else if (measured_mass < 0.65f) {
    real_mass = drone_mass;
  }
  
  //Thrust pointing in crazyflie body frame Z direction  
  // thrust = cmd_thrust_N;
  // control->thrustSi = thrust;

  r_pitch = -radians(sensors->gyro.y);

  /*if (control->thrustSi > 0) {
    control->torqueX = M.x;
    control->torqueY = -M.y;
    control->torqueZ = M.z;

  } else {
    control->torqueX = 0;
    control->torqueY = 0;
    control->torqueZ = 0;
  }
  //Log variables
  cmd_roll = control->torqueX;
  cmd_pitch = control->torqueY;
  cmd_yaw = control -> torqueZ;  
  */
}

PARAM_GROUP_START(ctrlLqr1Dof)
PARAM_ADD(PARAM_FLOAT, K13, &K13)
PARAM_ADD(PARAM_FLOAT, K14, &K14)
PARAM_ADD(PARAM_FLOAT, K21, &K21)
PARAM_ADD(PARAM_FLOAT, K22, &K22)
PARAM_ADD(PARAM_FLOAT, K25, &K25)
PARAM_ADD(PARAM_FLOAT, K26, &K26)
PARAM_ADD(PARAM_FLOAT, K27, &K27)
PARAM_ADD(PARAM_FLOAT, K28, &K28)
PARAM_ADD(PARAM_FLOAT, drone_mass, &drone_mass)
PARAM_ADD(PARAM_FLOAT, payload_mass, &payload_mass)
PARAM_ADD(PARAM_FLOAT, ctrl_thrust, &ctrl_thrust)
PARAM_ADD(PARAM_FLOAT, batt_comp_a, &batt_comp_a)
PARAM_ADD(PARAM_FLOAT, batt_comp_b, &batt_comp_b)
PARAM_ADD(PARAM_FLOAT, x, &x)
PARAM_ADD(PARAM_FLOAT, y, &y)
PARAM_ADD(PARAM_FLOAT, z, &z)
PARAM_GROUP_STOP(ctrlLqr1Dof)


LOG_GROUP_START(ctrlLqr1Dof)
LOG_ADD(LOG_FLOAT, cmd_thrust_N, &cmd_thrust_N)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, thrust, &thrust)
LOG_ADD(LOG_FLOAT, ex, &ex)
LOG_ADD(LOG_FLOAT, evx, &evx)
LOG_ADD(LOG_FLOAT, ez, &ez)
LOG_ADD(LOG_FLOAT, evz, &evz)
LOG_ADD(LOG_FLOAT, alpha, &alpha)
LOG_ADD(LOG_FLOAT, dalpha, &dalpha)
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
LOG_GROUP_STOP(ctrlLqr1Dof)
