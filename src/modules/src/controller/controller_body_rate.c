/**
 * controller_body_rate.c - Body rate P controller for an off-board MPC
 *
 * Input (mpc_input.c): thrust and body rate commands from the CRTP (Skybrush server) or UART link. The rate loop is
 * M = -kw (Omega - Omega_ref) per axis, mixed by the force/torque power distribution.
 *
 * Type 21 commands carry the rate at the time the command takes effect (for the INDI). This P loop needs a lead
 * instead, as type 20 provides (the MPC's predicted rate one node ahead), so it extrapolates Omega_ref + alpha_ff *
 * t21_lead. Without it, type 21 flights diverge (SIL). Default: NMPC node (50 ms) minus command latency (10 ms).
 */
#include "stabilizer_types.h"

#include "controller_body_rate.h"
#include "mpc_input.h"

#include "log.h"
#include "math3d.h"
#include "param.h"
#include "supervisor.h"

static float kw = 0.15f;       // Nm s/rad, roll and pitch
static float kw_yaw = 0.15f;   // Nm s/rad, yaw
static float t21_lead = 0.04f; // s

static struct vec wd, M;       // rate setpoint in use [rad/s, FLU], torque [Nm]

// Log
static float cmd_thrust;
static float r_roll;
static float r_pitch;

void controllerBodyRateInit(void)
{
  mpcInputInit();
}

bool controllerBodyRateTest(void)
{
  return true;
}

// Writes thrust [N] and the rate loop torques into control
static void applyOutput(control_t *control, const sensorData_t *sensors, float thrust)
{
  const struct vec w = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z));
  const struct vec ew = vsub(w, wd);
  M = mkvec(-kw * ew.x, -kw * ew.y, -kw_yaw * ew.z);

  control->controlMode = controlModeForceTorque;
  control->thrustSi = thrust;
  if (thrust > 0) {
    control->torqueX = M.x;
    control->torqueY = M.y;
    control->torqueZ = M.z;
  } else {
    control->torqueX = 0;
    control->torqueY = 0;
    control->torqueZ = 0;
  }

  cmd_thrust = thrust;
  r_roll = w.x;
  r_pitch = -w.y;
}

void controllerBodyRate(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  mpcCommand_t cmd;
  const mpcInputMode_t mode = mpcInputUpdate(setpoint, tick, &cmd);

  if (mode == mpcInputIdle) {
    mpcInputRunFallback(control, setpoint, sensors, state, tick);
    return;
  }
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  float thrust;
  if (mode == mpcInputActive) {
    const float lead = cmd.hasAngAccFF ? t21_lead : 0.0f;
    wd = mkvec(cmd.omegaRef[0] + cmd.angAccFF[0] * lead,
               cmd.omegaRef[1] + cmd.angAccFF[1] * lead,
               cmd.omegaRef[2] + cmd.angAccFF[2] * lead);
    thrust = cmd.thrust;
  } else if (mode == mpcInputAborted) {
    // The high-level commander has not taken over yet: hold level-ish with the last thrust for these few ticks, or
    // stop the motors if we are on the ground anyway
    wd = vzero();
    thrust = supervisorIsFlying() ? cmd.thrust : 0.0f;
  } else {
    wd = vzero();
    thrust = 0.0f;  // mpcInputOff
  }
  applyOutput(control, sensors, thrust);
}

PARAM_GROUP_START(bodyrate)
PARAM_ADD(PARAM_FLOAT, kw, &kw)
PARAM_ADD(PARAM_FLOAT, kw_yaw, &kw_yaw)
PARAM_ADD(PARAM_FLOAT, t21_lead, &t21_lead)
PARAM_GROUP_STOP(bodyrate)

LOG_GROUP_START(ctrlBR)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &M.x)
LOG_ADD(LOG_FLOAT, cmd_pitch, &M.y)
LOG_ADD(LOG_FLOAT, cmd_yaw, &M.z)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, wd_x, &wd.x)
LOG_ADD(LOG_FLOAT, wd_y, &wd.y)
LOG_ADD(LOG_FLOAT, wd_z, &wd.z)
LOG_GROUP_STOP(ctrlBR)
