/*
 *
 * Copyright (c) 2019 Ewoud Smeur and Andre Luis Ogando Paraense
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * controller_indi.c - INDI low-level controller for an off-board NMPC, after Sun et al., "A Comparative Study of
 * Nonlinear MPC and Differential-Flatness-Based Control for Quadrotor Agile Flight" (T-RO 2022)
 *
 * Input (mpc_input.c): thrust T, body rate Omega_ref and angular acceleration feedforward alpha_ff, best as type 21
 * setpoints. Omega_ref is the predicted rate at the time the command takes effect; between commands it is propagated
 * with alpha_ff. Type 20 and UART commands (no alpha_ff) are flown as a pure rate loop around the INDI.
 *
 * Every attitude tick the gyro and the bidirectional-DShot rotor speeds are filtered (also while the fallback
 * controller flies, so that the takeover is bumpless), then indi_core.c computes the torque and allocates the rotor
 * thrusts with roll/pitch priority. The rotor thrusts go to the motors directly (controlModeMotorForces).
 *
 * Parameters (group indi) default to config/bumblebee.yaml and config/sil.yaml of quadrotor_nmpc.
 */
#include "controller_indi.h"

#include "indi_core.h"
#include "mpc_input.h"

#include "log.h"
#include "math3d.h"
#include "motors.h"
#include "param.h"
#include "supervisor.h"

#define ATTITUDE_UPDATE_DT (float)(1.0f / ATTITUDE_RATE)
#define RPM_TO_RAD_S (2.0f * M_PI_F / 60.0f)
#define RPM_INVALID UINT16_MAX  // motorsGetRPM: motor stopped, no telemetry yet, or a corrupted telemetry frame
#define RPM_HOLD_TICKS 10       // attitude ticks (20 ms) to bridge invalid readings with the last valid speed

static indiCore_t indi;
static float cutoff = 20.0f;     // Hz, Butterworth on gyro and rotor speeds; 12 Hz (agilicious) lacked phase margin
static float max_hold = 0.1f;    // s, longest propagation of Omega_ref with alpha_ff (the input times out at 0.1 s)
static uint8_t yaw_ndi = 0;
static float filtered_cutoff;    // cutoff the filters were initialized with
static float rotor_valid[4];     // rad/s, last valid rotor speed
static uint8_t rotor_invalid[4]; // consecutive invalid readings
static uint32_t rpm_invalid_count;  // log: invalid readings bridged with the last valid speed

// Log
static float omega_ref_used[3];  // rad/s, after propagation
static uint8_t mode_log;

void controllerINDIInit(void)
{
  mpcInputInit();
  indiCoreInit(&indi, ATTITUDE_UPDATE_DT, cutoff);
  filtered_cutoff = cutoff;
}

bool controllerINDITest(void)
{
  return true;
}

static void updateFilters(const sensorData_t *sensors)
{
  if (cutoff != filtered_cutoff) {
    indi.cutoff = filtered_cutoff = cutoff;
    indiCoreResetFilters(&indi);
  }
  const float gyro[3] = {radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z)};
  // The DShot decoder reports UINT16_MAX both for a stopped motor and for a corrupted telemetry frame. A single
  // bad frame on a spinning rotor must not look like a 0 rad/s sample to the INDI (a fake torque step), so invalid
  // readings are bridged with the last valid speed for a while; a motor that keeps reporting invalid is stopped.
  float rotor[4];
  for (int i = 0; i < 4; i++) {
    const uint16_t rpm = motorsGetRPM(i);
    if (rpm != RPM_INVALID) {
      rotor_valid[i] = rpm * RPM_TO_RAD_S;
      rotor_invalid[i] = 0;
    } else if (rotor_invalid[i] < RPM_HOLD_TICKS) {
      rotor_invalid[i]++;
      if (rotor_valid[i] > 0.0f) {
        rpm_invalid_count++;
      }
    } else {
      rotor_valid[i] = 0.0f;
    }
    rotor[i] = rotor_valid[i];
  }
  indiCoreUpdateFilters(&indi, gyro, rotor);
}

static void motorsOff(control_t *control)
{
  control->controlMode = controlModeMotorForces;
  for (int i = 0; i < 4; i++) {
    control->motorForces[i] = 0.0f;
    indi.u[i] = 0.0f;
  }
}

void controllerINDI(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors,
                    const state_t *state, const stabilizerStep_t stabilizerStep)
{
  mpcCommand_t cmd;
  const mpcInputMode_t mode = mpcInputUpdate(setpoint, stabilizerStep, &cmd);
  mode_log = mode;

  if (mode == mpcInputIdle) {
    mpcInputRunFallback(control, setpoint, sensors, state, stabilizerStep);
    if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
      updateFilters(sensors);
    }
    return;
  }
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    return;
  }
  updateFilters(sensors);
  indi.yawNdi = yaw_ndi != 0;

  float thrust;
  float alpha[3] = {0.0f, 0.0f, 0.0f};
  if (mode == mpcInputActive) {
    float hold = (float)(stabilizerStep - cmd.arrivalTick) / RATE_MAIN_LOOP;
    hold = fminf(fmaxf(hold, 0.0f), max_hold);
    for (int i = 0; i < 3; i++) {
      alpha[i] = cmd.angAccFF[i];
      omega_ref_used[i] = cmd.omegaRef[i] + cmd.angAccFF[i] * hold;
    }
    thrust = cmd.thrust;
  } else if (mode == mpcInputAborted) {
    // The high-level commander has not taken over yet: hold level-ish with the last thrust, or stay off on the ground
    for (int i = 0; i < 3; i++) {
      omega_ref_used[i] = 0.0f;
    }
    thrust = supervisorIsFlying() ? cmd.thrust : 0.0f;
  } else {
    thrust = 0.0f;  // mpcInputOff
  }

  if (thrust <= 0.0f) {
    motorsOff(control);
    return;
  }
  control->controlMode = controlModeMotorForces;
  indiCoreControl(&indi, thrust, omega_ref_used, alpha, control->motorForces);
}

PARAM_GROUP_START(indi)
PARAM_ADD(PARAM_FLOAT, Jxx, &indi.J[0])
PARAM_ADD(PARAM_FLOAT, Jyy, &indi.J[1])
PARAM_ADD(PARAM_FLOAT, Jzz, &indi.J[2])
PARAM_ADD(PARAM_FLOAT, rotor_x, &indi.rotorX)
PARAM_ADD(PARAM_FLOAT, rotor_y, &indi.rotorY)
PARAM_ADD(PARAM_FLOAT, kappa, &indi.kappa)
PARAM_ADD(PARAM_FLOAT, ct, &indi.ct)
PARAM_ADD(PARAM_FLOAT, u_min, &indi.uMin)
PARAM_ADD(PARAM_FLOAT, u_max, &indi.uMax)
PARAM_ADD(PARAM_FLOAT, k_x, &indi.kOmega[0])
PARAM_ADD(PARAM_FLOAT, k_y, &indi.kOmega[1])
PARAM_ADD(PARAM_FLOAT, k_z, &indi.kOmega[2])
PARAM_ADD(PARAM_FLOAT, cutoff, &cutoff)
PARAM_ADD(PARAM_FLOAT, max_hold, &max_hold)
PARAM_ADD(PARAM_UINT8, yaw_ndi, &yaw_ndi)
PARAM_ADD(PARAM_FLOAT, ndi_thrust, &indi.ndiThrust)  // N, NDI below this collective thrust (ground, bench)
PARAM_GROUP_STOP(indi)

LOG_GROUP_START(indi)
LOG_ADD(LOG_UINT8, mode, &mode_log)
LOG_ADD(LOG_UINT32, rpm_inv, &rpm_invalid_count)
LOG_ADD(LOG_FLOAT, wr_x, &omega_ref_used[0])
LOG_ADD(LOG_FLOAT, wr_y, &omega_ref_used[1])
LOG_ADD(LOG_FLOAT, wr_z, &omega_ref_used[2])
LOG_ADD(LOG_FLOAT, wf_x, &indi.omegaF[0])
LOG_ADD(LOG_FLOAT, wf_y, &indi.omegaF[1])
LOG_ADD(LOG_FLOAT, wf_z, &indi.omegaF[2])
LOG_ADD(LOG_FLOAT, a_x, &indi.alphaCmd[0])
LOG_ADD(LOG_FLOAT, a_y, &indi.alphaCmd[1])
LOG_ADD(LOG_FLOAT, a_z, &indi.alphaCmd[2])
LOG_ADD(LOG_FLOAT, tf_x, &indi.tauF[0])
LOG_ADD(LOG_FLOAT, tf_y, &indi.tauF[1])
LOG_ADD(LOG_FLOAT, tf_z, &indi.tauF[2])
LOG_ADD(LOG_FLOAT, t_x, &indi.tauCmd[0])
LOG_ADD(LOG_FLOAT, t_y, &indi.tauCmd[1])
LOG_ADD(LOG_FLOAT, t_z, &indi.tauCmd[2])
LOG_ADD(LOG_FLOAT, u1, &indi.u[0])
LOG_ADD(LOG_FLOAT, u2, &indi.u[1])
LOG_ADD(LOG_FLOAT, u3, &indi.u[2])
LOG_ADD(LOG_FLOAT, u4, &indi.u[3])
LOG_GROUP_STOP(indi)
