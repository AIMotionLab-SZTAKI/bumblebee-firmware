
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "controller_pid.h"

#include "log.h"
#include "param.h"
#include "math3d.h"

// Communication module include
#include "communication.h"

// Stabilizer include to be able to switch into emergency mode
#include "stabilizer.h"

// Power management to get battery voltage
#include "pm.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)
#define COMMUNICATION_RATE RATE_100_HZ

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static attitude_t rateDesired_ext;
static float actuatorThrust;
static float thrust_ext;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

static uint8_t external_control = 0;

static bool enable_uart_comm = true;

static float batt_comp_a = -0.1245;  // with kR = 0.6: -0.1205
static float batt_comp_b = 2.768;  // with kR = 0.6: 2.6802

static float supplyVoltage;

// ang_vel = a * pwm + b
static float pwmToAngVelA = 0.065769f;
static float pwmToAngVelB = -131.538;

// thrust = c * signed_sum(ang_vel^2)
static float angVelToThrust = 9.3945e-7f;


float getThrustPwm(float thrust_N) 
{
  // thrust = a * PWM^2 + b * PWM + c
  const float pwmToThrustA = angVelToThrust * pwmToAngVelA * pwmToAngVelA * UINT16_MAX * UINT16_MAX;
  const float pwmToThrustB = 2 * angVelToThrust * pwmToAngVelA * pwmToAngVelB * UINT16_MAX;
  const float pwmToThrustC = angVelToThrust * pwmToAngVelB * pwmToAngVelB;

  thrust_N = 0.25f * thrust_N;
  // calculate motor pwm in the range of [0, 1]
  float thrust_ratio = (-pwmToThrustB + sqrtf(pwmToThrustB * pwmToThrustB - 4.0f * pwmToThrustA * (pwmToThrustC - thrust_N))) / (2.0f * pwmToThrustA);
  return thrust_ratio * UINT16_MAX;
}

void controllerPidInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
  supplyVoltage = pmGetBatteryVoltage();
}

bool controllerPidTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

void controllerPid(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  control->controlMode = controlModeLegacy;

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
      attitudeDesired.yaw = capAngle(attitudeDesired.yaw + setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT);

      float yawMaxDelta = attitudeControllerGetYawMaxDelta();
      if (yawMaxDelta != 0.0f)
      {
      float delta = capAngle(attitudeDesired.yaw-state->attitude.yaw);
      // keep the yaw setpoint within +/- yawMaxDelta from the current yaw
        if (delta > yawMaxDelta)
        {
          attitudeDesired.yaw = state->attitude.yaw + yawMaxDelta;
        }
        else if (delta < -yawMaxDelta)
        {
          attitudeDesired.yaw = state->attitude.yaw - yawMaxDelta;
        }
      }
    } else if (setpoint->mode.yaw == modeAbs) {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    } else if (setpoint->mode.quat == modeAbs) {
      struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
      struct vec rpy = quat2rpy(setpoint_quat);
      attitudeDesired.yaw = degrees(rpy.z);
    }

    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
  }

  if (RATE_DO_EXECUTE(COMMUNICATION_RATE, tick)) {
    if (enable_uart_comm) {
      /*sendDataUART("C", &actuatorThrust, state);
      float dummy1 = 12.34;
      float dummy2 = 345.12;
      sendDataUART("T", &actuatorThrust, &dummy1, &dummy2);
      */
      float dummy1 = 12.34;
      float dummy2 = 345.12;
      sendDataUART("F", &actuatorThrust, &dummy1, &dummy2);
      uart_packet receiverPacket;
      if (receiveDataUART(&receiverPacket)) {
        if (receiverPacket.serviceType == CONTROL_PACKET) {
          handle_control_packet(&receiverPacket, &thrust_ext, &rateDesired_ext.roll, &rateDesired_ext.pitch, &rateDesired_ext.yaw);
        } else if (receiverPacket.serviceType == FORWARDED_CONTROL_PACKET) {
          handle_forwarded_packet(&receiverPacket, &thrust_ext, &rateDesired_ext.roll, &rateDesired_ext.pitch, &rateDesired_ext.yaw);
        }
        // convert thrust from N to PWM
        supplyVoltage = pmGetBatteryVoltage();  
        float mass_ratio = batt_comp_a * supplyVoltage + batt_comp_b;
        float thrust_battery_corrected = thrust_ext * mass_ratio;
        thrust_ext = getThrustPwm(thrust_battery_corrected);
      } else if (external_control) { // communication not successful but still trying to control externally
        stabilizerSetEmergencyStop();  // TODO: just disable uart communication and find a safe setpoint for PID
      }
    }
  }

  if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {
      actuatorThrust = setpoint->thrust;
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      attitudeDesired.roll = setpoint->attitude.roll;
      attitudeDesired.pitch = setpoint->attitude.pitch;
    }

    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity) {
      rateDesired.roll = setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudePID();
    }
    if (setpoint->mode.pitch == modeVelocity) {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID();
    }

    // TODO: Investigate possibility to subtract gyro drift.
    if (external_control) {
      attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired_ext.roll, rateDesired_ext.pitch, rateDesired_ext.yaw);
    } else {
      attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);
    }

    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    control->yaw = -control->yaw;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->acc.z;
  }

  if (external_control) {
    control->thrust = thrust_ext;
  } else {
    control->thrust = actuatorThrust;
  }

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllPID();
    positionControllerResetAllPID();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Gyro roll measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
/**
 * @brief Gyro pitch measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
/**
 * @brief Yaw  measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief Acceleration in the zaxis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &accelz)
/**
 * @brief Thrust command without (tilt)compensation
 */
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_ADD(LOG_FLOAT, rollRate_ext,  &rateDesired_ext.roll)
LOG_ADD(LOG_FLOAT, pitchRate_ext, &rateDesired_ext.pitch)
LOG_ADD(LOG_FLOAT, yawRate_ext,   &rateDesired_ext.yaw)
LOG_ADD(LOG_FLOAT, thrust_ext,   &thrust_ext)
LOG_GROUP_STOP(controller)

PARAM_GROUP_START(pid)
PARAM_ADD(PARAM_UINT8, external_control, &external_control)
PARAM_GROUP_STOP(pid)