/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
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
 * power_distribution_quadrotor.c - Crazyflie stock power distribution code
 */


#include "power_distribution.h"

#include <string.h>
#include "debug.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"
#include "math.h"
#include "platform_defaults.h"
#include "pm.h"

#if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0)) && defined(CONFIG_MOTORS_DEFAULT_IDLE_THRUST) && (CONFIG_MOTORS_DEFAULT_IDLE_THRUST > 0)
    #error "CONFIG_MOTORS_REQUIRE_ARMING must be defined and not set to 0 if CONFIG_MOTORS_DEFAULT_IDLE_THRUST is greater than 0"
#endif
#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

#define ANG_MEAN  893.933196
#define ANG_STD   350.589582
#define VBAT_MEAN 15.096775
#define VBAT_STD  0.607562

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;
static float armLength = 0.125f; // m;

// ang_vel = a * pwm + b
static float pwmToAngVelA = 0.065769f;  // 4310.17 / 65535
static float pwmToAngVelB = -131.538;

// thrust = c * signed_sum(ang_vel^2)
// static float angVelToThrust = 9.3945e-7f;  // old value: 9.3945e-7f; new value: 8.6584e-7f;
static float angVelToThrust = 1.26e-6f;  // old value: 9.3945e-7f; new value: 8.6584e-7f;

// torque = a/c * signed_sum(thrust) + b/c
// static float thrustToTorqueA = 5.5939e-7f;
// static float thrustToTorqueB = -0.4785f;
static float thrustToTorque = 1.4e-7f;

static float com_shift_x = 0.0f;
static float com_shift_y = 0.0f;

static float batteryVoltage = 16.8f; // V, fully charged 4S LiPo

static const double beta[10] = {
  16623.948470,   /* beta[0] */
  -627.506854,    /* beta[1] */
  32.047554,      /* beta[2] */
  1.493421,       /* beta[3] */
  7264.609848,    /* beta[4] */
  -247.540815,    /* beta[5] */
  18.580264,      /* beta[6] */
  36.646321,      /* beta[7] */
  1.165858,       /* beta[8] */
  -164.974485,    /* beta[9] */
};

static float batt_comp_a = -0.1245;  // with kR = 0.6: -0.1205
static float batt_comp_b = 2.768;  // with kR = 0.6: 2.6802

static uint16_t new_motor_pwm[4] = {0, 0, 0, 0};

static int8_t use_new_pwm_mapping = 1;

static float poly2d_eval(float ang_vel, float vbat)
{
    /* Normalize inputs */
    double a = ((double)ang_vel - ANG_MEAN) / ANG_STD;
    double v = ((double)vbat    - VBAT_MEAN) / VBAT_STD;

    /* Evaluate: sum beta[k] * a^i * v^j, same loop order as Python */
    double result = 0.0;
    int    k      = 0;
    int    degree = 3;

    double a_pow_i = 1.0;                  /* a^i */
    for (int i = 0; i <= degree; i++) {
        double v_pow_j = 1.0;              /* v^j */
        for (int j = 0; j <= degree - i; j++) {
            result += beta[k++] * a_pow_i * v_pow_j;
            v_pow_j *= v;
        }
        a_pow_i *= a;
    }

    /* Clamp to valid PWM range */
    if (result < 2500.0)        result = 0.0;
    if (result > 35000.0)    result = 35000.0;

    return (float)result;
}

void setComShift(float dx, float dy)
{
  com_shift_x = dx;
  com_shift_y = dy;
}

int powerDistributionMotorType(uint32_t id)
{
  return 1;
}

uint16_t powerDistributionStopRatio(uint32_t id)
{
  return 0;
}

void powerDistributionInit(void)
{
  #if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0))
  if(idleThrust > 0) {
    DEBUG_PRINT("WARNING: idle thrust will be overridden with value 0. Autoarming can not be on while idle thrust is higher than 0. If you want to use idle thust please use use arming\n");
  }
  #endif
}

bool powerDistributionTest(void)
{
  bool pass = true;
  return pass;
}

static uint16_t capMinThrust(float thrust, uint32_t minThrust) {
  if (thrust < minThrust) {
    return minThrust;
  }

  return thrust;
}

static void powerDistributionLegacy(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  int16_t r = control->roll / 2.0f;
  int16_t p = control->pitch / 2.0f;

  const float arm = 0.707106781f * armLength;
  float d1 = (arm + com_shift_x - com_shift_y) / arm;
  float d2 = (arm - com_shift_x - com_shift_y) / arm;
  float d3 = (arm - com_shift_x + com_shift_y) / arm;
  float d4 = (arm + com_shift_x + com_shift_y) / arm;
  motorThrustUncapped->motors.m1 = (uint16_t)(d1 * (float)control->thrust) - r + p + control->yaw;
  motorThrustUncapped->motors.m2 = (uint16_t)(d2 * (float)control->thrust) - r - p - control->yaw;
  motorThrustUncapped->motors.m3 = (uint16_t)(d3 * (float)control->thrust) + r - p + control->yaw;
  motorThrustUncapped->motors.m4 = (uint16_t)(d4 * (float)control->thrust) + r + p - control->yaw;
}

static void powerDistributionForceTorque(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
  static float motorForces[STABILIZER_NR_OF_MOTORS];

  const float arm = 0.707106781f * armLength;
  float d1 = (arm + com_shift_x - com_shift_y) / arm;
  float d2 = (arm - com_shift_x - com_shift_y) / arm;
  float d3 = (arm - com_shift_x + com_shift_y) / arm;
  float d4 = (arm + com_shift_x + com_shift_y) / arm;

  const float rollPart = 0.25f / arm * control->torqueX;
  const float pitchPart = 0.25f / arm * control->torqueY;
  const float thrustPart = 0.25f * control->thrustSi; // N (per rotor)
  // int yaw_sgn = (control->torqueZ > 0) - (control->torqueZ < 0);
  // const float yawPart = 0.25f * (control->torqueZ - yaw_sgn * thrustToTorqueB) * angVelToThrust / thrustToTorqueA;
  const float yawPart = 0.25f * control->torqueZ * angVelToThrust / thrustToTorque;

  batteryVoltage = pmGetBatteryVoltage();

  float batt_comp_ratio = batt_comp_a * batteryVoltage + batt_comp_b;

  motorForces[0] = d1 * thrustPart * batt_comp_ratio - rollPart - pitchPart - yawPart;
  motorForces[1] = d2 * thrustPart * batt_comp_ratio - rollPart + pitchPart + yawPart;
  motorForces[2] = d3 * thrustPart * batt_comp_ratio + rollPart + pitchPart - yawPart;
  motorForces[3] = d4 * thrustPart * batt_comp_ratio + rollPart - pitchPart + yawPart;

  float motorForcesNoBattComp[STABILIZER_NR_OF_MOTORS];
  motorForcesNoBattComp[0] = d1 * thrustPart - rollPart - pitchPart - yawPart;
  motorForcesNoBattComp[1] = d2 * thrustPart - rollPart + pitchPart + yawPart;
  motorForcesNoBattComp[2] = d3 * thrustPart + rollPart + pitchPart - yawPart;
  motorForcesNoBattComp[3] = d4 * thrustPart + rollPart - pitchPart + yawPart;


  // thrust = a * PWM^2 + b * PWM + c
  // maybe we shut put this computation somewhere else later
  const float pwmToThrustA = angVelToThrust * pwmToAngVelA * pwmToAngVelA * UINT16_MAX * UINT16_MAX;
  const float pwmToThrustB = 2 * angVelToThrust * pwmToAngVelA * pwmToAngVelB * UINT16_MAX;
  const float pwmToThrustC = angVelToThrust * pwmToAngVelB * pwmToAngVelB;

  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++) {
    float motorForce = motorForces[motorIndex];
    if (motorForce < 0.0f) {
      motorForce = 0.0f;
    }

    if (motorForcesNoBattComp[motorIndex] < 0.0f) {
      motorForcesNoBattComp[motorIndex] = 0.0f;
    }
    // calculate motor pwm in the range of [0, 65535]
    new_motor_pwm[motorIndex] = (uint16_t)poly2d_eval(sqrtf(motorForcesNoBattComp[motorIndex] / angVelToThrust), batteryVoltage);

    // calculate motor pwm in the range of [0, 1]
    float motor_pwm = (-pwmToThrustB + sqrtf(pwmToThrustB * pwmToThrustB - 4.0f * pwmToThrustA * (pwmToThrustC - motorForce))) / (2.0f * pwmToThrustA);
    if (motor_pwm < 0.04f) { // for this low value, the motors dont spin anyways
      motor_pwm = 0.0f;
    }
    
    if (use_new_pwm_mapping) {
      motorThrustUncapped->list[motorIndex] = new_motor_pwm[motorIndex];
    } else {
    motorThrustUncapped->list[motorIndex] = motor_pwm * UINT16_MAX;
    }
  }
}

static void powerDistributionForce(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
  // Not implemented yet
}

void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  switch (control->controlMode) {
    case controlModeLegacy:
      powerDistributionLegacy(control, motorThrustUncapped);
      break;
    case controlModeForceTorque:
      powerDistributionForceTorque(control, motorThrustUncapped);
      break;
    case controlModeForce:
      powerDistributionForce(control, motorThrustUncapped);
      break;
    default:
      // Nothing here
      break;
  }
}

bool powerDistributionCap(const motors_thrust_uncapped_t* motorThrustBatCompUncapped, motors_thrust_pwm_t* motorPwm)
{
  // const int32_t maxAllowedThrust = UINT16_MAX;
  const int32_t maxAllowedThrust = 30000;
  bool isCapped = false;

  // Find highest thrust
  int32_t highestThrustFound = 0;
  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    const int32_t thrust = motorThrustBatCompUncapped->list[motorIndex];
    if (thrust > highestThrustFound)
    {
      highestThrustFound = thrust;
    }
  }

  int32_t reduction = 0;
  if (highestThrustFound > maxAllowedThrust)
  {
    reduction = highestThrustFound - maxAllowedThrust;
    isCapped = true;
  }

  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    int32_t thrustCappedUpper = motorThrustBatCompUncapped->list[motorIndex] - reduction;
    motorPwm->list[motorIndex] = capMinThrust(thrustCappedUpper, powerDistributionGetIdleThrust());
  }

  return isCapped;
}

uint32_t powerDistributionGetIdleThrust()
{
  int32_t thrust = idleThrust;
  #if (!defined(CONFIG_MOTORS_REQUIRE_ARMING) || (CONFIG_MOTORS_REQUIRE_ARMING == 0))
    thrust = 0;
  #endif
  return thrust;
}

float powerDistributionGetMaxThrust() {
  return STABILIZER_NR_OF_MOTORS * THRUST_MAX;
}

/**
 * Power distribution parameters
 */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * This is often needed for brushless motors as
 * it takes time to start up the motor. Then a
 * common value is between 3000 - 6000.
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, idleThrust, &idleThrust)
PARAM_ADD(PARAM_INT8, new_pwm_map, &use_new_pwm_mapping)
PARAM_GROUP_STOP(powerDist)

/**
 * System identification parameters for quad rotor
 */
PARAM_GROUP_START(quadSysId)

/*PARAM_ADD(PARAM_FLOAT, thrustToTorque, &thrustToTorque)
PARAM_ADD(PARAM_FLOAT, pwmToThrustA, &pwmToThrustA)
PARAM_ADD(PARAM_FLOAT, pwmToThrustB, &pwmToThrustB)*/

/**
 * @brief Length of arms (m)
 *
 * The distance from the center to a motor
 */
PARAM_ADD(PARAM_FLOAT, armLength, &armLength)
PARAM_ADD(PARAM_FLOAT, thrustToTorque, &thrustToTorque)
PARAM_GROUP_STOP(quadSysId)


LOG_GROUP_START(powerDist)
LOG_ADD(LOG_UINT16, pwm0, &new_motor_pwm[0])
LOG_ADD(LOG_UINT16, pwm1, &new_motor_pwm[1])
LOG_ADD(LOG_UINT16, pwm2, &new_motor_pwm[2])
LOG_ADD(LOG_UINT16, pwm3, &new_motor_pwm[3])
LOG_GROUP_STOP(powerDist)