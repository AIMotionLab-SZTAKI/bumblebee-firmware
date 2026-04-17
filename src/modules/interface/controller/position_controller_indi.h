/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 * position_controller_indi.h - INDI Controller Interface
 */

#ifndef __POSITION_CONTROLLER_INDI_H__
#define __POSITION_CONTROLLER_INDI_H__

// Thrust command (in motor units)
#define MIN_THRUST  0
#define MAX_THRUST  60000

// Cutoff frequency used in the filtering 
#define POSITION_INDI_FILT_CUTOFF 8.0f

#include "controller_indi.h"
#include "stabilizer_types.h"
#include "filter.h"
#include "math3d.h"
#include "log.h"
#include "param.h"

// these parameters are used in the filtering of the angular acceleration [Hz]
#define STABILIZATION_INDI_FILT_CUTOFF 8.0f

// the yaw sometimes requires more filtering
#define STABILIZATION_INDI_FILT_CUTOFF_R STABILIZATION_INDI_FILT_CUTOFF

// Control effectiveness coefficients values Volodscoi
#define STABILIZATION_INDI_G1_P 0.0066146f
#define STABILIZATION_INDI_G1_Q 0.0052125f
#define STABILIZATION_INDI_G1_R 0.001497f
#define STABILIZATION_INDI_G2_R 0.000043475f

// Control effectiveness coefficients values Max Kemmeren, these will become the new ones, not finalized yet
// #define STABILIZATION_INDI_G1_P 0.0032502f
// #define STABILIZATION_INDI_G1_Q 0.0027555f
// #define STABILIZATION_INDI_G1_R 0.00068154f
// #define STABILIZATION_INDI_G2_R 0.00001725f

//Proportional gains inner INDI, attitude error
#define STABILIZATION_INDI_REF_ERR_P 5.0f
#define STABILIZATION_INDI_REF_ERR_Q 5.0f
#define STABILIZATION_INDI_REF_ERR_R 5.0f

//Derivative gains inner INDI, attitude rate error
#define STABILIZATION_INDI_REF_RATE_P 24.0f
#define STABILIZATION_INDI_REF_RATE_Q 24.0f
#define STABILIZATION_INDI_REF_RATE_R 24.0f

// Actuator model coefficient
#define STABILIZATION_INDI_ACT_DYN_P 0.03149f
#define STABILIZATION_INDI_ACT_DYN_Q 0.03149f
#define STABILIZATION_INDI_ACT_DYN_R 0.03149f


struct Vectr {
  float x; 
  float y; 
  float z;
};

struct Angles {
  float phi; 
  float theta;
  float psi;
};

struct IndiOuterVariables {

  Butterworth2LowPass ddxi[3];
  Butterworth2LowPass ang[3];
  Butterworth2LowPass thr[3];

  float filt_cutoff;
  float act_dyn_posINDI;

  struct Vectr linear_accel_ref;  
  struct Vectr linear_accel_err;
  struct Vectr linear_accel_s;    // acceleration sensed
  struct Vectr linear_accel_f;    // acceleration filtered
  struct Vectr linear_accel_ft;   // acceleration filtered transformed to NED 
  struct Angles attitude_s;       // attitude senssed (here estimated)
  struct Angles attitude_f;       // attitude filtered
  struct Angles attitude_c;       // attitude commanded to the inner loop
  float phi_tilde;                // roll angle increment
  float theta_tilde;              // pitch angle increment
  float T_tilde;                  // thrust increment
  float T_inner;                  // thrust to inner INDI
  float T_inner_f;
  float T_incremented;
};

void positionControllerINDIInit(void);
void positionControllerINDI(const sensorData_t *sensors,
                            const setpoint_t *setpoint,
                            const state_t *state, 
                            vector_t *refOuterINDI);

#endif //__POSITION_CONTROLLER_INDI_H__
