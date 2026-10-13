/**
 * indi_core.h - INDI angular acceleration control and motor allocation (Sun et al., T-RO 2022)
 *
 * Pure math without hardware access, so that it can be compiled on a PC and checked against the Python reference
 * (quadrotor_nmpc: qnmpc/control/inner.py IndiInner, allocate_rp_priority). Units SI, FLU body frame.
 *
 *   tau = tau_f + J (alpha_cmd - Omega_dot_f),  alpha_cmd = alpha_ff + K (Omega_ref - Omega_f)
 *
 * tau_f is the torque of the filtered measured rotor speeds. Gyro and rotor speeds go through the same 2nd-order
 * Butterworth, so tau_f and Omega_dot_f are synchronized. Collective thrust is passed through.
 *
 * Rotor layout and order as the firmware mixer: M1 front-right (x, -y), M2 back-right (-x, -y), M3 back-left
 * (-x, y), M4 front-left (x, y); yaw torque of rotor i = yaw_dir_i * kappa * u_i with yaw_dir = (-1, 1, -1, 1).
 */
#ifndef __INDI_CORE_H__
#define __INDI_CORE_H__

#include <stdbool.h>

#include "filter.h"

typedef struct {
  // Parameters (bumblebee.yaml)
  float J[3];          // kg m^2, diagonal inertia
  float rotorX;        // m, |x| of the rotors w.r.t. the CoG
  float rotorY;        // m, |y| of the rotors w.r.t. the CoG
  float kappa;         // m, yaw torque / thrust
  float ct;            // N s^2 / rad^2, thrust = ct * w^2
  float uMin, uMax;    // N, per rotor
  float kOmega[3];     // 1/s, rate feedback gains
  bool yawNdi;         // replace the INDI yaw torque by NDI (agilicious); does not reject yaw disturbances
  float ndiThrust;     // N, below this collective thrust NDI replaces INDI on all axes: on the ground the drone
                       // cannot rotate, Omega_dot_f stays 0 and the incremental law winds up to full differential
                       // thrust (seen on the bench). In flight the NMPC never commands that little thrust.
  float dt;            // s, update period
  float cutoff;        // Hz, Butterworth cutoff

  // Filter state (initialized with the first sample)
  struct SecondOrderLowPass gyroF[3];
  struct SecondOrderLowPass rotorF[4];
  bool filtersReady;
  float gyro[3];       // rad/s, last unfiltered gyro sample

  // Last results, for logging
  float omegaF[3];     // rad/s
  float omegaDotF[3];  // rad/s^2
  float tauF[3];       // Nm
  float alphaCmd[3];   // rad/s^2
  float tauCmd[3];     // Nm
  float u[4];          // N, rotor thrust commands
} indiCore_t;

// Sets the parameters to the Bumblebee defaults and resets the filters.
void indiCoreInit(indiCore_t *c, float dt, float cutoff);

// Resets the filters, e.g. after a parameter change of dt or cutoff; they restart from the next sample.
void indiCoreResetFilters(indiCore_t *c);

// Feeds one gyro sample [rad/s, FLU] and the measured rotor speeds [rad/s, M1..M4]. Call every period, also while
// another controller flies, so that the filters are warm when the INDI takes over.
void indiCoreUpdateFilters(indiCore_t *c, const float gyro[3], const float rotorSpeed[4]);

// INDI law and allocation from the latest filter state: thrust [N], Omega_ref [rad/s], alpha_ff [rad/s^2] ->
// rotor thrust commands u [N].
void indiCoreControl(indiCore_t *c, float thrust, const float omegaRef[3], const float alphaFF[3], float u[4]);

// u = G1^-1 [T; tau] with saturation priority roll/pitch > collective thrust > yaw, within [uMin, uMax].
void indiCoreAllocate(const indiCore_t *c, const float mu[4], float u[4]);

#endif // __INDI_CORE_H__
