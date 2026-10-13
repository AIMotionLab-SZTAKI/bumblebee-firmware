/**
 * indi_core.c - INDI angular acceleration control and motor allocation, see indi_core.h
 */
#include <math.h>

#include "indi_core.h"

#define PI_F 3.14159265358979323846f

// Sign of each rotor's contribution to (tau_x, tau_y, tau_z), firmware mixer order M1..M4:
// tau_x = sum(y_i u_i), tau_y = -sum(x_i u_i), tau_z = kappa * sum(yaw_dir_i u_i)
static const float SX[4] = {-1.0f, -1.0f, 1.0f, 1.0f};   // sign of y_i
static const float SY[4] = {-1.0f, 1.0f, 1.0f, -1.0f};   // sign of -x_i
static const float SZ[4] = {-1.0f, 1.0f, -1.0f, 1.0f};   // yaw_dir_i

void indiCoreInit(indiCore_t *c, float dt, float cutoff)
{
  c->J[0] = 0.00246f;
  c->J[1] = 0.00272f;
  c->J[2] = 0.00443f;
  c->rotorX = 0.083f;
  c->rotorY = 0.087f;
  c->kappa = 0.111f;
  c->ct = 1.26e-6f;
  c->uMin = 0.0f;
  c->uMax = 5.0f;
  c->kOmega[0] = 20.0f;
  c->kOmega[1] = 20.0f;
  c->kOmega[2] = 5.0f;
  c->yawNdi = false;
  c->ndiThrust = 2.0f;  // about 30 % of hover thrust
  c->dt = dt;
  c->cutoff = cutoff;
  indiCoreResetFilters(c);
}

void indiCoreResetFilters(indiCore_t *c)
{
  c->filtersReady = false;
}

void indiCoreUpdateFilters(indiCore_t *c, const float gyro[3], const float rotorSpeed[4])
{
  if (!c->filtersReady) {
    const float tau = 1.0f / (2.0f * PI_F * c->cutoff);  // K = tan(pi fc dt), as agilib / the Python reference
    for (int i = 0; i < 3; i++) {
      init_second_order_low_pass(&c->gyroF[i], tau, 0.7071f, c->dt, gyro[i]);
    }
    for (int i = 0; i < 4; i++) {
      init_second_order_low_pass(&c->rotorF[i], tau, 0.7071f, c->dt, rotorSpeed[i]);
    }
    c->filtersReady = true;
  }
  for (int i = 0; i < 3; i++) {
    c->gyro[i] = gyro[i];
    c->omegaF[i] = update_second_order_low_pass(&c->gyroF[i], gyro[i]);
    c->omegaDotF[i] = (c->gyroF[i].o[0] - c->gyroF[i].o[1]) / c->dt;
  }
  float tau[3] = {0.0f, 0.0f, 0.0f};
  for (int i = 0; i < 4; i++) {
    const float w = update_second_order_low_pass(&c->rotorF[i], rotorSpeed[i]);
    const float ui = c->ct * w * w;
    tau[0] += SX[i] * c->rotorY * ui;
    tau[1] += SY[i] * c->rotorX * ui;
    tau[2] += SZ[i] * c->kappa * ui;
  }
  for (int i = 0; i < 3; i++) {
    c->tauF[i] = tau[i];
  }
}

void indiCoreControl(indiCore_t *c, float thrust, const float omegaRef[3], const float alphaFF[3], float u[4])
{
  // NDI torque J alpha + w x J w with the unfiltered rate, for the axes not flown with INDI
  const float *w = c->gyro;
  const float gyroscopic[3] = {
    w[1] * c->J[2] * w[2] - w[2] * c->J[1] * w[1],
    w[2] * c->J[0] * w[0] - w[0] * c->J[2] * w[2],
    w[0] * c->J[1] * w[1] - w[1] * c->J[0] * w[0],
  };
  const bool ndi = thrust < c->ndiThrust;  // on the ground (see ndiThrust)
  for (int i = 0; i < 3; i++) {
    c->alphaCmd[i] = alphaFF[i] + c->kOmega[i] * (omegaRef[i] - c->omegaF[i]);
    if (ndi || (i == 2 && c->yawNdi)) {
      c->tauCmd[i] = c->J[i] * c->alphaCmd[i] + gyroscopic[i];
    } else {
      c->tauCmd[i] = c->tauF[i] + c->J[i] * (c->alphaCmd[i] - c->omegaDotF[i]);
    }
  }
  const float mu[4] = {thrust, c->tauCmd[0], c->tauCmd[1], c->tauCmd[2]};
  indiCoreAllocate(c, mu, u);
  for (int i = 0; i < 4; i++) {
    c->u[i] = u[i];
  }
}

void indiCoreAllocate(const indiCore_t *c, const float mu[4], float u[4])
{
  // Closed-form G1^-1 of the symmetric layout: u_i = T/4 + SX_i tau_x / (4 y) + SY_i tau_y / (4 x) + SZ_i tau_z / (4 kappa)
  float uRp[4], uY[4];
  float lo = INFINITY, hi = -INFINITY;
  for (int i = 0; i < 4; i++) {
    uRp[i] = 0.25f * mu[0] + SX[i] * mu[1] / (4.0f * c->rotorY) + SY[i] * mu[2] / (4.0f * c->rotorX);
    uY[i] = SZ[i] * mu[3] / (4.0f * c->kappa);
    lo = fminf(lo, uRp[i]);
    hi = fmaxf(hi, uRp[i]);
  }
  // Shift the collective thrust (uniform over the rotors) so that roll/pitch fit, if possible
  const float shiftLo = c->uMin - lo, shiftHi = c->uMax - hi;
  const float shift = shiftLo <= shiftHi ? fminf(fmaxf(0.0f, shiftLo), shiftHi) : 0.5f * (shiftLo + shiftHi);
  // Then scale the yaw part to what is left
  float s = 1.0f;
  for (int i = 0; i < 4; i++) {
    const float base = uRp[i] + shift;
    if (uY[i] > 0.0f) {
      s = fminf(s, (c->uMax - base) / uY[i]);
    } else if (uY[i] < 0.0f) {
      s = fminf(s, (c->uMin - base) / uY[i]);
    }
  }
  s = fmaxf(s, 0.0f);
  for (int i = 0; i < 4; i++) {
    u[i] = fminf(fmaxf(uRp[i] + shift + s * uY[i], c->uMin), c->uMax);
  }
}
