/*
 * controller_lqr_1dof.h - LQR Payload Controller Interface
 */
#ifndef __CONTROLLER_LQR_1DOF_H__
#define __CONTROLLER_LQR_1DOF_H__

#include "stabilizer_types.h"

void controllerLqr1DofInit(void);
bool controllerLqr1DofTest(void);
void controllerLqr1DofReset(void);
void controllerLqr1Dof(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);
void setLoadState1Dof(const poseMeasurement_t *measurement, const uint32_t dt_ms);
#endif //__CONTROLLER_LQR_1DOF_H__
