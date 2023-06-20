/*
 * controller_lqr_2dof.h - LQR Payload Controller Interface
 */
#ifndef __CONTROLLER_LQR_2DOF_H__
#define __CONTROLLER_LQR_2DOF_H__

#include "stabilizer_types.h"

void controllerLqr2DofInit(void);
bool controllerLqr2DofTest(void);
void controllerLqr2DofReset(void);
void controllerLqr2Dof(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);
void setLoadState2Dof(const poseMeasurement_t *measurement, const uint32_t dt_ms);
#endif //__CONTROLLER_LQR_2DOF_H__
