/*
 * controller_lqr_2dof.h - LQR Payload Controller Interface
 */
#ifndef __CONTROLLER_LQR_2DOF_H__
#define __CONTROLLER_LQR_2DOF_H__

#include "stabilizer_types.h"

void controllerLqr2DofInit(void);
bool controllerLqr2DofTest(void);
void controllerLqr2DofReset(void);
void lqr2RegisterMemoryHandler(void);
void controllerLqr2Dof(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);
void setLoadState2Dof(const poseMeasurement_t *measurement, const uint32_t dt_ms);
void setLqr2DofParams(float params[], int param_num, uint16_t timestamp);
#endif //__CONTROLLER_LQR_2DOF_H__
