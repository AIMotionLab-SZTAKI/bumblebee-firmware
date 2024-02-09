#ifndef __CONTROLLER_LQR_H__
#define __CONTROLLER_LQR_H__

#include "stabilizer_types.h"

void setLqrParams(float params[], int param_num, uint16_t timestamp);
void controllerLqrInit(void);
bool controllerLqrTest(void);
void controllerLqrReset(void);
void controllerLqr(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);
#endif //__CONTROLLER_LQR_H__
