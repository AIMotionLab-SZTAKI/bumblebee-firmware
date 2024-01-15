/*
 * controller_switch.h - Controller Switching Interface
 */
#ifndef __CONTROLLER_SWITCH_H__
#define __CONTROLLER_SWITCH_H__

#include "stabilizer_types.h"

void controllerSwitchInit(void);
bool controllerSwitchTest(void);
void controllerSwitchReset(void);
void controllerSwitch(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);
#endif //__CONTROLLER_SWITCH_H__
