/*
 * controller_geom.h - Geometric Controller Interface
 */
#ifndef __CONTROLLER_GEOM_H__
#define __CONTROLLER_GEOM_H__

#include "stabilizer_types.h"

void controllerGeomInit(void);
bool controllerGeomTest(void);
void controllerGeomReset(void);
void controllerGeom(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);
void setMode(bool val);
float getPsi(void);
#endif //__CONTROLLER_GEOM_H__
