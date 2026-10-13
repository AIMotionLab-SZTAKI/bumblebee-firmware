/**
 * mpc_input.h - External (off-board MPC) command input shared by the Bodyrate and INDI controllers
 *
 * Two sources, selected by the parameter mpcin.input_src:
 *   1: CRTP setpoints of type 20 / 21 (crtp_commander_generic.c), streamed through the Skybrush server. While no
 *      MPC command comes in, the fallback controller flies (take off, hover and land with the high-level commander).
 *      MPC commands take over when they arrive and hand back cleanly on a notify-setpoints-stop. If they stop
 *      without one, or keep being marked invalid, the session is aborted: the high-level commander lands the drone
 *      and MPC setpoints are locked out until the drone is on the ground and none arrived for a while.
 *   0: UART (Raspberry Pi with its own radio), only while mpcin.uart_ctrl = 1; rotor speeds are sent back. After
 *      too many invalid or missing commands the motors are stopped.
 */
#ifndef __MPC_INPUT_H__
#define __MPC_INPUT_H__

#include <stdbool.h>
#include <stdint.h>

#include "stabilizer_types.h"

typedef struct {
  float thrust;          // N
  float omegaRef[3];     // rad/s, FLU body frame (type 21: rate at the time the command takes effect)
  float angAccFF[3];     // rad/s^2, FLU body frame, 0 unless hasAngAccFF
  bool hasAngAccFF;      // type 21
  uint8_t status;        // nonzero: marked invalid by the sender
  uint32_t arrivalTick;  // stabilizer tick at which this command was consumed
  uint32_t session;      // incremented at the start of every MPC session
} mpcCommand_t;

typedef enum {
  mpcInputIdle = 0,     // no MPC session: run the fallback controller (mpcInputRunFallback)
  mpcInputActive = 1,   // follow the command
  mpcInputAborted = 2,  // session aborted, the high-level commander has not taken over yet: hold level
  mpcInputOff = 3,      // UART source without uart_ctrl: motors off
} mpcInputMode_t;

void mpcInputInit(void);

// Called by the CRTP task for every incoming MPC setpoint. Returns false while MPC setpoints are locked out after an
// aborted session, in which case the setpoint must be dropped.
bool mpcInputAcceptSetpoint(void);

// Called by the controller on every stabilizer tick, before anything else. Consumes new commands, runs the session
// state machine and the UART link, and returns what the controller should do; *cmd holds the latest command.
mpcInputMode_t mpcInputUpdate(const setpoint_t *setpoint, uint32_t tick, mpcCommand_t *cmd);

void mpcInputRunFallback(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors,
                         const state_t *state, uint32_t tick);

#endif // __MPC_INPUT_H__
