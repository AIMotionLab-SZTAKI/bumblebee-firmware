/**
 * mpc_input.c - External (off-board MPC) command input shared by the Bodyrate and INDI controllers, see mpc_input.h
 *
 * Factored out of controller_body_rate.c; the behaviour of both input sources is unchanged.
 */
#include <string.h>

#include "mpc_input.h"

#include "FreeRTOS.h"
#include "task.h"

#include "commander.h"
#include "communication.h"
#include "controller.h"
#include "controller_geom.h"
#include "controller_pid.h"
#include "crtp_commander_high_level.h"
#include "log.h"
#include "math3d.h"
#include "motors.h"
#include "param.h"
#include "power_distribution.h"
#include "supervisor.h"

#define MPC_MAX_FAILS 20                   // consecutive invalid MPC commands before aborting
#define MPC_LOCKOUT_RELEASE_MS 1000
#define UART_MAX_FAILS 20                  // UART: invalid commands (+11 per missing one) before stopping the motors

// Where the external commands come from: 0 UART, 1 CRTP (Skybrush server)
static uint8_t input_src = 1;
static uint8_t uart_ctrl = 0;              // UART: follow the received commands (else motors off)
static uint16_t timeout_ms = 100;          // CRTP: abort if no new MPC command for this long
static float land_velocity = 0.3f;         // m/s, landing after an abort
static uint8_t fallback = ControllerTypeGeom;  // ControllerTypeGeom or ControllerTypePID

typedef enum {
  mpcIdle = 0,     // no MPC command, fallback controller
  mpcActive = 1,   // following MPC commands
  mpcAborted = 2,  // MPC setpoints locked out
} mpcState_t;

static uint8_t state = mpcIdle;          // mpcState_t, also written by the CRTP task (atomic on the M4)
static uint32_t last_arrival;            // FreeRTOS tick of the last MPC setpoint received, accepted or not
static uint32_t last_timestamp;          // timestamp of the last MPC setpoint consumed, to detect new ones
static uint32_t rx_count;                // number of MPC setpoints consumed (read by the Skybrush server)
static int fail_counter;
static mpcCommand_t command;

// Log only: the received command (rates in the type 20 convention: deg/s, pitch inverted)
static uint8_t cmd_type;
static attitude_t cmd_rate_deg;

void mpcInputInit(void)
{
  controllerGeomInit();
  controllerPidInit();
  memset(&command, 0, sizeof(command));
}

bool mpcInputAcceptSetpoint(void)
{
  // Runs in the CRTP task, for every MPC setpoint that arrives
  uint32_t now = xTaskGetTickCount();
  if (state == mpcAborted && now - last_arrival > M2T(MPC_LOCKOUT_RELEASE_MS) && !supervisorIsFlying()) {
    state = mpcIdle;
  }
  last_arrival = now;
  return state != mpcAborted;
}

void mpcInputRunFallback(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors,
                         const state_t *state_, uint32_t tick)
{
  if (fallback == ControllerTypePID) {
    controllerPid(control, setpoint, sensors, state_, tick);
  } else {
    controllerGeom(control, setpoint, sensors, state_, tick);
  }
}

static void resetFallback(void)
{
  if (fallback == ControllerTypePID) {
    controllerPidInit();
  } else {
    controllerGeomReset();
  }
}

static void abortSession(void)
{
  state = mpcAborted;
  // Hand over to the high-level commander, starting from the current state, and land
  commanderRelaxPriority();
  if (supervisorIsFlying()) {
    crtpCommanderHighLevelLandWithVelocity(0.0f, land_velocity, false);
  }
}

// Rates in the type 20 convention (deg/s, pitch inverted) -> FLU rad/s
static void setRatesFromDeg(float roll, float pitch, float yaw)
{
  command.omegaRef[0] = radians(roll);
  command.omegaRef[1] = -radians(pitch);
  command.omegaRef[2] = radians(yaw);
  cmd_rate_deg.roll = roll;
  cmd_rate_deg.pitch = pitch;
  cmd_rate_deg.yaw = yaw;
}

static void consumeCrtp(const setpoint_t *setpoint, uint32_t tick)
{
  rx_count++;
  command.thrust = setpoint->thrust;
  command.status = setpoint->mpc.status;
  command.arrivalTick = tick;
  command.hasAngAccFF = setpoint->mpc.hasAngAccFF;
  if (setpoint->mpc.hasAngAccFF) {
    for (int i = 0; i < 3; i++) {
      command.omegaRef[i] = setpoint->mpc.omegaRef[i];
      command.angAccFF[i] = setpoint->mpc.angAccFF[i];
    }
    cmd_rate_deg = setpoint->attitudeRate;
  } else {
    setRatesFromDeg(setpoint->attitudeRate.roll, setpoint->attitudeRate.pitch, setpoint->attitudeRate.yaw);
    command.angAccFF[0] = command.angAccFF[1] = command.angAccFF[2] = 0.0f;
  }
  cmd_type = setpoint->mpc.hasAngAccFF ? 21 : 20;
  setComShift(setpoint->mpc.comShiftX, setpoint->mpc.comShiftY);
  fail_counter = setpoint->mpc.status ? fail_counter + 1 : 0;
}

static mpcInputMode_t updateCrtp(const setpoint_t *setpoint, uint32_t tick)
{
  if (!setpoint->mpc.active) {
    // High-level commander (takeoff, hover, land...), or nothing at all: the fallback controller flies
    if (state == mpcActive) {
      // The MPC handed over cleanly (notify-setpoints-stop)
      state = mpcIdle;
      resetFallback();
    }
    return mpcInputIdle;
  }

  if (state == mpcIdle) {
    // First command of an MPC session
    state = mpcActive;
    fail_counter = 0;
    command.session++;
  }

  if (state == mpcActive) {
    if (setpoint->timestamp != last_timestamp) {
      last_timestamp = setpoint->timestamp;
      consumeCrtp(setpoint, tick);
    }
    if (xTaskGetTickCount() - setpoint->timestamp > M2T(timeout_ms) || fail_counter >= MPC_MAX_FAILS) {
      abortSession();
    }
  }
  return state == mpcActive ? mpcInputActive : mpcInputAborted;
}

static mpcInputMode_t updateUart(uint32_t tick)
{
  if (RATE_DO_EXECUTE(COMMUNICATION_RATE, tick)) {
    uint16_t rpm[4];
    for (int i = 0; i < 4; i++) {
      rpm[i] = motorsGetRPM(i);
    }
    sendDataUART("F", rpm, rpm + 1, rpm + 2, rpm + 3);
    uart_packet packet;
    if (receiveDataUART(&packet)) {
      float thrust = command.thrust, roll = cmd_rate_deg.roll, pitch = cmd_rate_deg.pitch, yaw = cmd_rate_deg.yaw;
      if (packet.serviceType == CONTROL_PACKET) {
        handle_control_packet(&packet, &thrust, &roll, &pitch, &yaw);
      } else if (packet.serviceType == FORWARDED_CONTROL_PACKET) {
        float status = 0.0f, com_x = 0.0f, com_y = 0.0f;
        handle_forwarded_packet(&packet, &thrust, &roll, &pitch, &yaw, &status, &com_x, &com_y);
        setComShift(com_x, com_y);
        command.status = status > 0.5f;
        fail_counter = command.status ? fail_counter + 1 : 0;
      }
      command.thrust = thrust;
      setRatesFromDeg(roll, pitch, yaw);
      command.hasAngAccFF = false;
      command.angAccFF[0] = command.angAccFF[1] = command.angAccFF[2] = 0.0f;
      command.arrivalTick = tick;
      cmd_type = 0;
      rx_count++;
    } else if (uart_ctrl) {
      fail_counter += 11;  // communication timeout while trying to control externally
    }
    if (fail_counter >= UART_MAX_FAILS) {
      motorsStop();  // emergency mode
    }
  }
  return uart_ctrl ? mpcInputActive : mpcInputOff;
}

mpcInputMode_t mpcInputUpdate(const setpoint_t *setpoint, uint32_t tick, mpcCommand_t *cmd)
{
  mpcInputMode_t mode = input_src == 1 ? updateCrtp(setpoint, tick) : updateUart(tick);
  *cmd = command;
  return mode;
}

PARAM_GROUP_START(mpcin)
PARAM_ADD(PARAM_UINT8, input_src, &input_src)
PARAM_ADD(PARAM_UINT8, uart_ctrl, &uart_ctrl)
PARAM_ADD(PARAM_UINT16, timeout, &timeout_ms)
PARAM_ADD(PARAM_FLOAT, land_vel, &land_velocity)
PARAM_ADD(PARAM_UINT8, fallback, &fallback)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, rx, &rx_count)  // read by the Skybrush server after a session
PARAM_GROUP_STOP(mpcin)

// Names kept from controller_body_rate.c (the Skybrush server logs them). roll/pitch/yaw: the received rate setpoint
// in the type 20 convention (deg/s, pitch inverted), aff_*: type 21 angular acceleration feedforward (rad/s^2, FLU).
LOG_GROUP_START(ctrlMpc)
LOG_ADD(LOG_UINT8, state, &state)
LOG_ADD(LOG_UINT32, rx, &rx_count)
LOG_ADD(LOG_FLOAT, thrust, &command.thrust)
LOG_ADD(LOG_FLOAT, roll, &cmd_rate_deg.roll)
LOG_ADD(LOG_FLOAT, pitch, &cmd_rate_deg.pitch)
LOG_ADD(LOG_FLOAT, yaw, &cmd_rate_deg.yaw)
LOG_ADD(LOG_UINT8, type, &cmd_type)
LOG_ADD(LOG_FLOAT, aff_x, &command.angAccFF[0])
LOG_ADD(LOG_FLOAT, aff_y, &command.angAccFF[1])
LOG_ADD(LOG_FLOAT, aff_z, &command.angAccFF[2])
LOG_GROUP_STOP(ctrlMpc)
