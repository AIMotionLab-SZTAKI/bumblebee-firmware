
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "controller_body_rate.h"

#include "log.h"
#include "param.h"
#include "math3d.h"

// Communication module include
#include "communication.h"

// Stabilizer include to be able to switch into emergency mode and get motor PWMs
#include "stabilizer.h"

// Power management to get battery voltage
#include "pm.h"

// Gravity
#include "physicalConstants.h"

// Set center of mass shift externally
#include "power_distribution.h"

#include "motors.h"

// Radio (CRTP) input: tick counts, handing over to the high-level commander, fallback controllers
#include "FreeRTOS.h"
#include "task.h"
#include "commander.h"
#include "crtp_commander_high_level.h"
#include "supervisor.h"
#include "controller.h"
#include "controller_geom.h"
#include "controller_pid.h"


#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

// Inertia matrix components
static float Ixx = 0.0015;
static float Izz = 0.00277;

static float dt = ATTITUDE_UPDATE_DT;

static float kw = 0.15;
static float kw_yaw = 0.15;

//static float ctrl_thrust = 0;

static struct vec ew, wd, prev_wd, M;

//static float drone_mass = 0.635;
//static float measured_mass = 0;



static attitude_t rateDesired_ext;
// static float actuatorThrust;
static float thrust_ext;
static float status_ext;  // status flag of external control input
static int fail_counter;  // number of subsequent invalid external control inputs 
static float com_shift_x = 0.0f;
static float com_shift_y = 0.0f;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

static uint8_t external_control = 0;

static bool enable_uart_comm = true;

// Where the external body rate + thrust commands come from.
// 0: UART (Raspberry Pi with its own radio), 1: CRTP setpoints of type mpc, streamed through the Skybrush server.
static uint8_t input_src = 0;

// CRTP input only. While no MPC command is coming in, the drone is flown by the fallback controller, i.e. it can
// take off, hover and land with the high-level commander. When MPC commands arrive, they take over; when they stop
// with a notify-setpoints-stop (the server does this at the end of a session), the high-level commander takes over
// again. If they stop without one (radio or server lost), or the MPC keeps marking its commands as invalid, the MPC
// session is aborted: the high-level commander lands the drone, and MPC setpoints are ignored until the drone is on
// the ground and no MPC setpoint arrived for MPC_LOCKOUT_RELEASE_MS.
static uint16_t mpc_timeout_ms = 100;      // abort if no new MPC command for this long
static float mpc_land_velocity = 0.3f;     // m/s, landing after an abort
static uint8_t fallback = ControllerTypeGeom;  // ControllerTypeGeom or ControllerTypePID
#define MPC_MAX_FAILS 20                   // consecutive invalid MPC commands before aborting
#define MPC_LOCKOUT_RELEASE_MS 1000

typedef enum {
  mpcIdle = 0,     // no MPC command, fallback controller
  mpcActive = 1,   // following MPC commands
  mpcAborted = 2,  // MPC setpoints locked out
} mpcState_t;

static uint8_t mpc_state = mpcIdle;       // mpcState_t, also written by the CRTP task (atomic on the M4)
static uint32_t mpc_last_arrival;  // tick of the last MPC setpoint received by the radio, accepted or not
static uint32_t mpc_last_timestamp;         // timestamp of the last MPC setpoint consumed, to detect new ones
static uint32_t mpc_rx_count;               // number of MPC setpoints consumed

// static float batt_comp_a = -0.1245;  // with kR = 0.6: -0.1205
// static float batt_comp_b = 2.768;  // with kR = 0.6: 2.6802

// static float supplyVoltage;

// ang_vel = a * pwm + b
//static float pwmToAngVelA = 0.065769f;
//static float pwmToAngVelB = -131.538;

// thrust = c * signed_sum(ang_vel^2)
//static float angVelToThrust = 9.3945e-7f;

void controllerBodyRateInit(void)
{
  // supplyVoltage = pmGetBatteryVoltage();
  controllerGeomInit();
  controllerPidInit();
  prev_wd = mkvec(NAN, NAN, NAN);  // the derivative of the desired rate is not yet initialized
}

bool controllerBodyRateTest(void)
{
  return true;
}

// Body rate loop: computes the torque M that tracks the desired rates [deg/s]
static void computeTorques(const sensorData_t *sensors, const attitude_t *rateDesired)
{
    // TODO: Investigate possibility to subtract gyro drift.
    wd = mkvec(radians(rateDesired->roll), -radians(rateDesired->pitch), radians(rateDesired->yaw));
    struct vec w = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z));
    //Attitude rate error [rad/s]
    ew = vsub(w, wd);

    struct vec beta_desired = vzero();
    if (prev_wd.x == prev_wd.x) { //d part initialized
        beta_desired = vsub(wd, prev_wd);
        beta_desired.x = beta_desired.x/dt;
        beta_desired.y = beta_desired.y/dt;
        beta_desired.z = beta_desired.z/dt;
    }
    prev_wd = wd;

    struct vec diff_part = vsub(vcross(w, wd), beta_desired);
    diff_part.x = Ixx*diff_part.x;
    diff_part.y = Ixx*diff_part.y;
    diff_part.z = Izz*diff_part.z;

    //Torque component relating to angular acceleration
    // struct vec cross = vcross(w, mkvec(Ixx*w.x, Ixx*w.x, Izz*w.z));
    //Torque in each direction [Nm] Uncapped
    // M.x = cross.x - kw * ew.x - diff_part.x;
    // M.y = cross.y - kw * ew.y - diff_part.y;
    // M.z = cross.z - kw * ew.z - diff_part.z;

    M.x = - kw * ew.x;
    M.y = - kw * ew.y;
    M.z = - kw * ew.z;


//   attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
//                          rateDesired_ext.roll, rateDesired_ext.pitch, rateDesired_ext.yaw);
//   attitudeControllerGetActuatorOutput(&control->roll,
//                                     &control->pitch,
//                                     &control->yaw);
}

// Writes thrust [N] and the torques of the last computeTorques() call into control, and updates the log variables
static void applyOutput(control_t *control, const sensorData_t *sensors, float thrust)
{
    control->controlMode = controlModeForceTorque;
    control->thrustSi = thrust;

    if (control->thrustSi > 0) {
        control->torqueX = M.x;
        control->torqueY = M.y;
        control->torqueZ = M.z;

    } else {
        control->torqueX = 0;
        control->torqueY = 0;
        control->torqueZ = 0;
    }

    cmd_thrust = control->thrustSi;
    cmd_roll = control->torqueX;
    cmd_pitch = control->torqueY;
    cmd_yaw = control->torqueZ;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->acc.z;
}

bool controllerBodyRateAcceptMpcSetpoint(void)
{
  // Runs in the CRTP task, for every MPC setpoint that arrives
  uint32_t now = xTaskGetTickCount();
  if (mpc_state == mpcAborted && now - mpc_last_arrival > M2T(MPC_LOCKOUT_RELEASE_MS) && !supervisorIsFlying()) {
    mpc_state = mpcIdle;
  }
  mpc_last_arrival = now;
  return mpc_state != mpcAborted;
}

static void runFallback(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors,
                        const state_t *state, const uint32_t tick)
{
  if (fallback == ControllerTypePID) {
    controllerPid(control, setpoint, sensors, state, tick);
  } else {
    controllerGeom(control, setpoint, sensors, state, tick);
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

static void abortMpc(void)
{
  mpc_state = mpcAborted;
  // Hand over to the high-level commander, starting from the current state, and land
  commanderRelaxPriority();
  if (supervisorIsFlying()) {
    crtpCommanderHighLevelLandWithVelocity(0.0f, mpc_land_velocity, false);
  }
}

static void controllerBodyRateCrtp(control_t *control, const setpoint_t *setpoint,
                                   const sensorData_t *sensors,
                                   const state_t *state,
                                   const uint32_t tick)
{
  if (!setpoint->mpc.active) {
    // High-level commander (takeoff, hover, land...), or nothing at all: the fallback controller flies
    if (mpc_state == mpcActive) {
      // The MPC handed over cleanly (notify-setpoints-stop)
      mpc_state = mpcIdle;
      resetFallback();
    }
    runFallback(control, setpoint, sensors, state, tick);
    return;
  }

  if (mpc_state == mpcIdle) {
    // First command of an MPC session
    mpc_state = mpcActive;
    fail_counter = 0;
    prev_wd = mkvec(NAN, NAN, NAN);
  }

  if (mpc_state == mpcActive) {
    if (setpoint->timestamp != mpc_last_timestamp) {
      mpc_last_timestamp = setpoint->timestamp;
      mpc_rx_count++;
      thrust_ext = setpoint->thrust;
      rateDesired_ext = setpoint->attitudeRate;
      status_ext = setpoint->mpc.status;
      com_shift_x = setpoint->mpc.comShiftX;
      com_shift_y = setpoint->mpc.comShiftY;
      setComShift(com_shift_x, com_shift_y);
      fail_counter = setpoint->mpc.status ? fail_counter + 1 : 0;
    }
    if (xTaskGetTickCount() - setpoint->timestamp > M2T(mpc_timeout_ms) || fail_counter >= MPC_MAX_FAILS) {
      abortMpc();
    }
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    if (mpc_state == mpcActive) {
      computeTorques(sensors, &rateDesired_ext);
      applyOutput(control, sensors, thrust_ext);
    } else {
      // Aborted, but the high-level commander's first setpoint has not yet arrived: hold level-ish with the last
      // thrust for these few ticks, or stop the motors if we are on the ground anyway
      const attitude_t zeroRates = {0};
      computeTorques(sensors, &zeroRates);
      applyOutput(control, sensors, supervisorIsFlying() ? thrust_ext : 0.0f);
    }
  }
}

void controllerBodyRate(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (input_src == 1) {
    controllerBodyRateCrtp(control, setpoint, sensors, state, tick);
    return;
  }

  control->controlMode = controlModeForceTorque;

  if (RATE_DO_EXECUTE(COMMUNICATION_RATE, tick)) {
    if (enable_uart_comm) {
      /*sendDataUART("C", &actuatorThrust, state);
      float dummy1 = 12.34;
      float dummy2 = 345.12;
      sendDataUART("T", &actuatorThrust, &dummy1, &dummy2);
      */
      uint16_t motorRPMs[4];
      for (int i = 0; i < 4; i++) {
        motorRPMs[i] = motorsGetRPM(i);
      }
      sendDataUART("F", motorRPMs, motorRPMs + 1, motorRPMs + 2, motorRPMs + 3);
      uart_packet receiverPacket;
      if (receiveDataUART(&receiverPacket)) {
        if (receiverPacket.serviceType == CONTROL_PACKET) {
          handle_control_packet(&receiverPacket, &thrust_ext, &rateDesired_ext.roll, &rateDesired_ext.pitch, &rateDesired_ext.yaw);
        } else if (receiverPacket.serviceType == FORWARDED_CONTROL_PACKET) {
          handle_forwarded_packet(&receiverPacket, &thrust_ext, &rateDesired_ext.roll, &rateDesired_ext.pitch, &rateDesired_ext.yaw, &status_ext,
                                  &com_shift_x, &com_shift_y);
          setComShift(com_shift_x, com_shift_y);
          if (status_ext > 0.5f) { // invalid control input
            fail_counter += 1;
          } else {
            fail_counter = 0;
          }
        }
        // convert thrust from N to PWM
        // supplyVoltage =  0.99f * supplyVoltage + 0.01f * pmGetBatteryVoltage();  
        // float mass_ratio = batt_comp_a * supplyVoltage + batt_comp_b;
        //float thrust_battery_corrected = thrust_ext * mass_ratio;
        //thrust_ext = getThrustPwm(thrust_battery_corrected);
        // thrust_ext *= mass_ratio;
      } else if (external_control) { // communication timeout but still trying to control externally
        fail_counter += 11;
      }

      if (fail_counter >= 20) {
        motorsStop();  // switching to emergency mode
        // maybe later we could just disable uart communication and find a safe setpoint for PID
      }
    }
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    if (external_control) {
      computeTorques(sensors, &rateDesired_ext);
    }
    applyOutput(control, sensors, external_control ? thrust_ext : 0);
  }

}

float getThrust(void) {
  return thrust_ext;
}

float getStatus(void) {
  return status_ext;
}

void getRateDesired(attitude_t *rate) {
  *rate = rateDesired_ext;
}

PARAM_GROUP_START(bodyrate)
PARAM_ADD(PARAM_UINT8, external_control, &external_control)
PARAM_ADD(PARAM_FLOAT, kw, &kw)
PARAM_ADD(PARAM_FLOAT, kw_yaw, &kw_yaw)
PARAM_ADD(PARAM_UINT8, input_src, &input_src)
PARAM_ADD(PARAM_UINT16, mpc_timeout, &mpc_timeout_ms)
PARAM_ADD(PARAM_FLOAT, mpc_land_vel, &mpc_land_velocity)
PARAM_ADD(PARAM_UINT8, fallback, &fallback)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, mpc_rx, &mpc_rx_count)  // read by the Skybrush server after a session
PARAM_GROUP_STOP(bodyrate)

LOG_GROUP_START(ctrlBR)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_GROUP_STOP(ctrlBR)

LOG_GROUP_START(ctrlMpc)
LOG_ADD(LOG_UINT8, state, &mpc_state)
LOG_ADD(LOG_UINT32, rx, &mpc_rx_count)
LOG_ADD(LOG_FLOAT, thrust, &thrust_ext)
LOG_ADD(LOG_FLOAT, roll, &rateDesired_ext.roll)
LOG_ADD(LOG_FLOAT, pitch, &rateDesired_ext.pitch)
LOG_ADD(LOG_FLOAT, yaw, &rateDesired_ext.yaw)
LOG_GROUP_STOP(ctrlMpc)