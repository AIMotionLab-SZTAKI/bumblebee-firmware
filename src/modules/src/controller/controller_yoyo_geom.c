
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "controller_yoyo_geom.h"

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


#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

// Inertia matrix components
static float Ixx = 0.0025;
static float Izz = 0.0044;
static float drone_mass = 0.69;

static float kr_xy = 4.0f;
static float kv_xy = 2.0f;
static float kr_z = 10.0f;
static float kv_z = 5.0f;
static float kR = 0.6f;
static float kw = 0.15f;
static struct vec eR, ew, er, ev, M;
static float thrust;

static float pos_ref, vel_ref, acc_ref;
// static float actuatorThrust;
static float status_ext;  // status flag of external control input
static int fail_counter;  // number of subsequent invalid external control inputs 

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;

static uint8_t external_control = 0;

static bool enable_uart_comm = true;

void controllerYoyoGeomInit(void)
{
  // supplyVoltage = pmGetBatteryVoltage();
}

bool controllerYoyoGeomTest(void)
{
  return true;
}

void controllerYoyoGeom(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{

  control->controlMode = controlModeForceTorque;

  if (RATE_DO_EXECUTE(COMMUNICATION_RATE, tick)) {
    if (enable_uart_comm) {
      float dummy = 0.0f;
      sendDataUART("Y", &dummy, &dummy, &dummy, &dummy);
      uart_packet receiverPacket;
      if (receiveDataUART(&receiverPacket)) {
        if (receiverPacket.serviceType == YOYO_REF_PACKET) {
          handle_yoyo_ref_packet(&receiverPacket, &pos_ref, &vel_ref, &acc_ref, &status_ext);
          if (status_ext > 1.5f) { // invalid control input
            fail_counter += 1;
          } else if (status_ext > 0.5f) { // valid control input, status = 1
            fail_counter = 0;
          }
        }
      } else if (external_control && status_ext > 0.5f) { // communication timeout but still trying to control externally
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
        struct vec r1, r2, r3;
        struct mat33 Rd;
        struct vec setpointPos = mkvec(0.0f, 0.0f, pos_ref);
        struct vec setpointVel = mkvec(0.0f, 0.0f, vel_ref);
        struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
        struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);
        float yaw_des = 0.0f;

        // Position Error [m]
        er = vsub(statePos, setpointPos);

        // Velocity Error [m/s]
        ev = vsub(stateVel, setpointVel);

        struct vec target_thrust = vzero();
        target_thrust.x = -kr_xy*er.x - kv_xy*ev.x;
        target_thrust.y = -kr_xy*er.y - kv_xy*ev.y;
        target_thrust.z = -kr_z*er.z - kv_z*ev.z + drone_mass*(acc_ref + GRAVITY_MAGNITUDE);

        r3 = vnormalize(target_thrust);
        r2 = vnormalize(vcross(r3,mkvec(cosf(yaw_des), sinf(yaw_des), 0)));
        r1 = vcross(r2, r3);
        Rd = mcolumns(r1, r2, r3);
  
        struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);

        struct mat33 R = quat2rotmat(q);

        struct mat33 eR1 = mmul(mtranspose(Rd),R);
        struct mat33 eR2 = mmul(mtranspose(R),Rd);

        //Attitude tracking error
        struct mat33 eRm = msub(eR1,eR2);
        eR.x = 0.5f*eRm.m[2][1];
        eR.y = 0.5f*eRm.m[0][2];
        eR.z = 0.5f*eRm.m[1][0];
        
        struct vec w = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z));
        //Attitude rate error [rad/s]
        ew = w;

        //Torque component relating to angular acceleration
        struct vec cross = vcross(w, mkvec(Ixx*w.x, Ixx*w.y, Izz*w.z));
        //Torque in each direction [Nm] Uncapped
        M.x = cross.x - kR * eR.x - kw * ew.x;
        M.y = cross.y - kR * eR.y - kw * ew.y;
        M.z = cross.z - kR * eR.z - kw * ew.z;

        //Thrust pointing in crazyflie body frame Z direction  
        thrust = vdot(target_thrust, mcolumn(R,2));
    }

    if (external_control && status_ext > 0.5f && thrust > 0) {
        control->thrustSi = thrust;
    } else {
        control->thrustSi = 0;
    }

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
  }

}


PARAM_GROUP_START(yoyo)
PARAM_ADD(PARAM_UINT8, external_control, &external_control)
PARAM_ADD(PARAM_FLOAT, kr_xy, &kr_xy)
PARAM_ADD(PARAM_FLOAT, kv_xy, &kv_xy)
PARAM_ADD(PARAM_FLOAT, kr_z, &kr_z)
PARAM_ADD(PARAM_FLOAT, kv_z, &kv_z)
PARAM_ADD(PARAM_FLOAT, kR, &kR)
PARAM_ADD(PARAM_FLOAT, kw, &kw)
PARAM_ADD(PARAM_FLOAT, drone_mass, &drone_mass)
PARAM_GROUP_STOP(yoyo)

LOG_GROUP_START(yoyo)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
LOG_ADD(LOG_FLOAT, pos_ref, &pos_ref)
LOG_ADD(LOG_FLOAT, vel_ref, &vel_ref)
LOG_ADD(LOG_FLOAT, acc_ref, &acc_ref)
LOG_ADD(LOG_FLOAT, status_ext, &status_ext)
LOG_GROUP_STOP(yoyo)