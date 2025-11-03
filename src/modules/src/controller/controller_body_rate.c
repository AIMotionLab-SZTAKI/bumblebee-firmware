
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

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)
#define COMMUNICATION_RATE RATE_100_HZ

// Inertia matrix components
static float Ixx = 0.0015;
static float Izz = 0.00277;

static float dt = ATTITUDE_UPDATE_DT;

static float kw = 0.15;

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

static float batt_comp_a = -0.1245;  // with kR = 0.6: -0.1205
static float batt_comp_b = 2.768;  // with kR = 0.6: 2.6802

static float supplyVoltage;

// ang_vel = a * pwm + b
//static float pwmToAngVelA = 0.065769f;
//static float pwmToAngVelB = -131.538;

// thrust = c * signed_sum(ang_vel^2)
//static float angVelToThrust = 9.3945e-7f;

void controllerBodyRateInit(void)
{
  supplyVoltage = pmGetBatteryVoltage();
}

bool controllerBodyRateTest(void)
{
  return true;
}

void controllerBodyRate(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{

  control->controlMode = controlModeForceTorque;

  if (RATE_DO_EXECUTE(COMMUNICATION_RATE, tick)) {
    if (enable_uart_comm) {
      /*sendDataUART("C", &actuatorThrust, state);
      float dummy1 = 12.34;
      float dummy2 = 345.12;
      sendDataUART("T", &actuatorThrust, &dummy1, &dummy2);
      */
      motors_thrust_pwm_t pwm = getMotorPwm();
      sendDataUART("F", &pwm.motors.m1, &pwm.motors.m2, &pwm.motors.m3, &pwm.motors.m4);
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
        supplyVoltage =  0.99f * supplyVoltage + 0.01f * pmGetBatteryVoltage();  
        float mass_ratio = batt_comp_a * supplyVoltage + batt_comp_b;
        //float thrust_battery_corrected = thrust_ext * mass_ratio;
        //thrust_ext = getThrustPwm(thrust_battery_corrected);
        thrust_ext *= mass_ratio;
      } else if (external_control) { // communication timeout but still trying to control externally
        fail_counter += 2;
      }

      if (fail_counter >= 20) {
        stabilizerSetEmergencyStop();  // switching to emergency mode
        // maybe later we could just disable uart communication and find a safe setpoint for PID
      }
    }
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {

    // TODO: Investigate possibility to subtract gyro drift.
    if (external_control) {
        wd = mkvec(radians(rateDesired_ext.roll), -radians(rateDesired_ext.pitch), radians(rateDesired_ext.yaw));
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
        struct vec cross = vcross(w, mkvec(Ixx*w.x, Ixx*w.x, Izz*w.z));
        //Torque in each direction [Nm] Uncapped
        M.x = cross.x - kw * ew.x - diff_part.x;
        M.y = cross.y - kw * ew.y - diff_part.y;
        M.z = cross.z - kw * ew.z - diff_part.z;

    
    //   attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
    //                          rateDesired_ext.roll, rateDesired_ext.pitch, rateDesired_ext.yaw);
    //   attitudeControllerGetActuatorOutput(&control->roll,
    //                                     &control->pitch,
    //                                     &control->yaw);
    }

    if (external_control)  control->thrustSi = thrust_ext; else control->thrustSi = 0;

    if (control->thrustSi > 0) {
        control->torqueX = M.x;
        control->torqueY = M.y;
        control->torqueZ = M.z;

    } else {
        control->torqueX = 0;
        control->torqueY = 0;
        control->torqueZ = 0;
    }

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->acc.z;
  }

}

PARAM_GROUP_START(bodyrate)
PARAM_ADD(PARAM_UINT8, external_control, &external_control)
PARAM_GROUP_STOP(bodyrate)