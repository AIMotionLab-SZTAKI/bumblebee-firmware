/*
 *
 * Copyright (c) 2019 Ewoud Smeur and Andre Luis Ogando Paraense
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * This control algorithm is the Incremental Nonlinear Dynamic Inversion (INDI)
 * controller.
 *
 * This is an implementation of the publication in the
 * journal of Control Guidance and Dynamics: Adaptive Incremental Nonlinear
 * Dynamic Inversion for Attitude Control of Micro Aerial Vehicles
 * http://arc.aiaa.org/doi/pdf/10.2514/1.G001490
 */

 #include "controller_body_rate.h"

#include "controller_indi.h"
#include "math3d.h"
#include "log.h"
#include "param.h"

// Communication module include
#include "communication.h"

// Power management to get battery voltage
#include "pm.h"

// Gravity
#include "physicalConstants.h"

// Set center of mass shift externally
#include "power_distribution.h"

#include "motors.h"

#include "filter.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

// Inertia matrix components
static float Ixx = 0.0045;
static float Izz = 0.005;

static float dt = ATTITUDE_UPDATE_DT;

static float kw = 25.0f;

static float armLength = 0.085f; // m;

// thrust = c * signed_sum(ang_vel^2)
static float angVelToThrust = 9.3945e-7f;  // old value: 9.3945e-7f; new value: 8.6584e-7f;

// torque = a/c * signed_sum(thrust) + b/c
static float thrustToTorqueA = 5.5939e-7f;
static float thrustToTorqueB = -0.4785f;

// main variables
static struct vec body_rate, angular_acc, desired_angular_acc, desired_body_torque, filtered_body_torque, prev_wd, computed_body_torque;
static float rotor_acc[4];
static uint16_t motorRPMs[4];
static float filter_cutoff = 10.0f;  // Hz
static Butterworth2LowPass body_rate_filters[3];
static Butterworth2LowPass rotor_speed_filters[4];

static attitude_t rateDesired_ext;
// static float actuatorThrust;
static float thrust_ext;
static float status_ext;  // status flag of external control input
static int fail_counter;  // number of subsequent invalid external control inputs 
static float com_shift_x = 0.0f;
static float com_shift_y = 0.0f;

// log variables
static float cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw, r_roll, r_pitch, r_yaw, accelz;

// helpers
static uint8_t external_control = 0;

static bool enable_uart_comm = true;

static float batt_comp_a = -0.1245;  // with kR = 0.6: -0.1205
static float batt_comp_b = 2.768;  // with kR = 0.6: 2.6802

static float supplyVoltage;

void indi_init_filters(void)
{
	// tau = 1/(2*pi*Fc)
	float tau = 1.0f / (2.0f * M_PI_F * filter_cutoff);

	// Filtering of gyroscope and actuators
	for (int8_t i = 0; i < 3; i++) {
		init_butterworth_2_low_pass(&body_rate_filters[i], tau, dt, 0.0f);
	}
	// Initialize rotor speed filters
	for (int8_t i = 0; i < 4; i++) {
		init_butterworth_2_low_pass(&rotor_speed_filters[i], tau, dt, 0.0f);
	}
}

static inline void filter_body_rates(Butterworth2LowPass *filter, struct vec *new_values)
{
	update_butterworth_2_low_pass(&filter[0], new_values->x);
	update_butterworth_2_low_pass(&filter[1], new_values->y);
	update_butterworth_2_low_pass(&filter[2], new_values->z);
}

static inline void filter_rotor_speeds(Butterworth2LowPass *filter, float *new_values)
{
	for (int8_t i = 0; i < 4; i++) {
		update_butterworth_2_low_pass(&filter[i], new_values[i]);
	}
}

static inline void finite_difference_from_body_rate_filter(struct vec *output, Butterworth2LowPass *filter)
{
	output->x = (filter[0].o[0] - filter[0].o[1]) / dt;
	output->y = (filter[1].o[0] - filter[1].o[1]) / dt;
	output->z = (filter[2].o[0] - filter[2].o[1]) / dt;
}

static inline void finite_difference_from_rotor_speed_filter(float *output, Butterworth2LowPass *filter)
{
	for (int8_t i = 0; i < 4; i++) {
		output[i] = (filter[i].o[0] - filter[i].o[1]) / dt;
	}
}

void controllerINDIInit(void)
{
	indi_init_filters();
    supplyVoltage = pmGetBatteryVoltage();
}

bool controllerINDITest(void)
{
	return true;
}

void controllerINDI(control_t *control, const setpoint_t *setpoint,
	const sensorData_t *sensors,
	const state_t *state,
	const stabilizerStep_t stabilizerStep)
{
  control->controlMode = controlModeForceTorque;

  if (RATE_DO_EXECUTE(COMMUNICATION_RATE, stabilizerStep)) {
    if (enable_uart_comm) {
      /*sendDataUART("C", &actuatorThrust, state);
      float dummy1 = 12.34;
      float dummy2 = 345.12;
      sendDataUART("T", &actuatorThrust, &dummy1, &dummy2);
      */
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
        supplyVoltage =  0.99f * supplyVoltage + 0.01f * pmGetBatteryVoltage();  
        float mass_ratio = batt_comp_a * supplyVoltage + batt_comp_b;
        //float thrust_battery_corrected = thrust_ext * mass_ratio;
        //thrust_ext = getThrustPwm(thrust_battery_corrected);
        thrust_ext *= mass_ratio;
      } else if (external_control) { // communication timeout but still trying to control externally
        fail_counter += 11;
      }

      if (fail_counter >= 20) {
        motorsStop();  // switching to emergency mode
        // maybe later we could just disable uart communication and find a safe setpoint for PID
      }
    }
	// thrust_ext = getThrust();
    // getRateDesired(&rateDesired_ext);
  }


	if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
	  if (external_control) {
		/*
		* 1 - Update the gyro filter with the new measurements.
		*/

		body_rate = mkvec(radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z));
		filter_body_rates(body_rate_filters, &body_rate);

		/*
		 * 2 - Calculate the derivative with finite difference.
		 */

		finite_difference_from_body_rate_filter(&angular_acc, body_rate_filters);

		/*
		 * 3 - same filter on the rotor speeds and derivative computation with finite difference
		 */
		for (int i = 0; i < 4; i++) {
			motorRPMs[i] = motorsGetRPM(i);
			if (motorRPMs[i] > 65000) {
				motorRPMs[i] = 0; // motor is not running
			}
		}
		float motor_speeds[4];
		for (int i = 0; i < 4; i++) {
			motor_speeds[i] = (float)motorRPMs[i] * 2.0f * M_PI_F / 60.0f; // convert RPM to rad/s
		}
		filter_rotor_speeds(rotor_speed_filters, motor_speeds);
		finite_difference_from_rotor_speed_filter(rotor_acc, rotor_speed_filters);

		/*
		 * 4 - Calculate the desired angular acceleration from (33)
		 */
		struct vec beta_desired = vzero();
		struct vec wd = mkvec(radians(rateDesired_ext.roll), -radians(rateDesired_ext.pitch), radians(rateDesired_ext.yaw));
        if (prev_wd.x == prev_wd.x) { //d part initialized
            beta_desired = vscl(1.0f/dt, vsub(wd, prev_wd));
        }
        prev_wd = wd;

		// desired_angular_acc.x = kw * (wd.x - body_rate_filters[0].o[0]) + beta_desired.x;
		// desired_angular_acc.y = kw * (wd.y - body_rate_filters[1].o[0]) + beta_desired.y;
		// desired_angular_acc.z = kw * (wd.z - body_rate_filters[2].o[0]) + beta_desired.z;
		desired_angular_acc.x = kw * (wd.x - body_rate_filters[0].o[0]);
		desired_angular_acc.y = kw * (wd.y - body_rate_filters[1].o[0]);
		desired_angular_acc.z = kw * (wd.z - body_rate_filters[2].o[0]);

		/*
		 * 5. Calculate the control moment from (34)
		 */
		float c = armLength * angVelToThrust;
		float w_f_2[4];
		for (int i = 0; i < 4; i++) {
			w_f_2[i] = rotor_speed_filters[i].o[0] * rotor_speed_filters[i].o[0];
		}
		filtered_body_torque.x = -c * w_f_2[0] - c * w_f_2[1] + c * w_f_2[2] + c * w_f_2[3];
		filtered_body_torque.y = -c * w_f_2[0] + c * w_f_2[1] + c * w_f_2[2] - c * w_f_2[3];
		float signed_squared_sum = - w_f_2[0] + w_f_2[1] - w_f_2[2] + w_f_2[3];
		if (signed_squared_sum > - thrustToTorqueB / thrustToTorqueA) {
			filtered_body_torque.z = thrustToTorqueA * signed_squared_sum + thrustToTorqueB;
		} else if (signed_squared_sum < thrustToTorqueB / thrustToTorqueA) {
			filtered_body_torque.z = thrustToTorqueA * signed_squared_sum - thrustToTorqueB;
		} else {
			filtered_body_torque.z = 0.0f;
		}

		computed_body_torque.x = Ixx * angular_acc.x;
		computed_body_torque.y = Ixx * angular_acc.y;
		computed_body_torque.z = Izz * angular_acc.z;

		desired_body_torque.x = filtered_body_torque.x + Ixx * (desired_angular_acc.x - angular_acc.x);
		desired_body_torque.y = filtered_body_torque.y + Ixx * (desired_angular_acc.y - angular_acc.y);
		desired_body_torque.z = filtered_body_torque.z + Izz * (desired_angular_acc.z - angular_acc.z);
	  }
	  
    }

    if (external_control)  control->thrustSi = thrust_ext; else control->thrustSi = 0;

    if (control->thrustSi > 0) {
        control->torqueX = desired_body_torque.x;
        control->torqueY = desired_body_torque.y;
        control->torqueZ = desired_body_torque.z;

    } else {
        control->torqueX = 0;
        control->torqueY = 0;
        control->torqueZ = 0;
    }

    cmd_thrust = thrust_ext;
    cmd_roll = desired_body_torque.x;
    cmd_pitch = desired_body_torque.y;
    cmd_yaw = desired_body_torque.z;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->acc.z;

}

PARAM_GROUP_START(indi)
PARAM_ADD(PARAM_UINT8, external_control, &external_control)
PARAM_ADD(PARAM_FLOAT, kw, &kw)
PARAM_ADD(PARAM_FLOAT, fc, &filter_cutoff)
PARAM_ADD(PARAM_FLOAT, Ixx, &Ixx)
PARAM_ADD(PARAM_FLOAT, Izz, &Izz)
PARAM_GROUP_STOP(indi)


LOG_GROUP_START(ctrlINDI)

LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, m1f, &rotor_speed_filters[0].o[0])
LOG_ADD(LOG_FLOAT, m2f, &rotor_speed_filters[1].o[0])
LOG_ADD(LOG_FLOAT, m3f, &rotor_speed_filters[2].o[0])
LOG_ADD(LOG_FLOAT, m4f, &rotor_speed_filters[3].o[0])
LOG_ADD(LOG_FLOAT, ctx, &computed_body_torque.x)
LOG_ADD(LOG_FLOAT, cty, &computed_body_torque.y)
LOG_ADD(LOG_FLOAT, ctz, &computed_body_torque.z)
LOG_ADD(LOG_FLOAT, ftx, &filtered_body_torque.x)
LOG_ADD(LOG_FLOAT, fty, &filtered_body_torque.y)
LOG_ADD(LOG_FLOAT, ftz, &filtered_body_torque.z)


LOG_GROUP_STOP(ctrlINDI)
