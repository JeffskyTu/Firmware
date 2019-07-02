/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "mc_att_control.hpp"

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

#define MIN_TAKEOFF_THRUST    0.2f
#define TPA_RATE_LOWER_LIMIT 0.05f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

/**
 * Used for Model Based Control.
 * Model Based Controller outputs commands of thrust and torques around three-axis,
 * while the commands send by px4 to low-level mixer and motors are normalized values in 0~1(thrust) or -1~1(torque).
 * By converting F&T commands to normalized commands, we can implement Model Based Control methods readily,
 * using existing mixer and saturation limiter.
 */
/* Model of QAV250 */
#define HALF_LENGTH		0.101f
#define HALF_WIDTH		0.080f

#define C_M				0.0097f		// C_M = drag torque / motor thrust.
//#define THRUST_FACTOR	0.0f
#define THRUST_FACTOR	0.4785f

#define I_XX			3e-3f
#define I_YY			2.7e-3f
#define I_ZZ			6.0e-3f
// Parameters of attitude(acc) estimator
#define OMEGA_ATT		50.0f
#define ZETA_ATT		0.8f


using namespace matrix;

void MulticopterAttitudeControl::print_vector3(const char *name, Vector3f &vector3, bool print_time){
	if(_num_update%200 == 0){
		if(print_time){
			PX4_INFO("------- time = %f s -------", (double)(hrt_absolute_time()*1e-6f));
		}
		PX4_INFO("%s = %8.6f, %8.6f, %8.6f", name, (double)vector3(0), (double)vector3(1), (double)vector3(2));
	}
}

int MulticopterAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude and rate controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) or rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has two loops: a P loop for angular error and a PID loop for angular rate error.

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

### Implementation
To reduce control latency, the module directly polls on the gyro topic published by the IMU driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

MulticopterAttitudeControl::MulticopterAttitudeControl() :
	ModuleParams(nullptr),
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control")),
	_lp_filters_d{
	{initial_update_rate_hz, 50.f},
	{initial_update_rate_hz, 50.f},
	{initial_update_rate_hz, 50.f}} // will be initialized correctly when params are loaded
{
	for (uint8_t i = 0; i < MAX_GYRO_COUNT; i++) {
		_sensor_gyro_sub[i] = -1;
	}

	_vehicle_status.is_rotary_wing = true;

	/* initialize quaternions in messages to be valid */
	_v_att.q[0] = 1.f;
	_v_att_sp.q_d[0] = 1.f;

	_rates_prev.zero();
	_rates_prev_filtered.zero();
	_rates_sp.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	_x1.zero();
	_x2.zero();
//	_x1_dot.zero();
//	_x2_dot.zero();
	_torque_hat.zero();
	_torque_motor.zero();
	_torque_dist.zero();
	_torque_dist_last.zero();
	_x1_trq.zero();
	_x2_trq.zero();

	/* initialize thermal corrections as we might not immediately get a topic update (only non-zero values) */
	for (unsigned i = 0; i < 3; i++) {
		// used scale factors to unity
		_sensor_correction.gyro_scale_0[i] = 1.0f;
		_sensor_correction.gyro_scale_1[i] = 1.0f;
		_sensor_correction.gyro_scale_2[i] = 1.0f;
	}

	parameters_updated();
}

void
MulticopterAttitudeControl::parameters_updated()
{
	/* Store some of the parameters in a more convenient way & precompute often-used values */

	/* roll gains */
	_attitude_p(0) = _roll_p.get();
	_rate_p(0) = _roll_rate_p.get();
	_rate_i(0) = _roll_rate_i.get();
	_rate_int_lim(0) = _roll_rate_integ_lim.get();
	_rate_d(0) = _roll_rate_d.get();
	_rate_ff(0) = _roll_rate_ff.get();

	/* pitch gains */
	_attitude_p(1) = _pitch_p.get();
	_rate_p(1) = _pitch_rate_p.get();
	_rate_i(1) = _pitch_rate_i.get();
	_rate_int_lim(1) = _pitch_rate_integ_lim.get();
	_rate_d(1) = _pitch_rate_d.get();
	_rate_ff(1) = _pitch_rate_ff.get();

	/* yaw gains */
	_attitude_p(2) = _yaw_p.get();
	_rate_p(2) = _yaw_rate_p.get();
	_rate_i(2) = _yaw_rate_i.get();
	_rate_int_lim(2) = _yaw_rate_integ_lim.get();
	_rate_d(2) = _yaw_rate_d.get();
	_rate_ff(2) = _yaw_rate_ff.get();

	if (fabsf(_lp_filters_d[0].get_cutoff_freq() - _d_term_cutoff_freq.get()) > 0.01f) {
		_lp_filters_d[0].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
		_lp_filters_d[1].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
		_lp_filters_d[2].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
		_lp_filters_d[0].reset(_rates_prev(0));
		_lp_filters_d[1].reset(_rates_prev(1));
		_lp_filters_d[2].reset(_rates_prev(2));
	}

	/* angular rate limits */
	_mc_rate_max(0) = math::radians(_roll_rate_max.get());
	_mc_rate_max(1) = math::radians(_pitch_rate_max.get());
	_mc_rate_max(2) = math::radians(_yaw_rate_max.get());

	/* auto angular rate limits */
	_auto_rate_max(0) = math::radians(_roll_rate_max.get());
	_auto_rate_max(1) = math::radians(_pitch_rate_max.get());
	_auto_rate_max(2) = math::radians(_yaw_auto_max.get());

	/* manual rate control acro mode rate limits and expo */
	_acro_rate_max(0) = math::radians(_acro_roll_max.get());
	_acro_rate_max(1) = math::radians(_acro_pitch_max.get());
	_acro_rate_max(2) = math::radians(_acro_yaw_max.get());

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	/* get transformation matrix from sensor/board to body frame */
	_board_rotation = get_rot_matrix((enum Rotation)_board_rotation_param.get());

	/* fine tune the rotation */
	Dcmf board_rotation_offset(Eulerf(
			M_DEG_TO_RAD_F * _board_offset_x.get(),
			M_DEG_TO_RAD_F * _board_offset_y.get(),
			M_DEG_TO_RAD_F * _board_offset_z.get()));
	_board_rotation = board_rotation_offset * _board_rotation;

	/* esc parameters */
	_pwm_min_value = _pwm_min.get();
	_pwm_max_value = _pwm_max.get();

	param_get(param_find("MC_OMEGA_ATT"), &_omega_att);
	param_get(param_find("MC_ZETA_ATT"), &_zeta_att);
	param_get(param_find("MC_HALF_LENGTH"), &_half_length);
	param_get(param_find("MC_HALF_WIDTH"), &_half_width);
	param_get(param_find("MC_C_M"), &_C_M);
	param_get(param_find("MC_I_XX"), &_I_XX);
	param_get(param_find("MC_I_YY"), &_I_YY);
	param_get(param_find("MC_I_ZZ"), &_I_ZZ);
	param_get(param_find("MC_THRUST_FACTOR"), &_thrust_factor);
	param_get(param_find("MC_NDRC_ATT_EN"), &_ndrc_att_enable);
	param_get(param_find("MC_NDI_ENABLE"), &_ndi_enable);

}

void
MulticopterAttitudeControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		updateParams();
		parameters_updated();
	}
}

void
MulticopterAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
MulticopterAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (_rates_sp_id == nullptr) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(mc_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}

void
MulticopterAttitudeControl::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_motor_limits_sub, &updated);

	if (updated) {
		multirotor_motor_limits_s motor_limits = {};
		orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &motor_limits);

		_saturation_status.value = motor_limits.saturation_status;
	}
}

void
MulticopterAttitudeControl::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

void
MulticopterAttitudeControl::vehicle_actuator_outputs_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_actuator_outputs_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &_actuator_outputs);
	}
}

void
MulticopterAttitudeControl::vehicle_attitude_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_v_att_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
	}
}

void
MulticopterAttitudeControl::sensor_correction_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_sensor_correction_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sensor_correction), _sensor_correction_sub, &_sensor_correction);
	}

	/* update the latest gyro selection */
	if (_sensor_correction.selected_gyro_instance < _gyro_count) {
		_selected_gyro = _sensor_correction.selected_gyro_instance;
	}
}

void
MulticopterAttitudeControl::sensor_bias_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_sensor_bias_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sensor_bias), _sensor_bias_sub, &_sensor_bias);
	}

}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void
MulticopterAttitudeControl::control_attitude(float dt)
{
//	PX4_INFO("attitude loop: 1000*dt = %d", (int)(dt * 1000.0f));
	vehicle_attitude_setpoint_poll();
	_thrust_sp = _v_att_sp.thrust;

	/* prepare yaw weight from the ratio between roll/pitch and yaw gains */
	Vector3f attitude_gain = _attitude_p;
	const float roll_pitch_gain = (attitude_gain(0) + attitude_gain(1)) / 2.f;
	const float yaw_w = math::constrain(attitude_gain(2) / roll_pitch_gain, 0.f, 1.f);
	attitude_gain(2) = roll_pitch_gain;

	/* get estimated and desired vehicle attitude */
	Quatf q(_v_att.q);
	Quatf qd(_v_att_sp.q_d);

	/* ensure input quaternions are exactly normalized because acosf(1.00001) == NaN */
	q.normalize();
	qd.normalize();

	/* calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch */
	Vector3f e_z = q.dcm_z();
	Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (abs(qd_red(1)) > (1.f - 1e-5f) || abs(qd_red(2)) > (1.f - 1e-5f)) {
		/* In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
		 * full attitude control anyways generates no yaw input and directly takes the combination of
		 * roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable. */
		qd_red = qd;

	} else {
		/* transform rotation from current to desired thrust vector into a world frame reduced desired attitude */
		qd_red *= q;
	}

	/* mix full and reduced desired attitude */
	Quatf q_mix = qd_red.inversed() * qd;
	q_mix *= math::signNoZero(q_mix(0));
	/* catch numerical problems with the domain of acosf and asinf */
	q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
	q_mix(3) = math::constrain(q_mix(3), -1.f, 1.f);
	qd = qd_red * Quatf(cosf(yaw_w * acosf(q_mix(0))), 0, 0, sinf(yaw_w * asinf(q_mix(3))));

	/* quaternion attitude control law, qe is rotation from q to qd */
	Quatf qe = q.inversed() * qd;

	/* using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	 * also taking care of the antipodal unit quaternion ambiguity */
	Vector3f eq = 2.f * math::signNoZero(qe(0)) * qe.imag();

	/* calculate angular rates setpoint */
	_rates_sp = eq.emult(attitude_gain);

	/* Feed forward the yaw setpoint rate.
	 * The yaw_feedforward_rate is a commanded rotation around the world z-axis,
	 * but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
	 * Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
	 * and multiply it by the yaw setpoint rate (yaw_sp_move_rate) and gain (_yaw_ff).
	 * This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
	 * such that it can be added to the rates setpoint.
	 */
	Vector3f yaw_feedforward_rate = q.inversed().dcm_z();
	yaw_feedforward_rate *= _v_att_sp.yaw_sp_move_rate * _yaw_ff.get();
	_rates_sp += yaw_feedforward_rate;


	/* limit rates */
	for (int i = 0; i < 3; i++) {
		if ((_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) &&
		    !_v_control_mode.flag_control_manual_enabled) {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_auto_rate_max(i), _auto_rate_max(i));

		} else {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_mc_rate_max(i), _mc_rate_max(i));
		}
	}

	/* VTOL weather-vane mode, dampen yaw rate */
	if (_vehicle_status.is_vtol && _v_att_sp.disable_mc_yaw_control) {
		if (_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) {

			const float wv_yaw_rate_max = _auto_rate_max(2) * _vtol_wv_yaw_rate_scale.get();
			_rates_sp(2) = math::constrain(_rates_sp(2), -wv_yaw_rate_max, wv_yaw_rate_max);

			// prevent integrator winding up in weathervane mode
			_rates_int(2) = 0.0f;
		}
	}
}

/*
 * Throttle PID attenuation
 * Function visualization available here https://www.desmos.com/calculator/gn4mfoddje
 * Input: 'tpa_breakpoint', 'tpa_rate', '_thrust_sp'
 * Output: 'pidAttenuationPerAxis' vector
 */
Vector3f
MulticopterAttitudeControl::pid_attenuations(float tpa_breakpoint, float tpa_rate)
{
	/* throttle pid attenuation factor */
	float tpa = 1.0f - tpa_rate * (fabsf(_v_rates_sp.thrust) - tpa_breakpoint) / (1.0f - tpa_breakpoint);
	tpa = fmaxf(TPA_RATE_LOWER_LIMIT, fminf(1.0f, tpa));

	Vector3f pidAttenuationPerAxis;
	pidAttenuationPerAxis(AXIS_INDEX_ROLL) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_PITCH) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_YAW) = 1.0;

	return pidAttenuationPerAxis;
}

/**
 * Transform computed torques to equivalent normalized inputs
 */
Vector3f
MulticopterAttitudeControl::torque_to_attctrl(Vector3f &computed_torque)
{
	/* motor maximum thrust model */
	float thrust_max = motor_max_thrust(_battery_status.voltage_filtered_v);

	/* mixer matrix */
	float array_mixer[4][3] = {
			{-0.707107f,  0.707107f,  1.0f},
			{0.707107f,  -0.707107f,  1.0f},
			{0.707107f,  0.707107f,  -1.0f},
			{-0.707107f,  -0.707107f,  -1.0f}
	};
	Matrix<float, 4, 3> mixer_matrix(array_mixer);

	/* matrix from thrust of four propellers to torque about 3-axes */
//	float array_torque[3][4] = {
//			{-HALF_LENGTH, HALF_LENGTH, HALF_LENGTH, -HALF_LENGTH},
//			{HALF_WIDTH, -HALF_WIDTH, HALF_WIDTH, -HALF_WIDTH},
//			{C_M, C_M, -C_M, -C_M}
//	};
	float array_torque[3][4] = {
			{-_half_length, _half_length, _half_length, -_half_length},
			{_half_width, -_half_width, _half_width, -_half_width},
			{_C_M, _C_M, -_C_M, -_C_M}
	};
	Matrix<float, 3, 4> gentrq_matrix(array_torque);

	/* input = (Gamma * Mixer * Tmax)^(-1) * computed_torque */
	SquareMatrix<float, 3> gentrq_mixer = Matrix<float, 3, 3>(gentrq_matrix * mixer_matrix);
	Matrix<float, 3, 3> trq_to_attctrl = gentrq_mixer.I() / thrust_max;

	return trq_to_attctrl * computed_torque;

}

/**
 * Motor thrust model
 */
inline float MulticopterAttitudeControl::motor_max_thrust(float battery_voltage)
{
	if(battery_voltage < 10.9f){
		battery_voltage = 10.9f;
	} else if(battery_voltage > 12.6f){
		battery_voltage = 12.6;
	}
	return 1.0f*5.488f * sinf(battery_voltage * 0.4502f + 2.2241f);
//	return 4.77f;
}

/**
 * Torque around 3-axis generated by four motor thrusts
 */
Vector3f
MulticopterAttitudeControl::compute_actuate_torque()
{
	float thrust_max = motor_max_thrust(_battery_status.voltage_filtered_v);
//	float array_torque[3][4] = {
//			{-HALF_LENGTH, HALF_LENGTH, HALF_LENGTH, -HALF_LENGTH},
//			{HALF_WIDTH, -HALF_WIDTH, HALF_WIDTH, -HALF_WIDTH},
//			{C_M, C_M, -C_M, -C_M}
//	};
	float array_torque[3][4] = {
			{-_half_length, _half_length, _half_length, -_half_length},
			{_half_width, -_half_width, _half_width, -_half_width},
			{_C_M, _C_M, -_C_M, -_C_M}
	};
	Matrix<float, 3, 4> gentrq_matrix(array_torque);
	// motor thrust model:
	// thrust = (1 - _thrust_factor) * PWM + _thrust_factor * PWM^2
	float pwm[4];
	float pwm_amp = _pwm_max_value - _pwm_min_value;
//	pwm[0] = (1.0f + _actuator_outputs.output[0]) / 2.0f;
//	pwm[1] = (1.0f + _actuator_outputs.output[1]) / 2.0f;
//	pwm[2] = (1.0f + _actuator_outputs.output[2]) / 2.0f;
//	pwm[3] = (1.0f + _actuator_outputs.output[3]) / 2.0f;
	pwm[0] = math::constrain((_actuator_outputs.output[0] - _pwm_min_value) / pwm_amp, 0.f, 1.f);
	pwm[1] = math::constrain((_actuator_outputs.output[1] - _pwm_min_value) / pwm_amp, 0.f, 1.f);
	pwm[2] = math::constrain((_actuator_outputs.output[2] - _pwm_min_value) / pwm_amp, 0.f, 1.f);
	pwm[3] = math::constrain((_actuator_outputs.output[3] - _pwm_min_value) / pwm_amp, 0.f, 1.f);
	Vector<float, 4> throttle, thrust;
//	throttle(0) = (1.0f - THRUST_FACTOR) * pwm[0] + THRUST_FACTOR * pwm[0] * pwm[0];
//	throttle(1) = (1.0f - THRUST_FACTOR) * pwm[1] + THRUST_FACTOR * pwm[1] * pwm[1];
//	throttle(2) = (1.0f - THRUST_FACTOR) * pwm[2] + THRUST_FACTOR * pwm[2] * pwm[2];
//	throttle(3) = (1.0f - THRUST_FACTOR) * pwm[3] + THRUST_FACTOR * pwm[3] * pwm[3];
	throttle(0) = (1.0f - _thrust_factor) * pwm[0] + _thrust_factor * pwm[0] * pwm[0];
	throttle(1) = (1.0f - _thrust_factor) * pwm[1] + _thrust_factor * pwm[1] * pwm[1];
	throttle(2) = (1.0f - _thrust_factor) * pwm[2] + _thrust_factor * pwm[2] * pwm[2];
	throttle(3) = (1.0f - _thrust_factor) * pwm[3] + _thrust_factor * pwm[3] * pwm[3];
	thrust = throttle * thrust_max;

//	if(_num_update%500 == 0){
//		PX4_INFO("thrust_max = %8.6f", (double)thrust_max);
////		PX4_INFO("_pwm_min_value = %d", _pwm_min_value);
////		PX4_INFO("_pwm_max_value = %d", _pwm_max_value);
//		PX4_INFO("pwm = %8.6f, %8.6f, %8.6f, %8.6f", (double)pwm[0], (double)pwm[1], (double)pwm[2], (double)pwm[3]);
//		PX4_INFO("throttle = %8.6f, %8.6f, %8.6f, %8.6f", (double)throttle(0), (double)throttle(1), (double)throttle(2), (double)throttle(3));
//		PX4_INFO("thrust = %8.6f, %8.6f, %8.6f, %8.6f", (double)thrust(0), (double)thrust(1), (double)thrust(2), (double)thrust(3));
//	}

	return gentrq_matrix * (throttle * thrust_max);
}

/**
 * Estimator for attitude rates and accelerations using Runge-Kutta methods
 */
void MulticopterAttitudeControl::estimator_update(Vector3f x, const float dt, Vector3f &x1, Vector3f &x2)
{
	Vector3f x1_dot_1, x2_dot_1, x1_dot_2, x2_dot_2, x1_dot_3, x2_dot_3, x1_dot_4, x2_dot_4;
	Vector3f x1_tmp, x2_tmp;

	x1_tmp = x1;	// x1: estimate of x
	x2_tmp = x2;	// x2: estimate of x' acceleration
	estimator_model(x, x1_tmp, x2_tmp, x1_dot_1, x2_dot_1);

	x1_tmp = x1 + x1_dot_1 * dt/2.0f;
	x2_tmp = x2 + x2_dot_1 * dt/2.0f;
	estimator_model(x, x1_tmp, x2_tmp, x1_dot_2, x2_dot_2);

	x1_tmp = x1 + x1_dot_2 * dt/2.0f;
	x2_tmp = x2 + x2_dot_2 * dt/2.0f;
	estimator_model(x, x1_tmp, x2_tmp, x1_dot_3, x2_dot_3);

	x1_tmp = x1 + x1_dot_3 * dt;
	x2_tmp = x2 + x2_dot_3 * dt;
	estimator_model(x, x1_tmp, x2_tmp, x1_dot_4, x2_dot_4);

	x1 += (x1_dot_1 + x1_dot_2 * 2.0f + x1_dot_3 * 2.0f + x1_dot_4) * dt/6.0f;
	x2 += (x2_dot_1 + x2_dot_2 * 2.0f + x2_dot_3 * 2.0f + x2_dot_4) * dt/6.0f;

}

/**
 * Estimator model
 * model: x_hat = wa^2 / (s^2 + 2*zeta*wa*s + wa^2) * x
 * x1 = x_hat; x2 = x_dot_hat
 * ==> x2_dot + 2*zeta*wa*x2 + wa^2*x1 = wa^2 * x
 * ==> x1_dot = x2;
 * 	   x2_dot = wa^2*(x - x1) - 2*zeta*wa*x2
 */
void MulticopterAttitudeControl::estimator_model(Vector3f &rates, Vector3f &x1, Vector3f &x2, Vector3f &x1_dot, Vector3f &x2_dot)
{
	x1_dot = x2;
//	x2_dot =  (rates - x1) * OMEGA_ATT * OMEGA_ATT - x2 * 2.0f * ZETA_ATT * OMEGA_ATT;
	x2_dot =  (rates - x1) * _omega_att * _omega_att - x2 * 2.0f * _zeta_att * _omega_att;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
MulticopterAttitudeControl::control_attitude_rates(float dt)
{
	/* reset integral if disarmed */
	if (!_v_control_mode.flag_armed || !_vehicle_status.is_rotary_wing) {
		_rates_int.zero();
	}

	// get the raw gyro data and correct for thermal errors
	Vector3f rates;

	if (_selected_gyro == 0) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_0[0]) * _sensor_correction.gyro_scale_0[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_0[1]) * _sensor_correction.gyro_scale_0[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_0[2]) * _sensor_correction.gyro_scale_0[2];

	} else if (_selected_gyro == 1) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_1[0]) * _sensor_correction.gyro_scale_1[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_1[1]) * _sensor_correction.gyro_scale_1[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_1[2]) * _sensor_correction.gyro_scale_1[2];

	} else if (_selected_gyro == 2) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_2[0]) * _sensor_correction.gyro_scale_2[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_2[1]) * _sensor_correction.gyro_scale_2[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_2[2]) * _sensor_correction.gyro_scale_2[2];

	} else {
		rates(0) = _sensor_gyro.x;
		rates(1) = _sensor_gyro.y;
		rates(2) = _sensor_gyro.z;
	}

	// rotate corrected measurements from sensor to body frame
	rates = _board_rotation * rates;

	// correct for in-run bias errors
	rates(0) -= _sensor_bias.gyro_x_bias;
	rates(1) -= _sensor_bias.gyro_y_bias;
	rates(2) -= _sensor_bias.gyro_z_bias;

//	Vector3f rates_p_scaled = _rate_p.emult(pid_attenuations(_tpa_breakpoint_p.get(), _tpa_rate_p.get()));
//	Vector3f rates_i_scaled = _rate_i.emult(pid_attenuations(_tpa_breakpoint_i.get(), _tpa_rate_i.get()));
//	Vector3f rates_d_scaled = _rate_d.emult(pid_attenuations(_tpa_breakpoint_d.get(), _tpa_rate_d.get()));
	Vector3f rates_p_scaled = _rate_p;
	Vector3f rates_i_scaled = _rate_i;
	Vector3f rates_d_scaled = _rate_d;

//	PX4_INFO("dt = %8.6f", (double)dt);

	/* angular rates error */
	Vector3f rates_err = _rates_sp - rates;

	/* apply low-pass filtering to the rates for D-term */
	Vector3f rates_filtered(
		_lp_filters_d[0].apply(rates(0)),
		_lp_filters_d[1].apply(rates(1)),
		_lp_filters_d[2].apply(rates(2)));

	/* estimate attitude rates and accelerations */
	estimator_update(rates, dt, _x1, _x2);

	/* compute torque_hat */
//	Vector3f inertia(I_XX, I_YY, I_ZZ);
	Vector3f inertia(_I_XX, _I_YY, _I_ZZ);
	_torque_hat = inertia.emult(_x2) + _x1.cross(inertia.emult(_x1));	// tao = I*w_dot + w x I*w

	/* compute torque_motor through motor thrust model */
	_torque_motor = compute_actuate_torque();
	estimator_update(_torque_motor, dt, _x1_trq, _x2_trq);

	_torque_dist = _torque_hat - _x1_trq;
	_torque_dist_last = _torque_dist;

//	print_vector3("compen_torque", compen_torque, 1);
//	print_vector3("torque_dist", _torque_dist, 1);
//	print_vector3("torque_hat", _torque_hat);
//	print_vector3("torque_motor", _x1_trq);

	/* publish estimated disturbance */
	_ext_torque.esti_dist[0] = _torque_dist(0);
	_ext_torque.esti_dist[1] = _torque_dist(1);
	_ext_torque.esti_dist[2] = _torque_dist(2);

	_ext_torque.ft_hat[0] = _torque_hat(0);
	_ext_torque.ft_hat[1] = _torque_hat(1);
	_ext_torque.ft_hat[2] = _torque_hat(2);

	_ext_torque.ft_motor[0] = _torque_motor(0);
	_ext_torque.ft_motor[1] = _torque_motor(1);
	_ext_torque.ft_motor[2] = _torque_motor(2);
	_ext_torque.ft_motor_filter[0] = _x1_trq(0);
	_ext_torque.ft_motor_filter[1] = _x1_trq(1);
	_ext_torque.ft_motor_filter[2] = _x1_trq(2);

	_ext_torque.timestamp = hrt_absolute_time();

	if(_ext_torque_pub != nullptr){
		orb_publish(ORB_ID(estimate_torque), _ext_torque_pub, &_ext_torque);
	} else {
		_ext_torque_pub = orb_advertise(ORB_ID(estimate_torque), &_ext_torque);
	}

	/* publish angular vel/acc */
	_v_esti_att.rollspeed = _x1(0);
	_v_esti_att.pitchspeed = _x1(1);
	_v_esti_att.yawspeed = _x1(2);
	_v_esti_att.rollacc = _x2(0);
	_v_esti_att.pitchacc = _x2(1);
	_v_esti_att.yawacc = _x2(2);
	_v_esti_att.dt = dt;
	_v_esti_att.timestamp = hrt_absolute_time();

	if(_v_esti_att_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_estimate_attitude), _v_esti_att_pub, &_v_esti_att);
	} else {
		_v_esti_att_pub = orb_advertise(ORB_ID(vehicle_estimate_attitude), &_v_esti_att);
	}

	/* control law part */
	if(_ndi_enable == 0){
		_att_control = rates_p_scaled.emult(rates_err) +
				   _rates_int -
				   rates_d_scaled.emult(rates_filtered - _rates_prev_filtered) / dt +
				   _rate_ff.emult(_rates_sp);
	} else if(_ndi_enable == 1){
		/* using PD controller for linear part */
		Vector3f att_ctrl = rates_p_scaled.emult(rates_err) + _rates_int - rates_d_scaled.emult(rates_filtered - _rates_prev_filtered) / dt * 1.0f;
		/* compute gyro moment */
		Vector3f torque_affix = rates.cross(inertia.emult(rates));
		if(_ndrc_att_enable == 0){
			Vector3f torque_att_ctrl = inertia.emult(att_ctrl) + torque_affix;
			_att_control = torque_to_attctrl(torque_att_ctrl);
		} else{
			/* Compensate nonlinear gyro moment and disturbed moment */
			Vector3f compen_torque = torque_affix * 1.0f - _torque_dist * 1.0f;
			/* Convert torque command to normalized input */
			Vector3f torque_att_ctrl = inertia.emult(att_ctrl) + compen_torque * 1.f;
			_att_control = torque_to_attctrl(torque_att_ctrl);
		}
	}

	_rates_prev = rates;
	_rates_prev_filtered = rates_filtered;

	/* update integral only if motors are providing enough thrust to be effective */
	if (_thrust_sp > MIN_TAKEOFF_THRUST) {
		for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
			// Check for positive control saturation
			bool positive_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_pos) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_pos) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_pos);

			// Check for negative control saturation
			bool negative_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_neg) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_neg) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_neg);

			// prevent further positive control saturation
			if (positive_saturation) {
				rates_err(i) = math::min(rates_err(i), 0.0f);

			}

			// prevent further negative control saturation
			if (negative_saturation) {
				rates_err(i) = math::max(rates_err(i), 0.0f);

			}

			// Perform the integration using a first order method and do not propagate the result if out of range or invalid
			float rate_i = _rates_int(i) + rates_i_scaled(i) * rates_err(i) * dt;

			if (PX4_ISFINITE(rate_i) && rate_i > -_rate_int_lim(i) && rate_i < _rate_int_lim(i)) {
				_rates_int(i) = rate_i;

			}
		}
	}

	/* explicitly limit the integrator state */
	for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
		_rates_int(i) = math::constrain(_rates_int(i), -_rate_int_lim(i), _rate_int_lim(i));

	}
}

void
MulticopterAttitudeControl::run()
{
	/*
	 * do subscriptions
	 */
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	_actuator_outputs_sub = orb_subscribe(ORB_ID(actuator_outputs));

	_gyro_count = math::min(orb_group_count(ORB_ID(sensor_gyro)), MAX_GYRO_COUNT);

	if (_gyro_count == 0) {
		_gyro_count = 1;
	}

	for (unsigned s = 0; s < _gyro_count; s++) {
		_sensor_gyro_sub[s] = orb_subscribe_multi(ORB_ID(sensor_gyro), s);
	}

	_sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));

	// sensor correction topic is not being published regularly and we might have missed the first update.
	// so copy it once initially so that we have the latest data. In future this will not be needed anymore as the
	// behavior of the orb_check function will change
	if (_sensor_correction_sub > 0) {
		orb_copy(ORB_ID(sensor_correction), _sensor_correction_sub, &_sensor_correction);
	}

	_sensor_bias_sub = orb_subscribe(ORB_ID(sensor_bias));

	/* wakeup source: gyro data from sensor selected by the sensor app */
	px4_pollfd_struct_t poll_fds = {};
	poll_fds.events = POLLIN;

	const hrt_abstime task_start = hrt_absolute_time();
	hrt_abstime last_run = task_start;
	float dt_accumulator = 0.f;
	int loop_counter = 0;

	while (!should_exit()) {

		_num_update++;	// count for print info

		poll_fds.fd = _sensor_gyro_sub[_selected_gyro];

		/* wait for up to 100ms for data */
		int pret = px4_poll(&poll_fds, 1, 100);

		/* timed out - periodic check for should_exit() */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			PX4_ERR("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		/* run controller on gyro changes */
		if (poll_fds.revents & POLLIN) {
			const hrt_abstime now = hrt_absolute_time();
			float dt = (now - last_run) / 1e6f;
			last_run = now;

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			/* copy gyro data */
			orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub[_selected_gyro], &_sensor_gyro);

			/* check for updates in other topics */
			parameter_update_poll();
			vehicle_control_mode_poll();
			vehicle_manual_poll();
			vehicle_status_poll();
			vehicle_motor_limits_poll();
			battery_status_poll();
			vehicle_attitude_poll();
			sensor_correction_poll();
			sensor_bias_poll();
			vehicle_actuator_outputs_poll();

			/* Check if we are in rattitude mode and the pilot is above the threshold on pitch
			 * or roll (yaw can rotate 360 in normal att control).  If both are true don't
			 * even bother running the attitude controllers */
			if (_v_control_mode.flag_control_rattitude_enabled) {
				if (fabsf(_manual_control_sp.y) > _rattitude_thres.get() ||
				    fabsf(_manual_control_sp.x) > _rattitude_thres.get()) {
					_v_control_mode.flag_control_attitude_enabled = false;
				}
			}

			if (_v_control_mode.flag_control_attitude_enabled) {

				control_attitude(dt);

				/* publish attitude rates setpoint */
				_v_rates_sp.roll = _rates_sp(0);
				_v_rates_sp.pitch = _rates_sp(1);
				_v_rates_sp.yaw = _rates_sp(2);
				_v_rates_sp.thrust = _thrust_sp;
				_v_rates_sp.timestamp = hrt_absolute_time();

				if (_v_rates_sp_pub != nullptr) {
					orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

				} else if (_rates_sp_id) {
					_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
				}

			} else {
				/* attitude controller disabled, poll rates setpoint topic */
				if (_v_control_mode.flag_control_manual_enabled) {
					/* manual rates control - ACRO mode */
					Vector3f man_rate_sp(
							math::superexpo(_manual_control_sp.y, _acro_expo_rp.get(), _acro_superexpo_rp.get()),
							math::superexpo(-_manual_control_sp.x, _acro_expo_rp.get(), _acro_superexpo_rp.get()),
							math::superexpo(_manual_control_sp.r, _acro_expo_y.get(), _acro_superexpo_y.get()));
					_rates_sp = man_rate_sp.emult(_acro_rate_max);
					_thrust_sp = _manual_control_sp.z;

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

					} else if (_rates_sp_id) {
						_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
					}

				} else {
					/* attitude controller disabled, poll rates setpoint topic */
					vehicle_rates_setpoint_poll();
					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = _v_rates_sp.thrust;
				}
			}

			if (_v_control_mode.flag_control_rates_enabled) {
				control_attitude_rates(dt);

				/* publish actuator controls */
				_actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
				_actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
				_actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
				_actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
				_actuators.control[7] = _v_att_sp.landing_gear;
				_actuators.timestamp = hrt_absolute_time();
				_actuators.timestamp_sample = _sensor_gyro.timestamp;

				/* scale effort by battery status */
				if (_bat_scale_en.get() && _battery_status.scale > 0.0f) {
					for (int i = 0; i < 4; i++) {
						_actuators.control[i] *= _battery_status.scale;
					}
				}

				if (!_actuators_0_circuit_breaker_enabled) {
					if (_actuators_0_pub != nullptr) {

						orb_publish(_actuators_id, _actuators_0_pub, &_actuators);

					} else if (_actuators_id) {
						_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
					}

				}

				/* publish controller status */
				rate_ctrl_status_s rate_ctrl_status;
				rate_ctrl_status.timestamp = hrt_absolute_time();
				rate_ctrl_status.rollspeed = _rates_prev(0);
				rate_ctrl_status.pitchspeed = _rates_prev(1);
				rate_ctrl_status.yawspeed = _rates_prev(2);
				rate_ctrl_status.rollspeed_integ = _rates_int(0);
				rate_ctrl_status.pitchspeed_integ = _rates_int(1);
				rate_ctrl_status.yawspeed_integ = _rates_int(2);

				int instance;
				orb_publish_auto(ORB_ID(rate_ctrl_status), &_controller_status_pub, &rate_ctrl_status, &instance, ORB_PRIO_DEFAULT);
			}

			if (_v_control_mode.flag_control_termination_enabled) {
				if (!_vehicle_status.is_vtol) {

					_rates_sp.zero();
					_rates_int.zero();
					_thrust_sp = 0.0f;
					_att_control.zero();

					/* publish actuator controls */
					_actuators.control[0] = 0.0f;
					_actuators.control[1] = 0.0f;
					_actuators.control[2] = 0.0f;
					_actuators.control[3] = 0.0f;
					_actuators.timestamp = hrt_absolute_time();
					_actuators.timestamp_sample = _sensor_gyro.timestamp;

					if (!_actuators_0_circuit_breaker_enabled) {
						if (_actuators_0_pub != nullptr) {

							orb_publish(_actuators_id, _actuators_0_pub, &_actuators);

						} else if (_actuators_id) {
							_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
						}
					}
				}
			}

			/* calculate loop update rate while disarmed or at least a few times (updating the filter is expensive) */
			if (!_v_control_mode.flag_armed || (now - task_start) < 3300000) {
				dt_accumulator += dt;
				++loop_counter;

				if (dt_accumulator > 1.f) {
					const float loop_update_rate = (float)loop_counter / dt_accumulator;
					_loop_update_rate_hz = _loop_update_rate_hz * 0.5f + loop_update_rate * 0.5f;
					dt_accumulator = 0;
					loop_counter = 0;
					_lp_filters_d[0].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
					_lp_filters_d[1].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
					_lp_filters_d[2].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
				}
			}

		}

		perf_end(_loop_perf);
	}

	orb_unsubscribe(_v_att_sub);
	orb_unsubscribe(_v_att_sp_sub);
	orb_unsubscribe(_v_rates_sp_sub);
	orb_unsubscribe(_v_control_mode_sub);
	orb_unsubscribe(_params_sub);
	orb_unsubscribe(_manual_control_sp_sub);
	orb_unsubscribe(_vehicle_status_sub);
	orb_unsubscribe(_motor_limits_sub);
	orb_unsubscribe(_battery_status_sub);

	for (unsigned s = 0; s < _gyro_count; s++) {
		orb_unsubscribe(_sensor_gyro_sub[s]);
	}

	orb_unsubscribe(_sensor_correction_sub);
	orb_unsubscribe(_sensor_bias_sub);
	orb_unsubscribe(_actuator_outputs_sub);
}

int MulticopterAttitudeControl::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("mc_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_ATTITUDE_CONTROL,
					   1700,
					   (px4_main_t)&run_trampoline,
					   (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

MulticopterAttitudeControl *MulticopterAttitudeControl::instantiate(int argc, char *argv[])
{
	return new MulticopterAttitudeControl();
}

int MulticopterAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int mc_att_control_main(int argc, char *argv[])
{
	return MulticopterAttitudeControl::main(argc, argv);
}
