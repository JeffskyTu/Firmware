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

#include <lib/mixer/mixer.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_estimate_attitude.h>
#include <uORB/topics/estimate_disturb.h>

/**
 * Multicopter attitude control app start / stop handling function
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[]);

#define MAX_GYRO_COUNT 3


class MulticopterAttitudeControl : public ModuleBase<MulticopterAttitudeControl>, public ModuleParams
{
public:
	MulticopterAttitudeControl();

	virtual ~MulticopterAttitudeControl() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static MulticopterAttitudeControl *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

private:

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void			parameters_updated();

	/**
	 * Check for parameter update and handle it.
	 */
	void		battery_status_poll();
	void		parameter_update_poll();
	void		sensor_bias_poll();
	void		sensor_correction_poll();
	void		vehicle_attitude_poll();
	void		vehicle_attitude_setpoint_poll();
	void		vehicle_control_mode_poll();
	void		vehicle_manual_poll();
	void		vehicle_motor_limits_poll();
	void		vehicle_rates_setpoint_poll();
	void		vehicle_status_poll();

	void		vehicle_actuator_outputs_poll();

	/**
	 * Attitude controller.
	 */
	void		control_attitude(float dt);

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rates(float dt);

	/**
	 * Throttle PID attenuation.
	 */
	matrix::Vector3f pid_attenuations(float tpa_breakpoint, float tpa_rate);

	/**
	 * Transform computed torques to equivalent normalized inputs
	 */
	matrix::Vector3f torque_to_attctrl(matrix::Vector3f &torque);

	/**
	 * Motor thrust model
	 */
	inline float motor_max_thrust(float battery_voltage);

	/**
	 * Torque around 3-axis generated by four motor thrusts
	 */
	matrix::Vector3f compute_actuate_torque();

	/**
	 * Estimator for attitude rates and accelerations using Runge-Kutta methods
	 */
	void estimator_update(matrix::Vector3f x, const float dt, matrix::Vector3f &x1, matrix::Vector3f &x2);

	/**
	 * Estimator model
	 * model: x_hat = wa^2 / (s^2 + 2*zeta*wa*s + wa^2) * x
	 * x1 = x_hat; x2 = x_dot_hat
	 * ==> x2_dot + 2*zeta*wa*x2 + wa^2*x1 = wa^2 * x
	 * ==> x1_dot = x2;
	 * 	   x2_dot = wa^2*(x - x1) - 2*zeta*wa*x2
	 */
	void estimator_model(matrix::Vector3f &rates, matrix::Vector3f &x1, matrix::Vector3f &x2,
						matrix::Vector3f &x1_dot, matrix::Vector3f &x2_dot);

	void print_vector3(const char *name, matrix::Vector3f &vector3, bool print_time = false);



	int		_v_att_sub{-1};			/**< vehicle attitude subscription */
	int		_v_att_sp_sub{-1};		/**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub{-1};		/**< vehicle rates setpoint subscription */
	int		_v_control_mode_sub{-1};	/**< vehicle control mode subscription */
	int		_params_sub{-1};		/**< parameter updates subscription */
	int		_manual_control_sp_sub{-1};	/**< manual control setpoint subscription */
	int		_vehicle_status_sub{-1};	/**< vehicle status subscription */
	int		_motor_limits_sub{-1};		/**< motor limits subscription */
	int		_battery_status_sub{-1};	/**< battery status subscription */
	int		_sensor_gyro_sub[MAX_GYRO_COUNT];	/**< gyro data subscription */
	int		_sensor_correction_sub{-1};	/**< sensor thermal correction subscription */
	int		_sensor_bias_sub{-1};		/**< sensor in-run bias correction subscription */

	int		_actuator_outputs_sub{-1};	/**< actuator outputs subscription */

	unsigned _gyro_count{1};
	int _selected_gyro{0};

	orb_advert_t	_v_rates_sp_pub{nullptr};		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub{nullptr};		/**< attitude actuator controls publication */
	orb_advert_t	_controller_status_pub{nullptr};	/**< controller status publication */
	orb_advert_t	_v_esti_att_pub{nullptr};		/**< estimated attitude speed/acc publication */
	orb_advert_t	_ext_torque_pub{nullptr};		/**< estimated external torque disturbance publication*/

	orb_id_t _rates_sp_id{nullptr};		/**< pointer to correct rates setpoint uORB metadata structure */
	orb_id_t _actuators_id{nullptr};	/**< pointer to correct actuator controls0 uORB metadata structure */

	bool		_actuators_0_circuit_breaker_enabled{false};	/**< circuit breaker to suppress output */

	struct vehicle_attitude_s		_v_att {};		/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_v_att_sp {};		/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp {};		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp {};	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode {};	/**< vehicle control mode */
	struct actuator_controls_s		_actuators {};		/**< actuator controls */
	struct vehicle_status_s			_vehicle_status {};	/**< vehicle status */
	struct battery_status_s			_battery_status {};	/**< battery status */
	struct sensor_gyro_s			_sensor_gyro {};	/**< gyro data before thermal correctons and ekf bias estimates are applied */
	struct sensor_correction_s		_sensor_correction {};	/**< sensor thermal corrections */
	struct sensor_bias_s			_sensor_bias {};	/**< sensor in-run bias corrections */

	struct actuator_outputs_s		_actuator_outputs {};		/**< actuator outputs to motors */
	struct vehicle_estimate_attitude_s  _v_esti_att {};		/**< estimated attitude speed/acc */
	struct estimate_disturb_s		_ext_torque {};		/** estimated external torque disturbance */

	MultirotorMixer::saturation_status _saturation_status{};

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	math::LowPassFilter2p _lp_filters_d[3];                      /**< low-pass filters for D-term (roll, pitch & yaw) */
	static constexpr const float initial_update_rate_hz = 250.f; /**< loop update rate used for initialization */
	float _loop_update_rate_hz{initial_update_rate_hz};          /**< current rate-controller loop update rate in [Hz] */

	matrix::Vector3f _rates_prev;			/**< angular rates on previous step */
	matrix::Vector3f _rates_prev_filtered;		/**< angular rates on previous step (low-pass filtered) */
	matrix::Vector3f _rates_sp;			/**< angular rates setpoint */
	matrix::Vector3f _rates_int;			/**< angular rates integral error */
	float _thrust_sp;				/**< thrust setpoint */
	matrix::Vector3f _att_control;			/**< attitude control vector */

	matrix::Dcmf _board_rotation;			/**< rotation matrix for the orientation that the board is mounted */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLL_P>) _roll_p,
		(ParamFloat<px4::params::MC_ROLLRATE_P>) _roll_rate_p,
		(ParamFloat<px4::params::MC_ROLLRATE_I>) _roll_rate_i,
		(ParamFloat<px4::params::MC_RR_INT_LIM>) _roll_rate_integ_lim,
		(ParamFloat<px4::params::MC_ROLLRATE_D>) _roll_rate_d,
		(ParamFloat<px4::params::MC_ROLLRATE_FF>) _roll_rate_ff,

		(ParamFloat<px4::params::MC_PITCH_P>) _pitch_p,
		(ParamFloat<px4::params::MC_PITCHRATE_P>) _pitch_rate_p,
		(ParamFloat<px4::params::MC_PITCHRATE_I>) _pitch_rate_i,
		(ParamFloat<px4::params::MC_PR_INT_LIM>) _pitch_rate_integ_lim,
		(ParamFloat<px4::params::MC_PITCHRATE_D>) _pitch_rate_d,
		(ParamFloat<px4::params::MC_PITCHRATE_FF>) _pitch_rate_ff,

		(ParamFloat<px4::params::MC_YAW_P>) _yaw_p,
		(ParamFloat<px4::params::MC_YAWRATE_P>) _yaw_rate_p,
		(ParamFloat<px4::params::MC_YAWRATE_I>) _yaw_rate_i,
		(ParamFloat<px4::params::MC_YR_INT_LIM>) _yaw_rate_integ_lim,
		(ParamFloat<px4::params::MC_YAWRATE_D>) _yaw_rate_d,
		(ParamFloat<px4::params::MC_YAWRATE_FF>) _yaw_rate_ff,

		(ParamFloat<px4::params::MC_YAW_FF>) _yaw_ff,					/**< yaw control feed-forward */

		(ParamFloat<px4::params::MC_DTERM_CUTOFF>) _d_term_cutoff_freq,			/**< Cutoff frequency for the D-term filter */

		(ParamFloat<px4::params::MC_TPA_BREAK_P>) _tpa_breakpoint_p,			/**< Throttle PID Attenuation breakpoint */
		(ParamFloat<px4::params::MC_TPA_BREAK_I>) _tpa_breakpoint_i,			/**< Throttle PID Attenuation breakpoint */
		(ParamFloat<px4::params::MC_TPA_BREAK_D>) _tpa_breakpoint_d,			/**< Throttle PID Attenuation breakpoint */
		(ParamFloat<px4::params::MC_TPA_RATE_P>) _tpa_rate_p,				/**< Throttle PID Attenuation slope */
		(ParamFloat<px4::params::MC_TPA_RATE_I>) _tpa_rate_i,				/**< Throttle PID Attenuation slope */
		(ParamFloat<px4::params::MC_TPA_RATE_D>) _tpa_rate_d,				/**< Throttle PID Attenuation slope */

		(ParamFloat<px4::params::MC_ROLLRATE_MAX>) _roll_rate_max,
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _pitch_rate_max,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>) _yaw_rate_max,
		(ParamFloat<px4::params::MC_YAWRAUTO_MAX>) _yaw_auto_max,

		(ParamFloat<px4::params::MC_ACRO_R_MAX>) _acro_roll_max,
		(ParamFloat<px4::params::MC_ACRO_P_MAX>) _acro_pitch_max,
		(ParamFloat<px4::params::MC_ACRO_Y_MAX>) _acro_yaw_max,
		(ParamFloat<px4::params::MC_ACRO_EXPO>) _acro_expo_rp,				/**< expo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_EXPO_Y>) _acro_expo_y,				/**< expo stick curve shape (yaw) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPO>) _acro_superexpo_rp,			/**< superexpo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPOY>) _acro_superexpo_y,			/**< superexpo stick curve shape (yaw) */

		(ParamFloat<px4::params::MC_RATT_TH>) _rattitude_thres,

		(ParamBool<px4::params::MC_BAT_SCALE_EN>) _bat_scale_en,

		(ParamInt<px4::params::SENS_BOARD_ROT>) _board_rotation_param,

		(ParamFloat<px4::params::SENS_BOARD_X_OFF>) _board_offset_x,
		(ParamFloat<px4::params::SENS_BOARD_Y_OFF>) _board_offset_y,
		(ParamFloat<px4::params::SENS_BOARD_Z_OFF>) _board_offset_z,

		(ParamFloat<px4::params::VT_WV_YAWR_SCL>) _vtol_wv_yaw_rate_scale,		/**< Scale value [0, 1] for yaw rate setpoint  */

		(ParamInt<px4::params::PWM_MIN>) _pwm_min,
		(ParamInt<px4::params::PWM_MAX>) _pwm_max
	)

	matrix::Vector3f _attitude_p;		/**< P gain for attitude control */
	matrix::Vector3f _rate_p;		/**< P gain for angular rate error */
	matrix::Vector3f _rate_i;		/**< I gain for angular rate error */
	matrix::Vector3f _rate_int_lim;		/**< integrator state limit for rate loop */
	matrix::Vector3f _rate_d;		/**< D gain for angular rate error */
	matrix::Vector3f _rate_ff;		/**< Feedforward gain for desired rates */

	matrix::Vector3f _mc_rate_max;		/**< attitude rate limits in stabilized modes */
	matrix::Vector3f _auto_rate_max;	/**< attitude rate limits in auto modes */
	matrix::Vector3f _acro_rate_max;	/**< max attitude rates in acro mode */

	/* for disturbance estimator */
	matrix::Vector3f	_x1;		// rates_hat
	matrix::Vector3f	_x2;		// estimate of angular acceleration
//	matrix::Vector3f	_x1_dot;
//	matrix::Vector3f	_x2_dot;
	matrix::Vector3f	_torque_hat;	// estimate of torque
	matrix::Vector3f	_torque_motor;	// current torque computed by motor thrust model
	matrix::Vector3f	_torque_dist;	// disturbed torque
	matrix::Vector3f	_torque_dist_last;
	matrix::Vector3f	_x1_trq;
	matrix::Vector3f	_x2_trq;

	int32_t	_pwm_min_value;
	int32_t _pwm_max_value;
	float 	_omega_att = 50.0f;
	float	_zeta_att = 0.7f;
	float	_half_length = 0.101f;
	float	_half_width	 = 0.08f;
	float	_C_M	= 	0.0097f;
	float	_I_XX	= 3e-3f;
	float	_I_YY	= 2.7e-3f;
	float	_I_ZZ	= 6e-3f;
	float	_thrust_factor = 0.4785f;
	int32_t _ndrc_att_enable;
	int32_t _ndi_enable;


	int _num_update = 0;

};

