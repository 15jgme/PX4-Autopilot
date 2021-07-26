/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#ifndef JUAN_POSITION_CONTROL_HPP
#define JUAN_POSITION_CONTROL_HPP

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/juan_attitude_variables.h> //JUAN
#include <uORB/topics/juan_position_variables.h> //JUAN
#include <uORB/topics/juan_reference_variables.h> //JUAN
#include <uORB/topics/vehicle_local_position_setpoint.h> //JUAN
#include <uORB/topics/vehicle_local_position.h> //JUAN
#include <uORB/topics/wind.h> //JACKSON
#include <parameters/param.h>
#include <uORB/SubscriptionInterval.hpp>

#include <airspeed/airspeed.h>
#include <drivers/drv_sensor.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/sensor_combined.h>
#include <uORB/uORB.h>


extern "C" __EXPORT int juan_position_control_main(int argc, char *argv[]);


class juan_position_control :  public ModuleBase<juan_position_control> , public ModuleParams
{
public:
	juan_position_control(int example_param, bool example_flag);

	virtual ~juan_position_control() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static juan_position_control *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run();

	/** @see ModuleBase::print_status() */
	int print_status();

	// static int _task_id;

	/** Calibration for airspeed sensors */
	int diff_pressure_calib();

private:
	float saturate(float value, float min, float max);

	void wind_ff_rot_update();

	void JUAN_singularity_management(float xy_speed, float angle_vect);

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)
	/* ---- Subscriptions ---- */
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1000};

	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};	/**< vehicle attitude */

	uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};		/**< vehicle attitude setpoint */
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};			/**< battery status subscription */
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};		/**< global position subscription */
	uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};		/**< notification of manual control updates */
	uORB::Subscription _rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};		/**< vehicle rates setpoint */
	uORB::Subscription _vcontrol_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle status subscription */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< vehicle status subscription */
	uORB::Subscription _vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};

	/* Jackson */
	uORB::Subscription _wind_estimate_sub{ORB_ID(wind)};

	/* Juan */
	uORB::Subscription _vehicle_local_position_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _juan_attitude_variables_sub{ORB_ID(juan_attitude_variables)}; //JUAN
	uORB::Subscription _juan_position_variables_sub{ORB_ID(juan_position_variables)};
	uORB::Subscription _juan_reference_variables_sub{ORB_ID(juan_reference_variables)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};		/**< local position subscription */

	uORB::SubscriptionData<airspeed_s> _airspeed_sub{ORB_ID(airspeed)};

	/* Juan */
	// uORB::Publication<vehicle_local_position_setpoint_s>	_local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<juan_attitude_variables_s>	_juan_attitude_variables_pub{ORB_ID(juan_attitude_variables)}; //JUAN


	/* Structs */
	orb_id_t	_attitude_setpoint_id{nullptr};
	orb_advert_t	_attitude_sp_pub{nullptr};	/**< attitude setpoint point */

	orb_id_t	_actuators_id{nullptr};		/**< pointer to correct actuator controls0 uORB metadata structure */
	orb_advert_t	_actuators_0_pub{nullptr};	/**< actuator control group 0 setpoint */

	actuator_controls_s			_actuators {};		/**< actuator control inputs */
	actuator_controls_s			_actuators_airframe {};	/**< actuator control inputs */
	manual_control_setpoint_s		_manual {};		/**< r/c channel data */
	vehicle_attitude_s			_att {};		/**< vehicle attitude setpoint */
	vehicle_attitude_setpoint_s		_att_sp {};		/**< vehicle attitude setpoint */
	vehicle_control_mode_s			_vcontrol_mode {};	/**< vehicle control mode */
	vehicle_global_position_s		_global_pos {};		/**< global position */
	vehicle_rates_setpoint_s		_rates_sp {};		/* attitude rates setpoint */
	vehicle_status_s			_vehicle_status {};	/**< vehicle status */

	//JACKSON
	wind_s				_wind {};		/**< wind */

	//JUAN
	vehicle_local_position_setpoint_s _local_pos_sp{}; //local position setpoint
	juan_attitude_variables_s _juan_att_var{}; // JUAN custom attitude control variables
	juan_position_variables_s _juan_pos_var{}; // JUAN custom poition control variables
	juan_reference_variables_s _juan_ref_var{}; // JUAN custom reference variables
	vehicle_local_position_s		_local_pos {};		/**< local position */


	// other variables
	float _previous_time{0.0f};
	float _ground_velocity_corrected{5.0f};
	float _time_elapsed{0.0f};
	float _delta_time_attitude{0.0f};
	float _e_int_1{0.0f};
	float _e_int_2{0.0f};
	float _e_int_3{0.0f};

	// position control variables
	float _pos_x_ref{0.0f}; // position references
	float _pos_y_ref{0.0f};
	float _pos_z_ref{0.0f};
	float _vel_x_ref{0.0f}; // velocity references
	float _vel_y_ref{0.0f};
	float _vel_z_ref{0.0f};
	float _initial_heading{0.0f}; // initialization values
	float _pos_x_initial{0.0f};
	float _pos_y_initial{0.0f};
	float _pos_z_initial{0.0f};
	float _error_x_int{0.0f};
	float _error_y_int{0.0f};
	float _error_z_int{0.0f};

	// Position controller outputs: ref. DCM and thrust command.
	float C_reference_rows[9] = {1.0f, 0.0f,0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,1.0f};
	float ThrustN{0.0f};
	int _control_operation_mode{0}; // controller exception management
	float belly_n_old;
	float belly_e_old;
	matrix::Dcmf C_bi;
	matrix::Dcmf C_ri_pos;
	float _error_heading_int{0.0f};
	int _JUAN_flight_mode{1};
	matrix::Vector3f _omega_reference_body;
	float _throttle_out;
	matrix::Dcmf C_ri;
	float _previous_rpm{900.0f};
	float _advance_ratio{0.5f};
	matrix::Vector3f _alpha_reference_body;
	float _global_jar;

	/* ------ Jackson's stuff -----*/
	// float t_turn_left = NULL;
	float t_last = 0.0f;

	float t_last_vertex{0.0f};

	float _pos_x_last_vtx{0.0f}; //Positions of last vertex reletive to _pos_init
	float _pos_y_last_vtx{0.0f};
	int turnCount{0}; //Number of turns of the box performed
	bool completeFlag = false; //flag showing that the desired path is complete
	bool exitMsgSent = false;
	matrix::Dcmf R_wind; //Feedforward rotation


	bool feedforward_flag = false; //If true wind feedforward on position is enabled

};

#endif

#ifndef M_PI_f
#define M_PI_f			3.14159265f
#endif
#ifndef PI_f
#define PI_f			3.14159265f
#endif
