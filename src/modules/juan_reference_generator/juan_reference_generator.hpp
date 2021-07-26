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

#ifndef JUAN_REFERENCE_GENERATOR_HPP
#define JUAN_REFERENCE_GENERATOR_HPP

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
#include <uORB/topics/juan_manouver_start.h>
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


extern "C" __EXPORT int juan_reference_generator_main(int argc, char *argv[]);


class juan_reference_generator :  public ModuleBase<juan_reference_generator> , public ModuleParams
{
public:
	juan_reference_generator(int example_param, bool example_flag);

	virtual ~juan_reference_generator() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static juan_reference_generator *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run();

	/** @see ModuleBase::print_status() */
	int print_status();

	// static int _task_id;

private:

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

	// uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};	/**< vehicle attitude */

	// uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};		/**< vehicle attitude setpoint */
	// uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};			/**< battery status subscription */
	// uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};		/**< global position subscription */
	// uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};		/**< notification of manual control updates */
	// uORB::Subscription _rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};		/**< vehicle rates setpoint */
	// uORB::Subscription _vcontrol_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle status subscription */
	// uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */
	// uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< vehicle status subscription */
	// uORB::Subscription _vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};

	/* Jackson */
	uORB::Subscription _wind_estimate_sub{ORB_ID(wind)};

	/* Juan */
	uORB::Subscription _vehicle_local_position_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _juan_attitude_variables_sub{ORB_ID(juan_attitude_variables)}; //JUAN
	uORB::Subscription _juan_position_variables_sub{ORB_ID(juan_position_variables)};
	uORB::Subscription _juan_reference_variables_sub{ORB_ID(juan_reference_variables)};
	uORB::Subscription _juan_manouver_start_sub{ORB_ID(juan_manouver_start)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};		/**< local position subscription */

	uORB::SubscriptionData<airspeed_s> _airspeed_sub{ORB_ID(airspeed)};

	/* Juan */
	// uORB::Publication<vehicle_local_position_setpoint_s>	_local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<juan_attitude_variables_s>	_juan_attitude_variables_pub{ORB_ID(juan_attitude_variables)}; //JUAN


	/* Structs */
	orb_id_t	_attitude_setpoint_id{nullptr};
	orb_advert_t	_attitude_sp_pub{nullptr};	/**< attitude setpoint point */

	//JUAN
	vehicle_local_position_setpoint_s _local_pos_sp{}; //local position setpoint
	juan_attitude_variables_s _juan_att_var{}; // JUAN custom attitude control variables
	juan_position_variables_s _juan_pos_var{}; // JUAN custom poition control variables
	juan_reference_variables_s _juan_ref_var{}; // JUAN custom reference variables
	juan_manouver_start_s _juan_man_start{}; // JUAN custom manouver start
	vehicle_local_position_s		_local_pos {};		/**< local position */


	// other variables
	float _previous_time{0.0f};
	float _ground_velocity_corrected{5.0f};
	float _time_elapsed{0.0f};
	float _delta_time_attitude{0.0f};

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

	float t_last_vertex{0.0f};
	float t_last{0.0f};

	float _pos_x_last_vtx{0.0f}; //Positions of last vertex reletive to _pos_init
	float _pos_y_last_vtx{0.0f};
	int turnCount{0}; //Number of turns of the box performed
	bool completeFlag = false; //flag showing that the desired path is complete
	bool exitMsgSent = false;

};

#endif

#ifndef M_PI_f
#define M_PI_f			3.14159265f
#endif
#ifndef PI_f
#define PI_f			3.14159265f
#endif
