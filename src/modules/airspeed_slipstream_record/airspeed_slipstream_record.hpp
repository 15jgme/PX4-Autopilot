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

#ifndef AIRSPEED_SLIPSTREAM_RECORD_HPP
#define AIRSPEED_SLIPSTREAM_RECORD_HPP

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed_multi_record.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/SubscriptionInterval.hpp>


extern "C" __EXPORT int airspeed_slipstream_record_main(int argc, char *argv[]);


class airspeed_slipstream_record :  public ModuleBase<airspeed_slipstream_record> , public ModuleParams
{
public:
	airspeed_slipstream_record(int example_param, bool example_flag);

	virtual ~airspeed_slipstream_record() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static airspeed_slipstream_record *instantiate(int argc, char *argv[]);

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

	uint sensID_1 = 4923657; //Sensor ID for primary airspeed sensor FMU
	// uint sensID_1 = 0;
	bool sens_1_active = true;
	// uint sensID_2 = 4748809; //Sensor ID for slipstream airspeed sensor FMU
	uint sensID_2 = 0;
	bool sens_2_active = false;

	bool errFlag = false;

	float airspeed_ID_1 = 4; //Float for storing airspeed calculated
	float airspeed_ID_2 = 4; //Float for storing airspeed calculated

	bool error_sent = false; //Have we already sent the error message?

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

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 50}; // PARAMS
	uORB::SubscriptionInterval _sensor_update_sub{ORB_ID(differential_pressure), 50}; //DIFF PRESSURE
	uORB::SubscriptionInterval _esc_update_sub{ORB_ID(esc_status), 50}; //ESC
	uORB::SubscriptionInterval _asp_update_sub{ORB_ID(airspeed), 50}; //AIRSPEED
	uORB::SubscriptionInterval _rc_update_sub{ORB_ID(rc_channels), 50}; //RC


};

#endif
