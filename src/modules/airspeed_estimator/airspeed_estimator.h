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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/airspeed_multi_record.h>
#include <uORB/topics/wind.h>
#include <uORB/topics/airspeed_wind.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>

#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>

extern "C" __EXPORT int airspeed_estimator_main(int argc, char *argv[]);


class AirspeedEstimator : public ModuleBase<AirspeedEstimator>, public ModuleParams
{
public:
	AirspeedEstimator(int example_param, bool example_flag);

	virtual ~AirspeedEstimator() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static AirspeedEstimator *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** solve quadratic for Va **/
	float calcVa(float Vpit, float n);

	float calcEKF(float n, float vPit);
	float Hfn(float Vakm1, float n);
	float SlipFn(float Vakm1, float n);

	float calcComp(float vaEst, float Va_w);

	float calcExpectAs();

	// Thruster curve fit
	float p00 = 0.04022f;
	float p10 = 0.07437f;
	float p01 = 0.528f;
	float p20 = 0.00004402f;
	float p11 = -0.001512f;
	float p02 = 0.009899f;

	bool solExist{true};

	// Complimentary filter
	float alpha{0.0f};
	float phi{0.0f};
	float phiStart = 25;
	float alphaStart = 0.3;
	float alphaEnd = 0.1;

	// EKF
	float Q = 10.0f; // wind variance
	float R = 10.0f; // sensor variance
	float Fk = 1.0f;
	float Pkm1_km1 = 20.0f;


private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

	//Subscriptions
	struct airspeed_multi_record_s masm;
	struct wind_s windEst;
	struct vehicle_attitude_s att;
	struct vehicle_local_position_s pos;

	float Va{10.0f}; // Declare here, only update if we get a new good Va
	float nRec{0.0f}; //prop Rev/s
	float pitRec{0.0f}; //pitot tube recording

};

