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

#include "airspeed_estimator.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>


int AirspeedEstimator::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int AirspeedEstimator::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int AirspeedEstimator::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

AirspeedEstimator *AirspeedEstimator::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	AirspeedEstimator *instance = new AirspeedEstimator(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

AirspeedEstimator::AirspeedEstimator(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void AirspeedEstimator::run()
{
	// Example: run the loop synchronized to the sensor_combined topic publication
	int wind_sub = orb_subscribe(ORB_ID(wind));
	int masm_sub = orb_subscribe(ORB_ID(airspeed_multi_record));

	orb_set_interval(wind_sub, 20);
	orb_set_interval(masm_sub, 20);

	px4_pollfd_struct_t fds[] = {
		{ .fd = wind_sub,   .events = POLLIN },
		{ .fd = masm_sub,   .events = POLLIN },
	};

	/* advertise airspeed topic */
	struct airspeed_s airspeed_d;
	memset(&airspeed_d, 0, sizeof(airspeed_d));
	orb_advert_t airspeed_pub = orb_advertise(ORB_ID(airspeed), &airspeed_d);


	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, 2, 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[1].revents & POLLIN) {

			orb_copy(ORB_ID(airspeed_multi_record), masm_sub, &masm); //Copy masm data

			if(fds[0].revents)
			{
				orb_copy(ORB_ID(wind), wind_sub, &windEst); //Copy wind data if we get it
			}


			nRec = masm.rpm_sens / 60.0f; // Convert to rev/s
			pitRec = masm.primary_airspeed_ms;

			float VaTemp = calcVa(pitRec, nRec);
			if(VaTemp > 0 && PX4_ISFINITE(VaTemp))
			{
				//Let through if positive
				airspeed_d.indicated_airspeed_m_s = calcVa(pitRec, nRec);
				airspeed_d.true_airspeed_m_s = calcVa(pitRec, nRec);
			}else{
				//Deny if negative
				airspeed_d.indicated_airspeed_m_s = 0.0f;
				airspeed_d.true_airspeed_m_s = 0.0f;
			}

			airspeed_d.timestamp = hrt_absolute_time();
			airspeed_d.air_temperature_celsius = masm.primary_temperature;
			airspeed_d.confidence = 1;

		}
		// else
		// {
		// 	px4_usleep(50000);
		// 	PX4_ERR("Fail");
		// }

		orb_publish(ORB_ID(airspeed), airspeed_pub, &airspeed_d); //Publish
	}

	// orb_unsubscribe(sensor_combined_sub);
}



int AirspeedEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int airspeed_estimator_main(int argc, char *argv[])
{
	return AirspeedEstimator::main(argc, argv);
}

float AirspeedEstimator::calcVa(float Vpit, float n)
{
	float A = p02;
	float B = (p01 + p11*n);
	float C = (p00 + p10*n + p20*n*n - Vpit);

	solExist = B*B - 4*A*C > 0;
	// float Va = 10.0f; //TODO should I just declare this in another scope and only write to it if its reasonable?

	if(solExist)
	{
		Va = (-B + sqrtf(B*B - 4*A*C)) / (2*A);

		if(Va<0)
		{
			Va = (-B - sqrtf(B*B - 4*A*C)) / (2*A);
		}
	}
	return Va;
}
