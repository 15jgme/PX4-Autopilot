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
				      SCHED_PRIORITY_MAX,
				      1676,
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
	int att_sub  = orb_subscribe(ORB_ID(vehicle_attitude));
	int pos_sub  = orb_subscribe(ORB_ID(vehicle_local_position));

	orb_set_interval(wind_sub, 2);
	orb_set_interval(masm_sub, 2);
	orb_set_interval(att_sub, 2);
	orb_set_interval(pos_sub, 2);

	px4_pollfd_struct_t fds[] = {
		{ .fd = wind_sub,   .events = POLLIN },
		{ .fd = masm_sub,   .events = POLLIN },
		{ .fd = att_sub,   .events = POLLIN },
		{ .fd = pos_sub,   .events = POLLIN },
	};

	/* advertise airspeed topic */
	struct airspeed_s airspeed_d;
	memset(&airspeed_d, 0, sizeof(airspeed_d));
	orb_advert_t airspeed_pub = orb_advertise(ORB_ID(airspeed), &airspeed_d);


	/* advertise estimator topic */
	memset(&airspeed_estimator_dat_d, 0, sizeof(airspeed_estimator_dat_d));
	orb_advert_t airspeed_estimator_dat_pub = orb_advertise(ORB_ID(airspeed_estimator_dat), &airspeed_estimator_dat_d);


	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, 4, 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[1].revents & fds[2].revents & fds[3].revents & POLLIN) {

			bool ekfSw = false; // If false use complimentary filter

			/* ---- Copy data ---- */
			orb_copy(ORB_ID(airspeed_multi_record), masm_sub, &masm);
			orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
			orb_copy(ORB_ID(vehicle_local_position), pos_sub, &pos);

			if(fds[0].revents)
			{
				orb_copy(ORB_ID(wind), wind_sub, &windEst); //Copy wind data if we get it
			}
			/* ---- Copy data ---- */

			/* ---- Get wind estimated airspeed ---- */
			float vaWindEst = calcExpectAs();
			/* ---- Get wind estimated airspeed ---- */

			nRec = masm.rpm_sens / 60.0f; // Convert to rev/s
			pitRec = masm.primary_airspeed_ms;

			float VaOut;

			if(ekfSw)
			{
				/* ---- EKF ---- */
				VaOut = calcEKF(nRec, pitRec);
				/* ---- End of EKF ---- */
			}
			else
			{
				/* ---- Complimentary ---- */
				float VaPit = calcVa(pitRec, nRec);
				VaOut = calcComp(VaPit, vaWindEst); // Run complimentary filter
				/* ---- End of Complimentary ---- */
			}

			/* ---- Protect system ---- */
			if(VaOut > 0 && PX4_ISFINITE(VaOut))
			{
				//Let through if positive
				airspeed_d.indicated_airspeed_m_s = VaOut;
				airspeed_d.true_airspeed_m_s = VaOut;
			}else{
				//Deny if negative
				airspeed_d.indicated_airspeed_m_s = VaOut*0.0f;
				airspeed_d.true_airspeed_m_s = VaOut*0.0f;
			}
			/* ---- Protect system ---- */

			airspeed_d.timestamp = hrt_absolute_time();
			airspeed_estimator_dat_d.timestamp = hrt_absolute_time();
			airspeed_d.air_temperature_celsius = masm.primary_temperature;
			airspeed_d.confidence = 1;


			orb_publish(ORB_ID(airspeed), airspeed_pub, &airspeed_d); //Publish
			orb_publish(ORB_ID(airspeed_estimator_dat), airspeed_estimator_dat_pub, &airspeed_estimator_dat_d); //Publish

		}
		else
		{
			px4_usleep(50);
			// PX4_ERR("MASM: %d ---- ATT: %d ---- POS: %d", fds[1].revents,  fds[2].revents,  fds[3].revents);

		}



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

	solExist = B*B - 4.0f*A*C > 0;
	// float Va = 10.0f; //TODO should I just declare this in another scope and only write to it if its reasonable?

	if(solExist)
	{
		Va = (-B + sqrtf(B*B - 4.0f*A*C)) / (2.0f*A);

		if(Va<0)
		{
			Va = (-B - sqrtf(B*B - 4.0f*A*C)) / (2.0f*A);
		}
	}
	return Va;
}

float AirspeedEstimator::calcComp(float vaEst, float Va_w)
{
	alpha = (-tanhf(phi - phiStart - 2.0f)/2.0f + 0.5f) * (alphaStart - alphaEnd) + alphaEnd;
	float Vak = (1.0f-alpha) * (Va_w) + alpha*(vaEst);

	airspeed_estimator_dat_d.va_wind = Va_w;
	airspeed_estimator_dat_d.phi = phi;
	airspeed_estimator_dat_d.alpha = alpha;
	airspeed_estimator_dat_d.va_est = vaEst;

	return Vak;
}


float AirspeedEstimator::calcExpectAs()
{
	float via_n = pos.vx - windEst.windspeed_north;
	float via_e = pos.vy - windEst.windspeed_east;
	float via_d = pos.vz;

	matrix::Dcmf Rib = matrix::Quatf(att.q);
	matrix::Dcmf Cbi = Rib.transpose();

	float vab1 = Cbi(0,0)*via_n + Cbi(0,1)*via_e + Cbi(0,2)*via_d;

	float normVa = sqrtf(via_n*via_n + via_e*via_e + via_d*via_d);

	phi =  57.2958f * acosf(vab1/normVa);

	return vab1;

}

float AirspeedEstimator::Hfn(float Vakm1, float n)
{
	return n*(-0.001512f) + Vakm1*0.019798f + 66.0f / 125.0f;
}

float AirspeedEstimator::SlipFn(float Vakm1, float n)
{
	return p00 + p10*n + p01*Vakm1 + p20*n*n + p11*n*Vakm1 + p02*Vakm1*Vakm1;
}

float AirspeedEstimator::calcEKF(float n, float vPit)
{
	float xk_km1 = calcExpectAs();
	float Hk = Hfn(n,xk_km1);

	float Pk_km1 = Fk*Pkm1_km1*Fk + Q;

	float ykModel = SlipFn(xk_km1,n);
	float yk = vPit - ykModel;
	float Sk = Hk * Pk_km1 * Hk + R;

	float Kk = ( Pk_km1 * Hk )/ Sk;
	float xk_k = xk_km1 + Kk*yk;
	float Pk_k = (1 - Kk*Hk)*Pk_km1;



	Pkm1_km1 = Pk_k;



	airspeed_estimator_dat_d.xk_km1 = xk_km1;
	airspeed_estimator_dat_d.hk = Hk;
	airspeed_estimator_dat_d.pk_km1 = Pk_km1;
	airspeed_estimator_dat_d.ykmodel = ykModel;
	airspeed_estimator_dat_d.yk = yk;
	airspeed_estimator_dat_d.sk = Sk;
	airspeed_estimator_dat_d.kk = Kk;

	airspeed_estimator_dat_d.ekf_flag = true;

	return xk_k;

}
