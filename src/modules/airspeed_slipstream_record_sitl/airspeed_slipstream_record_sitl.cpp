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
#include "airspeed_slipstream_record_sitl.hpp"


int airspeed_slipstream_record_sitl::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int airspeed_slipstream_record_sitl::custom_command(int argc, char *argv[])
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


int airspeed_slipstream_record_sitl::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_MIN,
				      1676,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

airspeed_slipstream_record_sitl *airspeed_slipstream_record_sitl::instantiate(int argc, char *argv[])
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
			// example_param = (int)strtol(myoptarg, nullptr, 10);
			example_flag = true;
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

	airspeed_slipstream_record_sitl *instance = new airspeed_slipstream_record_sitl(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

airspeed_slipstream_record_sitl::airspeed_slipstream_record_sitl(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void airspeed_slipstream_record_sitl::run()
{

	/* Initialize */

	/* Init first struct */
	diff_pres_ID_1.timestamp = 0;				// time since system start (microseconds)
	diff_pres_ID_1.error_count = 0;				// Number of errors detected by driver
	diff_pres_ID_1.differential_pressure_raw_pa = 0;	// Raw differential pressure reading (may be negative)
	diff_pres_ID_1.differential_pressure_filtered_pa = 0;	// Low pass filtered differential pressure reading
	diff_pres_ID_1.temperature = 0;				// Temperature provided by sensor, -1000.0f if unknown
	diff_pres_ID_1.device_id = 0;				// unique device ID for the sensor that does not change between power cycles

	/* Init second struct */
	diff_pres_ID_2.timestamp = 0;				// time since system start (microseconds)
	diff_pres_ID_2.error_count = 0;				// Number of errors detected by driver
	diff_pres_ID_2.differential_pressure_raw_pa = 0;	// Raw differential pressure reading (may be negative)
	diff_pres_ID_2.differential_pressure_filtered_pa = 0;	// Low pass filtered differential pressure reading
	diff_pres_ID_2.temperature = 0;				// Temperature provided by sensor, -1000.0f if unknown
	diff_pres_ID_2.device_id = 0;

	if (!calib_flag) //not calibrated
	{
		calib_flag = diff_pressure_calib(); //calibrate
	}

	/* advertise attitude topic */
	struct airspeed_multi_record_s airspeed_multi_data;
	memset(&airspeed_multi_data, 0, sizeof(airspeed_multi_data));
	orb_advert_t att_pub = orb_advertise(ORB_ID(airspeed_multi_record), &airspeed_multi_data);
			// Example: run the loop synchronized to the sensor_combined topic publication
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined)); // OOP way --> https://github.com/PX4/PX4-Autopilot/blob/master/src/modules/mavlink/mavlink_receiver.h

	/* subscribe to diff pressure topic */
	int sensor_sub_fd[2] = {};
	sensor_sub_fd[0] = orb_subscribe_multi(ORB_ID(differential_pressure), 0);
	sensor_sub_fd[1] = orb_subscribe_multi(ORB_ID(differential_pressure), 1);

	/* subscribe to esc rpm topic */
	int esc_sub_fd = orb_subscribe(ORB_ID(esc_status));
	/* subscribe to airspeed topic */
	int asp_sub_fd = orb_subscribe(ORB_ID(airspeed));
	/* subscribe to rc topic */
	int rc_sub_fd = orb_subscribe(ORB_ID(rc_channels));
	/* subscribe to airdata topic */
	int airdat_sub_fd = orb_subscribe(ORB_ID(vehicle_air_data));



	// /* subscribe to vehicle_acceleration topic */
	// int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_acceleration));
	// /* limit the update rate to 5 Hz */
	// orb_set_interval(sensor_sub_fd, 200);


	// Subscriptions
	// orb_set_interval(_parameter_update_sub, 50);
	orb_set_interval(sensor_sub_fd[0], 50);
	orb_set_interval(sensor_sub_fd[1], 50);
	orb_set_interval(esc_sub_fd, 50);
	orb_set_interval(asp_sub_fd, 50);
	orb_set_interval(rc_sub_fd, 500);
	orb_set_interval(airdat_sub_fd, 50);

	// uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 50}; // PARAMS
	// uORB::SubscriptionInterval sensor_sub_fd{ORB_ID(differential_pressure), 50}; //DIFF PRESSURE
	// uORB::SubscriptionInterval esc_sub_fd{ORB_ID(esc_status), 50}; //ESC
	// uORB::SubscriptionInterval asp_sub_fd{ORB_ID(airspeed), 50}; //AIRSPEED
	// uORB::SubscriptionInterval rc_sub_fd{ORB_ID(rc_channels), 50}; //RC
	// uORB::SubscriptionInterval airdat_sub_fd{ORB_ID(vehicle_air_data), 50}; //Airdata



	px4_pollfd_struct_t fds[] = {
		// { .fd = sensor_combined_sub,   .events = POLLIN },
		{ .fd = sensor_sub_fd[0],   .events = POLLIN },
		// { .fd = sensor_sub_fd[1],   .events = POLLIN },
		// { .fd = asp_sub_fd,   .events = POLLIN },
		// { .fd = rc_sub_fd,   .events = POLLIN },
		{ .fd = airdat_sub_fd, .events = POLLIN},
		{ .fd = esc_sub_fd,   .events = POLLIN },
	};


	// initialize parameters
	parameters_update(true);

	// param_get(param_find("air_cmodel"), &air_cmodel);
	// param_get(param_find("air_tube_length"), &air_tube_length);
	// param_get(param_find("air_tube_diameter_mm"), &air_tube_diameter_mm);

	// PX4_INFO("cmode -> %i",air_cmodel);

	enum AIRSPEED_SENSOR_MODEL smodel_1;

	switch ((sensID_1 >> 16) & 0xFF) {
		case DRV_DIFF_PRESS_DEVTYPE_SDP31:

		/* fallthrough */
		case DRV_DIFF_PRESS_DEVTYPE_SDP32:

		/* fallthrough */
		case DRV_DIFF_PRESS_DEVTYPE_SDP33:
			/* fallthrough */
			smodel_1 = AIRSPEED_SENSOR_MODEL_SDP3X;
			break;

		default:
			smodel_1 = AIRSPEED_SENSOR_MODEL_MEMBRANE;
			break;
	}

	// enum AIRSPEED_SENSOR_MODEL smodel_2;

	// switch ((sensID_2 >> 16) & 0xFF) {
	// 	case DRV_DIFF_PRESS_DEVTYPE_SDP31:

	// 	/* fallthrough */
	// 	case DRV_DIFF_PRESS_DEVTYPE_SDP32:

	// 	/* fallthrough */
	// 	case DRV_DIFF_PRESS_DEVTYPE_SDP33:
	// 		/* fallthrough */
	// 		smodel_2 = AIRSPEED_SENSOR_MODEL_SDP3X;
	// 		break;

	// 	default:
	// 		smodel_2 = AIRSPEED_SENSOR_MODEL_MEMBRANE;
	// 		break;
	// }

	while (!should_exit()) {
		px4_usleep(1);

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, 3, 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & fds[1].revents  & fds[2].revents & POLLIN) {


			if(true)	//RECORD!
			{
				orb_copy(ORB_ID(differential_pressure), sensor_sub_fd[0], &diff_pres_A);
				orb_copy(ORB_ID(differential_pressure), sensor_sub_fd[1], &diff_pres_B);

				/* --------------------- Sensor 1 assignment --------------------*/
				if(diff_pres_A.device_id == sensID_1 && sens_1_active){
					diff_pres_ID_1 = diff_pres_A;
					// PX4_INFO("sens1 found");
				} else if (diff_pres_B.device_id == sensID_1 && sens_1_active) {
					diff_pres_ID_1 = diff_pres_B;
					// PX4_INFO("sens1 found");
				}

				/* --------------------- Sensor 2 assignment --------------------*/
				if(diff_pres_A.device_id == sensID_2 && sens_2_active){
					diff_pres_ID_2 = diff_pres_A;
					// PX4_INFO("sens2 found");
				} else if (diff_pres_B.device_id == sensID_2 && sens_2_active) {
					diff_pres_ID_2 = diff_pres_B;
					// PX4_INFO("sens2 found");
				}


				orb_copy(ORB_ID(esc_status), esc_sub_fd, &esc_stat);
				orb_copy(ORB_ID(airspeed), asp_sub_fd, &airspeed);
				orb_copy(ORB_ID(vehicle_air_data), airdat_sub_fd, &airdat);

				//Put system IAS in message
				airspeed_multi_data.vehicle_ias = airspeed.indicated_airspeed_m_s; //Set airspeed from system to msg airspeed (for faster logging)

				/* <------------------------->SENSOR 1<--------------------->*/
				airspeed_multi_data.primary_differential_pressure_filtered_pa = diff_pres_ID_1.differential_pressure_filtered_pa + ID_1_cal;
				airspeed_multi_data.primary_differential_pressure_raw_pa = diff_pres_ID_1.differential_pressure_raw_pa + ID_1_cal;
				airspeed_multi_data.primary_temperature = diff_pres_ID_1.temperature;
				airspeed_multi_data.primary_device_id = diff_pres_ID_1.device_id;


				air_temperature_1_celsius = (diff_pres_ID_1.temperature > -300.0f) ? diff_pres_ID_1.temperature :
									(airdat.baro_temp_celcius - PCB_TEMP_ESTIMATE_DEG);

				airspeed_multi_data.air_temperature_celsius = air_temperature_1_celsius;


				// Finite check
				airspeed_ID_1  = calc_IAS_corrected((enum AIRSPEED_COMPENSATION_MODEL)
										2,
										smodel_1, 0.16f, 1.4f,
										diff_pres_ID_1.differential_pressure_filtered_pa + ID_1_cal, airdat.baro_pressure_pa,
										air_temperature_1_celsius);
				if(PX4_ISFINITE(airspeed_ID_1 )){
					airspeed_multi_data.primary_airspeed_ms = airspeed_ID_1;
				}


				/* <------------------------->SENSOR 2<--------------------->*/
				airspeed_multi_data.secondary_differential_pressure_filtered_pa = 0.0f;
				airspeed_multi_data.secondary_differential_pressure_raw_pa = 0.0f;
				airspeed_multi_data.secondary_temperature = 0.0f;
				airspeed_multi_data.secondary_device_id = 0;

				// air_temperature_2_celsius = (diff_pres_ID_2.temperature > -300.0f) ? diff_pres_ID_2.temperature :
				// 					(airdat.baro_temp_celcius - PCB_TEMP_ESTIMATE_DEG);

				// airspeed_ID_2 = calc_IAS_corrected((enum AIRSPEED_COMPENSATION_MODEL)
				// 						air_cmodel,
				// 						smodel_2, air_tube_length, air_tube_diameter_mm,
				// 						(diff_pres_ID_2.differential_pressure_filtered_pa + ID_2_cal), airdat.baro_pressure_pa,
				// 						air_temperature_2_celsius);
				// if(PX4_ISFINITE(airspeed_ID_2)){
					airspeed_multi_data.secondary_airspeed_ms = 0.0f;
				// }

				/* <--------------------------->ESC<-----------------------> */
				airspeed_multi_data.rpm_sens = esc_stat.esc[0].esc_rpm;


				/* <------------------------>CALIBRATIOn<------------------> */
				airspeed_multi_data.primary_calibrated = sensID_1_cal_flag;
				airspeed_multi_data.secondary_calibrated = 0;
				airspeed_multi_data.primary_calib_offset_pa = ID_1_cal;
				airspeed_multi_data.secondary_calib_offset_pa = 0;



				airspeed_multi_data.timestamp = hrt_absolute_time(); //Set timestamp

				orb_publish(ORB_ID(airspeed_multi_record), att_pub, &airspeed_multi_data); //Publish
			}else{
				px4_usleep(1000000); //sleep for one second
			}

		}

	}
	orb_unsubscribe(sensor_sub_fd[0]);
	orb_unsubscribe(sensor_sub_fd[1]);
	orb_unsubscribe(esc_sub_fd);
	orb_unsubscribe(asp_sub_fd);
	orb_unsubscribe(rc_sub_fd);
	orb_unsubscribe(sensor_combined_sub);
}

void airspeed_slipstream_record_sitl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int airspeed_slipstream_record_sitl::print_usage(const char *reason)
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

	PRINT_MODULE_USAGE_NAME("module", "airspeed_slipstream_record_sitl");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int airspeed_slipstream_record_sitl_main(int argc, char *argv[])
{
	return airspeed_slipstream_record_sitl::main(argc, argv);
}

int airspeed_slipstream_record_sitl::diff_pressure_calib()
{

	/* subscribe to diff pressure topic */
	int sensor_sub_fd[2] = {};
	sensor_sub_fd[0] = orb_subscribe_multi(ORB_ID(differential_pressure), 0);
	sensor_sub_fd[1] = orb_subscribe_multi(ORB_ID(differential_pressure), 1);

	orb_set_interval(sensor_sub_fd[0], 50);
	orb_set_interval(sensor_sub_fd[1], 50);

	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd[0],   .events = POLLIN },
		// { .fd = sensor_sub_fd[1],   .events = POLLIN },
	};

	int i = 0;
	while(i < maxSamp)
	{
		int pret = px4_poll(fds, 1, 1000);
		/* update uorb messages */
		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			orb_copy(ORB_ID(differential_pressure), sensor_sub_fd[0], &diff_pres_A);
			// orb_copy(ORB_ID(differential_pressure), sensor_sub_fd[1], &diff_pres_B);

			/*--------- Sort sensors ---------*/
			/* --------------------- Sensor 1 assignment --------------------*/
			if(diff_pres_A.device_id == sensID_1 && sens_1_active){
				diff_pres_ID_1 = diff_pres_A;
				// PX4_INFO("sens1 found");
			} else if (diff_pres_B.device_id == sensID_1 && sens_1_active) {
				diff_pres_ID_1 = diff_pres_B;
				// PX4_INFO("sens1 found");
			}
			// /* --------------------- Sensor 2 assignment --------------------*/
			// if(diff_pres_A.device_id == sensID_2 && sens_2_active){
			// 	diff_pres_ID_2 = diff_pres_A;
			// 	// PX4_INFO("sens2 found");
			// } else if (diff_pres_B.device_id == sensID_2 && sens_2_active) {
			// 	diff_pres_ID_2 = diff_pres_B;
			// 	// PX4_INFO("sens2 found");
			// }

			ID_1_cal -= diff_pres_ID_1.differential_pressure_filtered_pa / maxSamp;
			// ID_2_cal -= diff_pres_ID_2.differential_pressure_filtered_pa / maxSamp;

			i++;

		}

	}

	sensID_1_cal_flag = true;
	sensID_2_cal_flag = true;

	orb_unsubscribe(sensor_sub_fd[0]);
	orb_unsubscribe(sensor_sub_fd[1]);

	PX4_INFO("Differential pressure calibration complete");

	return 1;


}
