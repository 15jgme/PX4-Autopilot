/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file airspeed_slipstream_record.c
 * Minimal application example for PX4 autopilot
 *
 * @author Jackson Empey <jackson.empey@mail.mcgill.com>
 */

#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>

#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed_multi_record.h>
#include <uORB/topics/esc_status.h>

__EXPORT int airspeed_slipstream_record_main(int argc, char *argv[]);

int airspeed_slipstream_record_main(int argc, char *argv[])
{
	PX4_INFO("MASM started");

	/* subscribe to vehicle_acceleration topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(differential_pressure));
	/* subscribe to esc rpm topic */
	int esc_sub_fd = orb_subscribe(ORB_ID(esc_status));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);
	orb_set_interval(esc_sub_fd, 200);

	/* advertise attitude topic */
	struct airspeed_multi_record_s airspeed_multi_data;
	memset(&airspeed_multi_data, 0, sizeof(airspeed_multi_data));
	orb_advert_t att_pub = orb_advertise(ORB_ID(airspeed_multi_record), &airspeed_multi_data);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		{ .fd = esc_sub_fd,   .events = POLLIN },
	};

	int error_counter = 0;


	/* Primary sensor */
	uint sensID_1 = 4923657; //Sensor ID for primary airspeed sensor
	// uint sensID_1 = 0; //Sensor ID for primary airspeed sensor
	bool sens_1_active = true;
	struct differential_pressure_s diff_pres_ID_1; //Struct to store data for primary sensor
	float airspeed_ID_1 = 0.0f; //For storing calculated airspeed, m/s
	diff_pres_ID_1.error_count = 0;
	PX4_INFO("crap -> \t%8.4f", (double)diff_pres_ID_1.error_count);

	/* Slipstream sensor */
	uint sensID_2 = 4748809; //Sensor ID for slipstream airspeed sensor
	bool sens_2_active = true;
	struct differential_pressure_s diff_pres_ID_2; //Struct to store data for slipstream sensor
	diff_pres_ID_2.error_count = 0;
	PX4_INFO("crap -> \t%8.4f", (double)diff_pres_ID_2.error_count);
	float airspeed_ID_2 = 0.0f; //For storing calculated airspeed, m/s

	while(true)
	{
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */

				struct differential_pressure_s diff_pres;
				struct esc_status_s esc_stat;
				/* copy sensors raw data into local buffer */

				/* Copy message and assign it to device struct depending on ID */
				orb_copy(ORB_ID(differential_pressure), sensor_sub_fd, &diff_pres);
				if(diff_pres.device_id == sensID_1 && sens_1_active){
					diff_pres_ID_1 = diff_pres;
				} else if (diff_pres.device_id == sensID_2 && sens_2_active) {
					diff_pres_ID_2 = diff_pres;
					PX4_INFO("secondary");
				} else if (sens_1_active || sens_2_active) {
					PX4_ERR("Incorrect airspeed sensor ID detected, please check");
				}


				orb_copy(ORB_ID(esc_status), esc_sub_fd, &esc_stat);

				//CALCULATE AIRSPEED HERE AND PUBLISH
				airspeed_ID_1 = diff_pres.differential_pressure_filtered_pa;
				airspeed_ID_2 = 4.0f;

				airspeed_multi_data.primary_airspeed_ms = airspeed_ID_1;
				airspeed_multi_data.secondary_airspeed_ms = airspeed_ID_2;

				airspeed_multi_data.rpm_sens = esc_stat.esc[0].esc_rpm;
				// airspeed_multi_data.rpm_sens = 5;

				airspeed_multi_data.timestamp = hrt_absolute_time();

				orb_publish(ORB_ID(airspeed_multi_record), att_pub, &airspeed_multi_data);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	PX4_INFO("exiting");

	return 0;
}
