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
#include "juan_reference_generator.hpp"


int juan_reference_generator::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int juan_reference_generator::custom_command(int argc, char *argv[])
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


int juan_reference_generator::task_spawn(int argc, char *argv[])
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

juan_reference_generator *juan_reference_generator::instantiate(int argc, char *argv[])
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

	juan_reference_generator *instance = new juan_reference_generator(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

juan_reference_generator::juan_reference_generator(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void juan_reference_generator::run()
{
	uint8_t _maneuver_type = 3;

	while(!should_exit())
	{
		if (_juan_manouver_start_sub.update(&_juan_man_start))
		{
			_pos_x_initial = _juan_man_start.pos_x_initial;
			_pos_y_initial = _juan_man_start.pos_y_initial;
			_pos_z_initial = _juan_man_start.pos_z_initial;

			float tStart = _juan_man_start.man_start;
			float tEnd = tStart + 100.0f; //Dummy, ensures tStart > tEnd
			_initial_heading = _juan_man_start.initial_heading;

			while(tStart < tEnd)
			{
				_time_elapsed = (float)hrt_absolute_time()/(float)1e6 - tStart;

				if (_maneuver_type == 1)
				{
				// if (time_elapsed <= time_stage1)
				// {
				float t_man = _time_elapsed;
				float Vel_track1 = 10.0f;
				_vel_x_ref = Vel_track1*cosf(_initial_heading);
				_vel_y_ref = Vel_track1*sinf(_initial_heading);
				_vel_z_ref = 0.0f;

				_pos_x_ref = _pos_x_initial+Vel_track1*cosf(_initial_heading)*t_man;
				_pos_y_ref = _pos_y_initial+Vel_track1*sinf(_initial_heading)*t_man;
				_pos_z_ref = _pos_z_initial;
				// }
				}
				else if (_maneuver_type == 2)
				{
					float t_man = _time_elapsed;
					float V_i = 5.0f;
					float t_init = 2.0f;
					float t_stop = 2.0f;
					float a_slow = -V_i/t_stop;
					if (t_man < t_init)
					{
						_vel_x_ref = V_i*cosf(_initial_heading); //straight line
						_vel_y_ref = V_i*sinf(_initial_heading);
						_vel_z_ref = 0.0f;

						_pos_x_ref = _pos_x_initial+V_i*cosf(_initial_heading)*t_man;
						_pos_y_ref = _pos_y_initial+V_i*sinf(_initial_heading)*t_man;
						_pos_z_ref = _pos_z_initial;
					}
					else if (t_man < t_init+t_stop)
					{

						float vel_mag = a_slow*(t_man-t_init)+V_i;
						_vel_x_ref = vel_mag*cosf(_initial_heading);
						_vel_y_ref = vel_mag*sinf(_initial_heading);
						_vel_z_ref = 0.0f;

						float pos_mag = a_slow/2*(t_man-t_init)*(t_man-t_init)+V_i*(t_man-t_init)+V_i*t_init;
						_pos_x_ref = pos_mag*cosf(_initial_heading)+_pos_x_initial;
						_pos_y_ref = pos_mag*sinf(_initial_heading)+_pos_y_initial;
						_pos_z_ref = _pos_z_initial;
					}
					else {
						_vel_x_ref = 0.0f;
						_vel_y_ref = 0.0f;
						_vel_z_ref = 0.0f;

						float pos_mag = a_slow/2*t_stop*t_stop+V_i*t_stop+V_i*t_init;
						_pos_x_ref = pos_mag*cosf(_initial_heading)+_pos_x_initial;
						_pos_y_ref = pos_mag*sinf(_initial_heading)+_pos_y_initial;
						_pos_z_ref = _pos_z_initial;

					}


				}
				else if (_maneuver_type == 3) //Jackson's path, zigzag
				{
					float t_man = _time_elapsed;
					float t_turns = 7.5f; //time allowed for each sucessive straight run

					float V_i = 10.0f;
					// float t_init = 2.0f;

					// float spinTime = 2.0f; //time to do 180 spin
					if (turnCount < 8 && !completeFlag)
					{
						// PX4_INFO("t_man : %f", t_man);
						if (t_man - t_last_vertex > t_turns) //if it's time to turn
						{
							_initial_heading += PI_f/2.0f; //Rotate 90
							_pos_x_last_vtx = _pos_x_ref - _pos_x_initial;
							_pos_y_last_vtx = _pos_y_ref - _pos_y_initial;
							t_last_vertex = t_man;
							turnCount++;

							PX4_INFO("turn %i", turnCount);

							if(turnCount == 4){_juan_pos_var.feedforward_on = false;} //turn ff on
						}
						_vel_x_ref = V_i*cosf(_initial_heading);
						_vel_y_ref = V_i*sinf(_initial_heading);
						_vel_z_ref = 0.0f;
						// PX4_INFO("t_man : %f", t_man);
						_pos_x_ref = _pos_x_initial + V_i*cosf(_initial_heading)*(t_man - t_last_vertex) + _pos_x_last_vtx;
						_pos_y_ref = _pos_y_initial + V_i*sinf(_initial_heading)*(t_man - t_last_vertex) + _pos_y_last_vtx;
						_pos_z_ref = _pos_z_initial;

						t_last = t_man;


					}
					else { //Go into hover
						completeFlag = true;
						if(!exitMsgSent)
						{
							PX4_INFO("Path exiting");
							exitMsgSent = true;
						}


						/* ---- just keep swimming ---- */
						_vel_x_ref = V_i*cosf(_initial_heading);
						_vel_y_ref = V_i*sinf(_initial_heading);
						_vel_z_ref = 0.0f;

						_pos_x_ref = _pos_x_initial + V_i*cosf(_initial_heading)*(t_man - t_last); //need some t_end for this
						_pos_y_ref = _pos_y_initial + V_i*sinf(_initial_heading)*(t_man - t_last);
						_pos_z_ref = _pos_z_initial;

						/* -- for resetting path -- */
						_pos_x_last_vtx = 0.0f;
						_pos_y_last_vtx = 0.0f;

						t_last_vertex = 0.0f;
						turnCount = 0;


						/* ---- reset feedforward stuff ---- */
						_juan_pos_var.feedforward_on = true;

					}

				}

				// ASSIGN TO THING AND PUBLISH
			}

		}

	}

	exit_and_cleanup();
	return;


}

void juan_reference_generator::parameters_update(bool force)
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

int juan_reference_generator::print_usage(const char *reason)
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

	PRINT_MODULE_USAGE_NAME("module", "juan_reference_generator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int juan_reference_generator_main(int argc, char *argv[])
{
	return juan_reference_generator::main(argc, argv);
}
