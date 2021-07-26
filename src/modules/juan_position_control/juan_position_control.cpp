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
#include "juan_position_control.hpp"


int juan_position_control::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int juan_position_control::custom_command(int argc, char *argv[])
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


int juan_position_control::task_spawn(int argc, char *argv[])
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

juan_position_control *juan_position_control::instantiate(int argc, char *argv[])
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

	juan_position_control *instance = new juan_position_control(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

juan_position_control::juan_position_control(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void juan_position_control::run()
{
	while(!should_exit())
	{
		if (_juan_reference_variables_sub.update(&_juan_ref_var))
		{
			// Not variables; Make these constants later
			float _gravity_const = 9.81f;
			float _mass_const = 0.45f;
			// Position Control Gains
			float KpX = 0.75f*0.6f*1.0f*3*0.1f*1.8f;
			float KpY = 0.75f*0.6f*1.0f*3*0.1f*1.8f;
			float KpZ =  0.75f*0.7f*6*0.1f*6.0f;
			float KdX = 0.7f*0.9f*1.0f*0.5f*0.42f;
			float KdY = 0.7f*0.9f*1.0f*0.5f*0.42f;
			float KdZ = 0.7f*0.9f*1.0f*0.5f*0.21f;
			float KiX = 0.25f*0.0008f;
			float KiY = 0.25f*0.0008f;
			float KiZ = 0.25f*0.0004f;

			/* --- ⛽ integral gains --- */
			KiX *= 0.0f;
			KiY *= 0.0f;
			KiZ *= 0.0f;

			// Added roll gains
			// float k_roll_p = 35f*0.005f*4.32f;
			float k_roll_y = 0.5f*0.7f*0.02f*0.4f;
			// float max_roll = 30.0f;
			// float k_roll_i = 0.0f*0.01f;

			// Assigned measured position and velocity
			_local_pos_sub.update(&_local_pos);
			float _pos_x_est = _local_pos.x;
			float _pos_y_est = _local_pos.y;
			float _pos_z_est = _local_pos.z;
			float _vel_x_est = _local_pos.vx;
			float _vel_y_est = _local_pos.vy;
			float _vel_z_est = _local_pos.vz;

			// Control law
			float _error_pos_x = _juan_ref_var.reference_position_x -_pos_x_est;
			float _error_pos_y = _juan_ref_var.reference_position_y-_pos_y_est;
			float _error_pos_z = _juan_ref_var.reference_position_z-_pos_z_est;

			float _error_vel_x = _juan_ref_var.reference_velocity_x -_vel_x_est;
			float _error_vel_y = _juan_ref_var.reference_velocity_y -_vel_y_est;
			float _error_vel_z = _juan_ref_var.reference_velocity_z -_vel_z_est;

			/* TODO get correct dt for this */
			// _error_x_int = _error_x_int + _error_pos_x*_delta_time_attitude;
			// _error_y_int = _error_y_int + _error_pos_y*_delta_time_attitude;
			// _error_z_int = _error_z_int + _error_pos_z*_delta_time_attitude;

			// Nose vector
			// float Fv1 = 1.0f*0.8f*0.8f*(1.3f*KdX * _error_vel_x + KpX * _error_pos_x + 0.5f*KiX * _error_x_int);
			// float Fv2 = 1.0f*0.8f*0.8f*(1.3f*KdY * _error_vel_y + KpY * _error_pos_y + 0.5f*KiY * _error_y_int);
			// float Fv3 = 1.0f*0.8f*0.8f*(1.2f*KdZ * _error_vel_z + 1.2f*KpZ * _error_pos_z + 0.3f*KiZ * _error_z_int) - 0.4f*_gravity_const;
			float Fv1 = 1.0f*0.8f*0.8f*(1.3f*KdX * _error_vel_x + KpX * _error_pos_x);
			float Fv2 = 1.0f*0.8f*0.8f*(1.3f*KdY * _error_vel_y + KpY * _error_pos_y);
			float Fv3 = 1.0f*0.8f*0.8f*(1.2f*KdZ * _error_vel_z + 1.2f*KpZ * _error_pos_z) - 0.4f*_gravity_const;

			float _norm_F = sqrtf(Fv1*Fv1+Fv2*Fv2+Fv3*Fv3);
			float fv1 = Fv1/_norm_F;
			float fv2 = Fv2/_norm_F;
			float fv3 = Fv3/_norm_F;
			// Thrust magnitude (N)
			ThrustN = _mass_const*_norm_F;
			// Non-normalized wing vector
			float Wv1 = -fv2;
			float Wv2 = fv1;
			// float Wv3 = 0.0f;

			float _vel_xy_ref = sqrtf(_juan_ref_var.reference_velocity_x*_juan_ref_var.reference_velocity_x + _juan_ref_var.reference_velocity_y*_juan_ref_var.reference_velocity_y);
			float _norm_W = sqrtf(Wv1*Wv1+Wv2*Wv2);
			float angle_test = asinf(_norm_W);
			JUAN_singularity_management(_vel_xy_ref, angle_test);
			// _control_operation_mode = 0; //remove

			if (_control_operation_mode < 1)
			{
					float wv1 = Wv1/_norm_W;
					float wv2 = Wv2/_norm_W;
					float wv3 = 0.0f;

					float proj1 = wv3*fv2-wv2*fv3;
					float proj2 = wv1*fv3-wv3*fv1;
					float proj3 = wv2*fv1-wv1*fv2;

					belly_n_old = proj1;
					belly_e_old = proj2;

					float _heading_ref = atan2f(_vel_y_ref,_vel_x_ref);

					if (_heading_ref < -PI_f)
					{
							_heading_ref = _heading_ref + 2*M_PI_f;
					}
					else if (_heading_ref > M_PI_f)
					{
							_heading_ref = _heading_ref - 2*M_PI_f;
					}
					matrix::Vector3f error_inertial(_error_pos_x, _error_pos_y, _error_pos_z);
					matrix::Vector3f error_body = C_bi*error_inertial;
					float eby = error_body(1);

					float _heading_test = atan2f(_vel_y_est,_vel_x_est);

						if (_heading_test < -M_PI_f)
						{
								_heading_test = _heading_test + 2*M_PI_f;
						}
						else if (_heading_test > M_PI_f)
						{
								_heading_test = _heading_test - 2*M_PI_f;
						}

						float heading_aux = _heading_ref + atanf(k_roll_y*eby);

						if (heading_aux < -M_PI_f)
						{
								heading_aux = heading_aux + 2*M_PI_f;
						}
						else if (heading_aux > M_PI_f)
						{
								heading_aux = heading_aux - 2*M_PI_f;
						}

						float heading_com = heading_aux-_heading_test;

						if (heading_com < -M_PI_f)
						{
								heading_com = heading_com + 2*M_PI_f;
						}
						else if (heading_com > M_PI_f)
						{
								heading_com = heading_com - 2*M_PI_f;
						}

						_error_heading_int = _error_heading_int + (heading_aux-_heading_test)*_delta_time_attitude;
						// float roll_com = 0.1f*(0.8f*k_roll_p*heading_com+0.5f*k_roll_i*_error_heading_int);
						// if (roll_com >= 0.0f)
						// {
						// 	if (roll_com > max_roll*M_PI_f/180)
						// 	{
						// 		roll_com = max_roll*M_PI_f/180;
						// 	}
						// }
						// else {
						// 	if (roll_com < -max_roll*M_PI_f/180)
						// 	{
						// 		roll_com = -max_roll*M_PI_f/180;
						// 	}
					// }
						float roll_com = 0.0f;

						float m_ba11 = fv1;
						float m_ba12 = fv2;
						float m_ba13 = fv3;

						float m_ba21 =  wv1*cosf(roll_com) + proj1*sinf(roll_com);
						float m_ba22 =  wv2*cosf(roll_com) + proj2*sinf(roll_com);
						float m_ba23 =  wv3*cosf(roll_com) + proj3*sinf(roll_com);

						float m_ba31 =  -wv1*sinf(roll_com) + proj1*cosf(roll_com);
						float m_ba32 =  -wv2*sinf(roll_com) + proj2*cosf(roll_com);
						float m_ba33 =  -wv3*sinf(roll_com) + proj3*cosf(roll_com);

						float bldr_array_cri[9] = {m_ba11,m_ba21,m_ba31,m_ba12,m_ba22,m_ba32,m_ba13,m_ba23,m_ba33};
						matrix::SquareMatrix<float, 3> Bldr_Matrix_cri(bldr_array_cri);
						matrix::Dcmf C_ref_steady (Bldr_Matrix_cri);
						C_ri_pos = C_ref_steady;
			}
			else
			{
						float nrm_old = sqrtf(belly_e_old*belly_e_old+belly_n_old*belly_n_old);

						float vexi1 = belly_n_old/nrm_old;
						float vexi2 = belly_e_old/nrm_old;

						float Wv1n = vexi2*fv3;
						float Wv2n = -vexi1*fv3;
						float Wv3n = fv2*vexi1 - fv1*vexi2;

						float _norm_Wn = sqrtf(Wv1n*Wv1n+Wv2n*Wv2n+Wv3n*Wv3n);
						float wv1 = Wv1n/_norm_Wn;
						float wv2 = Wv2n/_norm_Wn;
						float wv3 = 0.0f;

						float proj1 = wv3*fv2-wv2*fv3;
						float proj2 = wv1*fv3-wv3*fv1;
						float proj3 = wv2*fv1-wv1*fv2;

						float bldr_array_cri[9] = {fv1,wv1,proj1,fv2,wv2,proj2,fv3,wv3,proj3};
						matrix::SquareMatrix<float, 3> Bldr_Matrix_cri(bldr_array_cri);
						matrix::Dcmf C_ref_hover (Bldr_Matrix_cri); // DCM cast transposes?
						C_ri_pos = C_ref_hover;
			}

			C_ri = C_ri_pos.transpose();
			_juan_att_var.c_ri_pre_ff[0] =  C_ri(0,0);
			_juan_att_var.c_ri_pre_ff[1] =  C_ri(0,1);
			_juan_att_var.c_ri_pre_ff[2] =  C_ri(0,2);
			_juan_att_var.c_ri_pre_ff[3] =  C_ri(1,0);
			_juan_att_var.c_ri_pre_ff[4] =  C_ri(1,1);
			_juan_att_var.c_ri_pre_ff[5] =  C_ri(1,2);
			_juan_att_var.c_ri_pre_ff[6] =  C_ri(2,0);
			_juan_att_var.c_ri_pre_ff[7] =  C_ri(2,1);
			_juan_att_var.c_ri_pre_ff[8] =  C_ri(2,2);

			if (feedforward_flag)
			{
				wind_ff_rot_update(); //update R_wind
				matrix::Dcmf temp_C_ri = C_ri*R_wind; //avoid any weirdness like in Eigen
				C_ri = temp_C_ri;
			}

			matrix::Vector3f _omega_ref_temp(0.0f, 0.0f, 0.0f); // change for command filter
			_omega_reference_body = _omega_ref_temp;

			float Jar = 0.5f;

			/*..........Proper advance ratio calculation........................*/
			// float _vel_x_est = _local_pos.vx;
			// float _vel_y_est = _local_pos.vy;
			// float _vel_z_est = _local_pos.vz;
			// if (_previous_rpm < 1000)
			// {
			// 	_advance_ratio = 0.5f;
			// }
			// else
			// {
			// 	float _norm_vel =  sqrtf(_local_pos.vx*_local_pos.vx+_local_pos.vy*_local_pos.vy+_local_pos.vz*_local_pos.vz);
			// 	_advance_ratio = _norm_vel/(0.254f*(_previous_rpm/60.0f));
			// }
			// float Jar = saturate(_advance_ratio,0.0f,0.5f);
			/*..................................................................*/


			_global_jar = Jar;

			float kt = (-1.43909969f * Jar * Jar - 2.21240323f * Jar + 2.24512051f) * powf(10.0f,-7.0f);
			float omega_t = sqrtf(ThrustN/kt);
			_previous_rpm = omega_t;
			float thrust_PWM = saturate(1.6572f * powf(10.0f,-5.0f) * powf(omega_t,2.0f) + .0166f * omega_t + 1121.8f,1000.0f,2000.0f);
			_throttle_out = (thrust_PWM-1000.0f)/1000.0f;

		}

	}

	exit_and_cleanup();
	return;


}

void juan_position_control::parameters_update(bool force)
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

int juan_position_control::print_usage(const char *reason)
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

	PRINT_MODULE_USAGE_NAME("module", "juan_position_control");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int juan_position_control_main(int argc, char *argv[])
{
	return juan_position_control::main(argc, argv);
}


float juan_position_control::saturate(float value, float min, float max)
{
  float output = value;
  if (value < min){output = min;}
  if (value > max){output = max;}
  return output;
}

void juan_position_control::wind_ff_rot_update()
{
	/* ---- Wind vector ---- */
	float v_wind_N = _wind.windspeed_north;
	float v_wind_E = _wind.windspeed_east;

	// float v_wind_norm = sqrtf(v_wind_N*v_wind_N + v_wind_E*v_wind_E);

	// float v_wind_N_u = v_wind_N/v_wind_norm;
	// float v_wind_E_u = v_wind_E/v_wind_norm;

	/* ---- Velocity vector ---- */
	float v_N = _local_pos.vx;
	float v_E = _local_pos.vy;

	float v_norm = sqrtf(v_N*v_N + v_E*v_E);

	float v_N_u = v_N/v_norm;
	float v_E_u = v_E/v_norm;

	float v_crs_z_N = -v_E_u;
	float v_crs_z_E = v_N_u;

	/* ---- rotation inertial to velocity vector ---- */

	// float bldr_array_cri[9] = {m_ba11,m_ba21,m_ba31,m_ba12,m_ba22,m_ba32,m_ba13,m_ba23,m_ba33};
	float bldr_array_Cvii[9] = {v_N_u, v_E_u, 0.0f, v_crs_z_N, v_crs_z_E, 0.0f, 0.0f, 0.0f, 1.0f};
	matrix::SquareMatrix<float, 3> Bldr_Matrix_Cvii(bldr_array_Cvii);
	matrix::Dcmf Cvii (Bldr_Matrix_Cvii); // DCM cast transposes?

	/* ---- rotation inertial to relative velocity vector ---- */

	float v_tild_N = v_N - v_wind_N; //North rel vel
	float v_tild_E = v_E - v_wind_E; //East rel vel

	float v_tild_norm = sqrtf(v_tild_N*v_tild_N + v_tild_E*v_tild_E);

	float v_tild_N_u = v_tild_N / v_tild_norm;
	float v_tild_E_u = v_tild_E / v_tild_norm;

	float v_tild_crs_z_N = -v_tild_E_u;
	float v_tild_crs_z_E = v_tild_N_u;

	// float bldr_array_cri[9] = {m_ba11,m_ba21,m_ba31,m_ba12,m_ba22,m_ba32,m_ba13,m_ba23,m_ba33};
	float bldr_array_Cv_tildi[9] = {v_tild_N_u, v_tild_E_u, 0.0f, v_tild_crs_z_N, v_tild_crs_z_E, 0.0f, 0.0f, 0.0f, 1.0f};
	matrix::SquareMatrix<float, 3> Bldr_Matrix_Cv_tildi(bldr_array_Cv_tildi);
	matrix::Dcmf Cv_tildi (Bldr_Matrix_Cv_tildi); // DCM cast transposes?

	matrix::Dcmf Civ_tild = Cv_tildi.transpose();

	R_wind = Cvii * Civ_tild;

	_juan_pos_var.crab_angle_ff = acosf(  R_wind(0,0) );
	_juan_pos_var.feedforward_on = true;

	_juan_pos_var.r_wind_rows[0] =  R_wind(0,0);
	_juan_pos_var.r_wind_rows[1] =  R_wind(0,1);
	_juan_pos_var.r_wind_rows[2] =  R_wind(0,2);
	_juan_pos_var.r_wind_rows[3] =  R_wind(1,0);
	_juan_pos_var.r_wind_rows[4] =  R_wind(1,1);
	_juan_pos_var.r_wind_rows[5] =  R_wind(1,2);
	_juan_pos_var.r_wind_rows[6] =  R_wind(2,0);
	_juan_pos_var.r_wind_rows[7] =  R_wind(2,1);
	_juan_pos_var.r_wind_rows[8] =  R_wind(2,2);

	_juan_pos_var.v_tild_n = v_tild_N;
	_juan_pos_var.v_tild_e = v_tild_E;

	_juan_pos_var.v_n = v_N;
	_juan_pos_var.v_e = v_E;
}


void juan_position_control::JUAN_singularity_management(float xy_speed, float angle_vect)
{
	// controller switch thresholds
	float angle_thrs_inf = 10*0.01745f;// 15 degrees
	float angle_thrs_sup = 20*0.01745f;// 25 degrees
	float vel_thrs_inf = 2.0f; // 2 m/s
	float vel_thrs_sup = 3.0f; // 5 m/s
	// Singularity management
	if (_control_operation_mode > 0)
	{ // in singularity mode, check if tilt and speed are enough
		if (xy_speed > vel_thrs_sup)
		{
			if (angle_vect > angle_thrs_sup)
			{
				_control_operation_mode = 0;
			}
			else{
				_control_operation_mode = 1;
			}
		}
		else{
			_control_operation_mode = 1;
		}
	}
	else //in normal mode
	{
		if (xy_speed > vel_thrs_inf)
		{
			if (angle_vect > angle_thrs_inf)
			{
				_control_operation_mode = 0;
			}
			else{
				_control_operation_mode = 1;
			}
		}
		else{
				_control_operation_mode = 1;
		}
	}
}
