uint64 timestamp		# time since system start (microseconds)

float32 reference_position_x # north reference
float32 reference_position_y # east reference
float32 reference_position_z # down reference

float32 estimated_position_x # north estimated
float32 estimated_position_y # east estimated
float32 estimated_position_z # down estimated

float32 estimated_velocity_x # north vel estimated
float32 estimated_velocity_y # east vel estimated
float32 estimated_velocity_z # down vel estimated



float32 control_status # singularity management

float32 true_airspeed # true airspeed tests
float32 indicated_airspeed #indicated airspeed test
float32 j_advance #advance ratio
float32 omega_rpm #previous omega

float32 crab_angle_ff #Angle of FF crab rotation matrix (R_wind)
bool feedforward_on #true if ff is on

float32[9] r_wind_rows #Rows of R_wind_mtx

float32 v_tild_n #Relative wind north N
float32 v_tild_e #Relative wind east E

float32 v_n #Ground speed north N
float32 v_e #Ground speed east E

bool reset # flipped true to reset this module
