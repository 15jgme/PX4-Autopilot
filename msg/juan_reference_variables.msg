uint64 timestamp		# time since system start (microseconds)

float32 reference_position_x # north reference
float32 reference_position_y # east reference
float32 reference_position_z # down reference

float32 reference_velocity_x # north vel estimated
float32 reference_velocity_y # east vel estimated
float32 reference_velocity_z # down vel estimated

uint8 reference_path # int representing the path of the manouver

bool reset # flipped true to reset this module
