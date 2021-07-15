uint64 timestamp				# time since system start (microseconds)

float32 vehicle_ias					#Indicated airspeed from sensors module (Primary airspeed)

							# PRIMARY SENSOR
uint64 primary_error_count				# Number of errors detected by driver
float32 primary_differential_pressure_raw_pa		# Raw differential pressure reading (may be negative)
float32 primary_differential_pressure_filtered_pa	# Low pass filtered differential pressure reading
float32 primary_temperature				# Temperature provided by sensor, -1000.0f if unknown
uint32 primary_device_id				# unique device ID for the sensor that does not change between power cycles
float32 primary_airspeed_ms				# Calculated airspeed for primary sensor (m/s)

							# SECONDARY SENSOR (SLIPSTREAM)
uint64 secondary_error_count				# Number of errors detected by driver
float32 secondary_differential_pressure_raw_pa		# Raw differential pressure reading (may be negative)
float32 secondary_differential_pressure_filtered_pa	# Low pass filtered differential pressure reading
float32 secondary_temperature				# Temperature provided by sensor, -1000.0f if unknown
uint32 secondary_device_id				# unique device ID for the sensor that does not change between power cycles
float32 secondary_airspeed_ms				# Calculated airspeed for secondary sensor (m/s)

							# RPM RECORD
float32 rpm_sens 							#recorded rpm
float32 air_temperature_celsius
