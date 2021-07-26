# workaround for syslink parameter PARAM_DEFINE_INT32(SLNK_RADIO_ADDR2, 3890735079); // 0xE7E7E7E7
add_compile_options(-Wno-narrowing)

px4_add_board(
	PLATFORM nuttx
	VENDOR jgme
	MODEL phantom-power
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_MEMORY
	ROMFSROOTs px4fmu_common
	CONSTRAINED_FLASH
	IO px4_io-v2_default
	# SERIAL_PORTS
	# # GPS1:/dev/ttyS0
	# 	# TEL1:/dev/ttyS1
	# 	# TEL2:/dev/ttyS2
	# 	TEL4:/dev/ttyS3
	DRIVERS
		barometer/bmp280
		# imu/bosch/bmi088/bmi088_i2c
		imu/bosch/bmi055
		# imu/bno055
		magnetometer/bosch/bmm150
		pwm_out
	MODULES
		attitude_estimator_q
		#camera_feedback
		commander
		dataman
		ekf2
		events
		flight_mode_manager
		#gyro_fft
		land_detector
		landing_target_estimator
		load_mon
		local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_hover_thrust_estimator
		mc_pos_control
		# mc_rate_control
		navigator #can change some params and disable all these modules
		rc_update
		sensors
		#temperature_compensation
	SYSTEMCMDS
		#bl_update
		dmesg
		dumpfile
		#esc_calib
		hardfault_log
		i2cdetect
		led_control
		mixer
		motor_ramp
		motor_test
		mtd
		nshterm
		param
		perf
		pwm
		reboot
		reflect
		sd_bench
		system_time
		top
		topic_listener
		tune_control
		uorb
		usb_connected
		ver
		work_queue
	)
