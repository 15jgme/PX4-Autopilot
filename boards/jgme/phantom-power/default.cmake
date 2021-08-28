# workaround for syslink parameter PARAM_DEFINE_INT32(SLNK_RADIO_ADDR2, 3890735079); // 0xE7E7E7E7
add_compile_options(-Wno-narrowing)

px4_add_board(
	PLATFORM nuttx
	VENDOR jgme
	MODEL phantom-power
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_MEMORY
	ROMFSROOT rockets
	# CONSTRAINED_FLASH
	SERIAL_PORTS
		TEL4:/dev/ttyS3
	DRIVERS
		barometer/bmp280
		# gps
		imu/bosch/bmi055/bmi055_i2c
		magnetometer/bosch/bmm150
		pwm_out
		adc/board_adc #get this working
		telemetry
	MODULES
		attitude_estimator_q
		#camera_feedback
		commander
		dataman
		ekf2
		events
		# flight_mode_manager
		#gyro_fft
		land_detector
		landing_target_estimator
		load_mon
		local_position_estimator
		logger
		mavlink
		mc_att_control
		# mc_hover_thrust_estimator
		mc_pos_control
		# mc_rate_control
		navigator
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
