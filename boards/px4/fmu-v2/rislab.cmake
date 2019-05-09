
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v2
	LABEL rislab
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT px4fmu_common
	BOOTLOADER ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/extras/px4fmuv3_bl.bin
	IO px4_io-v2_default

	SERIAL_PORTS
		GPS1:/dev/ttyS3
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		TEL4:/dev/ttyS6

	DRIVERS
		barometer/ms5611
		imu/l3gd20
		imu/lsm303d
		imu/mpu6000
		lights/rgbled
		pwm_input
		px4fmu
		px4io
		stm32
		stm32/adc
		stm32/tone_alarm
		tone_alarm

	MODULES
		attitude_estimator_q
		dataman
		events
		mavlink
    mocap_control
    mocap_status_monitor
		sensors

	SYSTEMCMDS
    config
		mtd
    nshterm
		param
		perf
		pwm
		reboot
		top
		tune_control
		ver

	EXAMPLES
		#bottle_drop # OBC challenge
		#fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		#hello
		#hwtest # Hardware test
		#matlab_csv_serial
		#position_estimator_inav
		#px4_mavlink_debug # Tutorial code from https://px4.io/dev/debug_values
		#px4_simple_app # Tutorial code from https://px4.io/dev/px4_simple_app
		#rover_steering_control # Rover example app
		#segway
		#uuv_example_app
	)
