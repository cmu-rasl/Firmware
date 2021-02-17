
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v4
	LABEL rismin
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT px4fmu_common
	TESTING
	UAVCAN_INTERFACES 1

	SERIAL_PORTS
		GPS1:/dev/ttyS3
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2

	DRIVERS
		adc
    dshot
    #imu # all available imu drivers
    imu/mpu6000
		uavcan
    rc_input

	MODULES
    mocap_control
    sensors

	SYSTEMCMDS
		bl_update
		config
		dumpfile
		hardfault_log
		mixer
		mtd
		nshterm
		param
		perf
		reboot
		reflect
		shutdown
		top
		topic_listener
		usb_connected
		ver
		work_queue

	EXAMPLES
)
