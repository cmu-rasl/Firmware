include(nuttx/px4_impl_nuttx)

px4_nuttx_configure(HWCLASS m4 CONFIG nsh ROMFS y ROMFSROOT px4fmu_common)

set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-none-eabi.cmake)

set(config_uavcan_num_ifaces 2)

set(config_module_list
	#
	# Board support modules
	#
	drivers/device
	drivers/stm32
	drivers/stm32/adc
	drivers/stm32/tone_alarm
	drivers/led
	drivers/px4fmu
	drivers/px4io
	drivers/boards
	drivers/rgbled

	drivers/l3gd20  # ST Micro 16 bit gyro
	drivers/lsm303d # ST Micro 14 bit accel / mag
	drivers/mpu6000 # Invensense 3-axis accel / gyro
	drivers/ms5611  # MEAS barometer

	modules/sensors
	drivers/pwm_input

	#
	# System commands
	#
	systemcmds/bl_update
	systemcmds/config
	systemcmds/dumpfile
	systemcmds/mixer
	systemcmds/mtd
	systemcmds/nshterm
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/reboot
	systemcmds/top
	systemcmds/ver

	#
	# General system control
	#
	modules/mavlink

	#
	# Estimation modules (EKF/ SO3 / other filters)
	#
	modules/attitude_estimator_q

	#
	# Vehicle Control
	#

	modules/mocap_control
	modules/mocap_status_monitor

	#
	# Logging
	#
	modules/sdlog2

	#
	# Library modules
	#
	modules/dataman
	modules/systemlib
	modules/systemlib/param
	modules/uORB

	#
	# Libraries
	#
	lib/controllib
	lib/conversion
	lib/DriverFramework/framework
	lib/ecl
	lib/geo
	lib/geo_lookup
	lib/led
	lib/mathlib
	lib/mathlib/math/filter
	lib/mixer
	lib/version


	platforms/common
	platforms/nuttx
	platforms/nuttx/px4_layer
)

set(config_extra_builtin_cmds
	serdis
	sercon
	)

set(config_io_board
	px4io-v2
	)

add_custom_target(sercon)
set_target_properties(sercon PROPERTIES
	PRIORITY "SCHED_PRIORITY_DEFAULT"
	MAIN "sercon" STACK_MAIN "2048")

add_custom_target(serdis)
set_target_properties(serdis PROPERTIES
	PRIORITY "SCHED_PRIORITY_DEFAULT"
	MAIN "serdis" STACK_MAIN "2048")
