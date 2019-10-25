#pragma once

#include "drv_tap_esc.h"

#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>

#include <lib/cdev/CDev.hpp>
#include <mixer/mixer.h>
#include <perf/perf_counter.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/esc_status.h>

#if !defined(BOARD_TAP_ESC_MODE)
#  define BOARD_TAP_ESC_MODE 0
#endif

#if !defined(DEVICE_ARGUMENT_MAX_LENGTH)
#  define DEVICE_ARGUMENT_MAX_LENGTH 32
#endif

// uorb update rate for control groups in miliseconds
#if !defined(TAP_ESC_CTRL_UORB_UPDATE_INTERVAL)
#  define TAP_ESC_CTRL_UORB_UPDATE_INTERVAL 2  // [ms] min: 2, max: 100
#endif

class TAP_ESC : public cdev::CDev, public ModuleBase<TAP_ESC>, public ModuleParams
{
public:
	TAP_ESC(char const *const device, uint8_t channels_count);
	virtual ~TAP_ESC();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static TAP_ESC *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	virtual int init();
	virtual int ioctl(cdev::file_t *filp, int cmd, unsigned long arg);
	void cycle();

	void send_esc_outputs(const uint16_t *pwm, const uint8_t motor_cnt);

private:
	char 			_device[DEVICE_ARGUMENT_MAX_LENGTH];
	int 			_uart_fd = -1;
	static const uint8_t 	_device_mux_map[TAP_ESC_MAX_MOTOR_NUM];
	static const uint8_t 	_device_dir_map[TAP_ESC_MAX_MOTOR_NUM];
	bool 			_is_armed = false;
	int			_armed_sub = -1;
	int 			_test_motor_sub = -1;
	int 			_params_sub = -1;
	orb_advert_t        	_outputs_pub = nullptr;
	actuator_outputs_s      _outputs = {};
	actuator_armed_s	_armed = {};

	perf_counter_t	_perf_control_latency;

	int			_control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	actuator_controls_s 	_controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	orb_id_t		_control_topics[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	px4_pollfd_struct_t	_poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	unsigned		_poll_fds_num = 0;

	orb_advert_t      _esc_feedback_pub = nullptr;
	orb_advert_t      _to_mixer_status = nullptr; 	///< mixer status flags
	esc_status_s      _esc_feedback = {};
	uint8_t    	  _channels_count = 0; 		///< nnumber of ESC channels
	uint8_t 	  _responding_esc = 0;

	MixerGroup	*_mixers = nullptr;
	uint32_t	_groups_required = 0;
	uint32_t	_groups_subscribed = 0;
	ESC_UART_BUF 	_uartbuf = {};
	EscPacket 	_packet = {};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MC_AIRMODE>) _airmode   ///< multicopter air-mode
	)

	void subscribe();
	static int control_callback_trampoline(uintptr_t handle,
					       uint8_t control_group, uint8_t control_index, float &input);
	inline int control_callback(uint8_t control_group, uint8_t control_index, float &input);
};
