/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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

/**
 * @file blinkm.cpp
 *
 * Driver for the BlinkM LED controller connected via I2C.
 *
 * Connect the BlinkM to I2C3 and put the following line to the rc startup-script:
 * blinkm start
 *
 * To start the system monitor put in the next line after the blinkm start:
 * blinkm systemmonitor
 *
 *
 * Description:
 * After startup, the Application checked how many lipo cells are connected to the System.
 * The recognized number off cells, will be blinked 5 times in purple color.
 * 2 Cells = 2 blinks
 * ...
 * 6 Cells = 6 blinks
 * Now the Application will show the actual selected Flightmode, GPS-Fix and Battery Warnings and Alerts.
 *
 * System disarmed and safe:
 * The BlinkM should light solid cyan.
 *
 * System safety off but not armed:
 * The BlinkM should light flashing orange
 *
 * System armed:
 * One message is made of 4 Blinks and a pause in the same length as the 4 blinks.
 *
 * X-X-X-X-_-_-_-_-_-_-
 * -------------------------
 * G G G M
 * P P P O
 * S S S D
 *       E
 *
 * (X = on, _=off)
 *
 * The first 3 blinks indicates the status of the GPS-Signal (red):
 * 0-4 satellites = X-X-X-X-X-_-_-_-_-_-
 *   5 satellites = X-X-_-X-X-_-_-_-_-_-
 *   6 satellites = X-_-_-X-X-_-_-_-_-_-
 * >=7 satellites = _-_-_-X-X-_-_-_-_-_-
 * If no GPS is found the first 3 blinks are white
 *
 * The fourth Blink indicates the Flightmode:
 * MANUAL     : amber
 * STABILIZED : yellow
 * HOLD		  : blue
 * AUTO       : green
 *
 * Battery Warning (low Battery Level):
 * Continuously blinking in yellow X-X-X-X-X-X-X-X-X-X
 *
 * Battery Alert (critical Battery Level)
 * Continuously blinking in red X-X-X-X-X-X-X-X-X-X
 *
 * General Error (no uOrb Data)
 * Continuously blinking in white X-X-X-X-X-X-X-X-X-X
 *
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>
#include <poll.h>

#include <px4_workqueue.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <board_config.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_blinkm.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/blinkm_control.h>
#include <uORB/topics/sensor_accel.h>

static const float MAX_CELL_VOLTAGE	= 4.3f;
static const int LED_ONTIME = 120;
static const int LED_OFFTIME = 120;
static const int LED_BLINK = 1;
static const int LED_NOBLINK = 0;

class BlinkM : public device::I2C
{
public:
	BlinkM(int bus, int blinkm, const char * path);
	virtual ~BlinkM();


	virtual int		init();
	virtual int		probe();
	virtual int 	address();
	virtual int		setMode(int mode);
	virtual int		ioctl(device::file_t *filp, int cmd, unsigned long arg);


	static const char	*const script_names[];

private:
	enum ScriptID {
		USER		= 0,
		RGB,
		WHITE_FLASH,
		RED_FLASH,
		GREEN_FLASH,
		BLUE_FLASH,
		CYAN_FLASH,
		MAGENTA_FLASH,
		YELLOW_FLASH,
		BLACK,
		HUE_CYCLE,
		MOOD_LIGHT,
		VIRTUAL_CANDLE,
		WATER_REFLECTIONS,
		OLD_NEON,
		THE_SEASONS,
		THUNDERSTORM,
		STOP_LIGHT,
		MORSE_CODE
	};

	enum ledColors {
		LED_OFF,
		LED_RED,
		LED_ORANGE,
		LED_YELLOW,
		LED_PURPLE,
		LED_GREEN,
		LED_BLUE,
		LED_CYAN,
		LED_WHITE,
		LED_AMBER
	};

	work_s			_work;

	int led_color_1;
	int led_color_2;
	int led_color_3;
	int led_color_4;
	int led_color_5;
	int led_color_6;
	int led_color_7;
	int led_color_8;
	int led_blink;
	int brightness;
	int led_r;
	int led_g;
	int led_b;

	bool systemstate_run;

	int vehicle_status_sub_fd;
	int vehicle_control_mode_sub_fd;
	int vehicle_gps_position_sub_fd;
	int actuator_armed_sub_fd;
	int safety_sub_fd;
	int blinkm_control_sub_fd;
	int sensor_accel_sub_fd;

	int num_of_cells;
	int detected_cells_runcount;

	int t_led_color[8];
	int t_led_blink;
	int led_thread_runcount;
	int led_interval;

	bool topic_initialized;
	bool detected_cells_blinked;
	bool led_thread_ready;

	int num_of_used_sats;

	void 			setLEDColor(int ledcolor);
	static void		led_trampoline(void *arg);
	void			led();

	int			set_rgb(uint8_t r, uint8_t g, uint8_t b);

	int			fade_rgb(uint8_t r, uint8_t g, uint8_t b);
	int			fade_hsb(uint8_t h, uint8_t s, uint8_t b);

	int			fade_rgb_random(uint8_t r, uint8_t g, uint8_t b);
	int			fade_hsb_random(uint8_t h, uint8_t s, uint8_t b);

	int			set_fade_speed(uint8_t s);

	int			play_script(uint8_t script_id);
	int			play_script(const char *script_name);
	int			stop_script();

	int			write_script_line(uint8_t line, uint8_t ticks, uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3);
	int			read_script_line(uint8_t line, uint8_t &ticks, uint8_t cmd[4]);
	int			set_script(uint8_t length, uint8_t repeats);

	int			get_rgb(uint8_t &r, uint8_t &g, uint8_t &b);

	int			get_firmware_version(uint8_t version[2]);

	int 	 	get_blinkm_address(uint8_t *addr);

};

/* for now, we only support one BlinkM */
namespace
{
BlinkM *g_blinkm;
BlinkM *blinkm_multi[10]={nullptr};
}



/* list of script names, must match script ID numbers */
const char *const BlinkM::script_names[] = {
	"USER",
	"RGB",
	"WHITE_FLASH",
	"RED_FLASH",
	"GREEN_FLASH",
	"BLUE_FLASH",
	"CYAN_FLASH",
	"MAGENTA_FLASH",
	"YELLOW_FLASH",
	"BLACK",
	"HUE_CYCLE",
	"MOOD_LIGHT",
	"VIRTUAL_CANDLE",
	"WATER_REFLECTIONS",
	"OLD_NEON",
	"THE_SEASONS",
	"THUNDERSTORM",
	"STOP_LIGHT",
	"MORSE_CODE",
	nullptr
};


extern "C" __EXPORT int blinkm_main(int argc, char *argv[]);

BlinkM::BlinkM(int bus, int blinkm, const char *path) :
	I2C("blinkm", path, bus, blinkm
#ifdef __PX4_NUTTX
	    , 100000
#endif
	   ),
	led_color_1(LED_OFF),
	led_color_2(LED_OFF),
	led_color_3(LED_OFF),
	led_color_4(LED_OFF),
	led_color_5(LED_OFF),
	led_color_6(LED_OFF),
	led_color_7(LED_OFF),
	led_color_8(LED_OFF),
	led_blink(LED_NOBLINK),
	brightness(LED_OFF),
	led_r(LED_OFF),
	led_g(LED_OFF),
	led_b(LED_OFF),
	systemstate_run(false),
	vehicle_status_sub_fd(-1),
	vehicle_control_mode_sub_fd(-1),
	vehicle_gps_position_sub_fd(-1),
	actuator_armed_sub_fd(-1),
	safety_sub_fd(-1),
	sensor_accel_sub_fd(-1),
	num_of_cells(0),
	detected_cells_runcount(0),
	t_led_color{0},
	t_led_blink(0),
	led_thread_runcount(0),
	led_interval(1000),
	topic_initialized(false),
	detected_cells_blinked(false),
	led_thread_ready(true),
	num_of_used_sats(0)
{
	memset(&_work, 0, sizeof(_work));
}

BlinkM::~BlinkM()
{
}

int
BlinkM::init()
{
	int ret;
	ret = I2C::init();

	if (ret != OK) {
		warnx("I2C init failed");
		return ret;
	}

	stop_script();
	set_rgb(0, 0, 0);

	return OK;
}

int
BlinkM::setMode(int mode)
{
	if (mode == 1) {
		if (systemstate_run == false) {
			stop_script();
			set_rgb(0, 0, 0);
			systemstate_run = true;
			work_queue(LPWORK, &_work, (worker_t)&BlinkM::led_trampoline, this, 1);
		}

	} else {
		systemstate_run = false;
	}

	return OK;
}

int
BlinkM::probe()
{
	uint8_t version[2];
	int ret;

	ret = get_firmware_version(version);

	if (ret == OK) {
		DEVICE_DEBUG("found BlinkM firmware version %c%c", version[1], version[0]);
	}

	return ret;
}

int
BlinkM::address()
{
	uint8_t addr;
	int ret;

	ret = get_blinkm_address(&addr);

	if (ret == OK) {
		DEVICE_DEBUG("found BlinkM address %d", addr);
	}

	return ret;
}

int
BlinkM::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = ENOTTY;

	switch (cmd) {
	case BLINKM_PLAY_SCRIPT_NAMED:
		if (arg == 0) {
			ret = EINVAL;
			break;
		}

		ret = play_script((const char *)arg);
		break;

	case BLINKM_PLAY_SCRIPT:
		ret = play_script(arg);
		break;

	case BLINKM_SET_USER_SCRIPT: {
			if (arg == 0) {
				ret = EINVAL;
				break;
			}

			unsigned lines = 0;
			const uint8_t *script = (const uint8_t *)arg;

			while ((lines < 50) && (script[1] != 0)) {
				ret = write_script_line(lines, script[0], script[1], script[2], script[3], script[4]);

				if (ret != OK) {
					break;
				}

				script += 5;
			}

			if (ret == OK) {
				ret = set_script(lines, 0);
			}

			break;
		}
          case BLINKM_SET_MODE:
          {
            switch (arg)
            {
              case 0: puts("not listening"); break;
              case 1: puts("listening"); break;
            }

            ret = 0;
            break;
          }
          case BLINKM_SET_INTENSITY:
          {
            printf("BLINKM_SET_INTENSITY = %lu\n", arg);
            break;
          }
    case BLINKM_SET_COLOR:
    {
    	setLEDColor(arg);
    	break;
    }
    case BLINKM_SET_RGB:
    {
    	int r = arg & 255;
    	int g = (arg >> 8) & 255;
    	int b = (arg >> 16) & 255;
    	set_rgb(r,g,b);
    	break;
    }

	default:
		break;
	}

	return ret;
}


void
BlinkM::led_trampoline(void *arg)
{
	BlinkM *bm = (BlinkM *)arg;

	bm->led();
}



void
BlinkM::led()
{

	if (!topic_initialized) {
		vehicle_status_sub_fd = orb_subscribe(ORB_ID(vehicle_status));
		orb_set_interval(vehicle_status_sub_fd, 250);

		vehicle_control_mode_sub_fd = orb_subscribe(ORB_ID(vehicle_control_mode));
		orb_set_interval(vehicle_control_mode_sub_fd, 250);

		vehicle_gps_position_sub_fd = orb_subscribe(ORB_ID(vehicle_gps_position));
		orb_set_interval(vehicle_gps_position_sub_fd, 250);

		blinkm_control_sub_fd = orb_subscribe(ORB_ID(blinkm_control));
		orb_set_interval(blinkm_control_sub_fd, 250);

		sensor_accel_sub_fd = orb_subscribe(ORB_ID(sensor_accel));
		//orb_set_interval(sensor_accel_sub_fd, 50);

		topic_initialized = true;
	}
		/* obtained data for the first file descriptor */
	struct vehicle_status_s vehicle_status_raw;
	struct vehicle_control_mode_s vehicle_control_mode;
	struct vehicle_gps_position_s vehicle_gps_position_raw;
	struct blinkm_control_s blk;
	struct sensor_accel_s sc;

	memset(&vehicle_status_raw, 0, sizeof(vehicle_status_raw));
	memset(&vehicle_gps_position_raw, 0, sizeof(vehicle_gps_position_raw));
	memset(&blk, 0, sizeof(blk));
		//memset(&sc, 0, sizeof(sc));

	bool new_data_vehicle_status;
	bool new_data_vehicle_control_mode;
	bool new_data_vehicle_gps_position;

	orb_check(vehicle_status_sub_fd, &new_data_vehicle_status);

	int no_data_vehicle_status = 0;
	int no_data_vehicle_control_mode = 0;
	int no_data_vehicle_gps_position = 0;

	if (new_data_vehicle_status) {
		orb_copy(ORB_ID(vehicle_status), vehicle_status_sub_fd, &vehicle_status_raw);
		no_data_vehicle_status = 0;

	} else {
	no_data_vehicle_status++;

	if (no_data_vehicle_status >= 3) {
		no_data_vehicle_status = 3;
	}
	}

	orb_check(vehicle_control_mode_sub_fd, &new_data_vehicle_control_mode);

	if (new_data_vehicle_control_mode) {
		orb_copy(ORB_ID(vehicle_control_mode), vehicle_control_mode_sub_fd, &vehicle_control_mode);
		no_data_vehicle_control_mode = 0;

	} else {
		no_data_vehicle_control_mode++;

		if (no_data_vehicle_control_mode >= 3) {
			no_data_vehicle_control_mode = 3;
		}
	}

	orb_check(vehicle_gps_position_sub_fd, &new_data_vehicle_gps_position);

	if (new_data_vehicle_gps_position) {
		orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub_fd, &vehicle_gps_position_raw);
		no_data_vehicle_gps_position = 0;

	} else {
		no_data_vehicle_gps_position++;

		if (no_data_vehicle_gps_position >= 3) {
			no_data_vehicle_gps_position = 3;
		}
	}

	orb_copy(ORB_ID(blinkm_control), blinkm_control_sub_fd, &blk);

	orb_copy(ORB_ID(sensor_accel), sensor_accel_sub_fd, &sc);

	//printf("\t%8.4f\t%8.4f\t%8.4f\n",(double)sc.x,(double)sc.y,(double)sc.z);
	brightness=blk.control;
	//if (blk.control>15){led_blink = LED_BLINK;}
	//else {}
	led_r=abs((int)(sc.x*20));
	led_g=abs((int)(sc.y*20));
	led_b=abs((int)(sc.z*20));

	if (this->get_address()==9) {
		set_rgb(led_r,led_g,led_b);
	}
	else {
		if (brightness) {
			if (led_thread_runcount%10==0) {
				setLEDColor(LED_ORANGE);
			}
			else {
				setLEDColor(LED_OFF);
			}
			led_thread_runcount++;
		}
		else {
			setLEDColor(LED_BLUE);
		}
	}
	led_interval=50;


	if (systemstate_run == true) {
		/* re-queue ourselves to run again later */
		work_queue(LPWORK, &_work, (worker_t)&BlinkM::led_trampoline, this, led_interval);

	} else {
		stop_script();
		set_rgb(0, 0, 0);
	}
}

void BlinkM::setLEDColor(int ledcolor)
{
	switch (ledcolor) {
	case LED_OFF:	// off
		set_rgb(0, 0, 0);
		break;

	case LED_RED:	// red
		set_rgb(255, 0, 0);
		break;

	case LED_ORANGE:	// orange
		set_rgb(255, 150, 0);
		break;

	case LED_YELLOW:	// yellow
		set_rgb(200, 200, 0);
		break;

	case LED_PURPLE:	// purple
		set_rgb(255, 0, 255);
		break;

	case LED_GREEN:	// green
		set_rgb(0, 255, 0);
		break;

	case LED_BLUE:	// blue
		set_rgb(0, 0, 255);
		break;

	case LED_CYAN:	// cyan
		set_rgb(0, 128, 128);
		break;

	case LED_WHITE:	// white
		set_rgb(255, 255, 255);
		break;

	case LED_AMBER:	// amber
		set_rgb(255, 65, 0);
		break;
	}
}

int
BlinkM::set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
	const uint8_t msg[4] = { 'n', r, g, b };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::fade_rgb(uint8_t r, uint8_t g, uint8_t b)
{
	const uint8_t msg[4] = { 'c', r, g, b };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::fade_hsb(uint8_t h, uint8_t s, uint8_t b)
{
	const uint8_t msg[4] = { 'h', h, s, b };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::fade_rgb_random(uint8_t r, uint8_t g, uint8_t b)
{
	const uint8_t msg[4] = { 'C', r, g, b };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::fade_hsb_random(uint8_t h, uint8_t s, uint8_t b)
{
	const uint8_t msg[4] = { 'H', h, s, b };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::set_fade_speed(uint8_t s)
{
	const uint8_t msg[2] = { 'f', s };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::play_script(uint8_t script_id)
{
	const uint8_t msg[4] = { 'p', script_id, 0, 0 };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::play_script(const char *script_name)
{
	/* handle HTML colour encoding */
	if (isxdigit(script_name[0]) && (strlen(script_name) == 6)) {
		char code[3];
		uint8_t r, g, b;

		code[2] = '\0';

		code[0] = script_name[1];
		code[1] = script_name[2];
		r = strtol(code, 0, 16);
		code[0] = script_name[3];
		code[1] = script_name[4];
		g = strtol(code, 0, 16);
		code[0] = script_name[5];
		code[1] = script_name[6];
		b = strtol(code, 0, 16);

		stop_script();
		return set_rgb(r, g, b);
	}

	for (unsigned i = 0; script_names[i] != nullptr; i++)
		if (!strcasecmp(script_name, script_names[i])) {
			return play_script(i);
		}

	return -1;
}

int
BlinkM::stop_script()
{
	const uint8_t msg[1] = { 'o' };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::write_script_line(uint8_t line, uint8_t ticks, uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3)
{
	const uint8_t msg[8] = { 'W', 0, line, ticks, cmd, arg1, arg2, arg3 };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::read_script_line(uint8_t line, uint8_t &ticks, uint8_t cmd[4])
{
	const uint8_t msg[3] = { 'R', 0, line };
	uint8_t result[5];

	int ret = transfer(msg, sizeof(msg), result, sizeof(result));

	if (ret == OK) {
		ticks = result[0];
		cmd[0] = result[1];
		cmd[1] = result[2];
		cmd[2] = result[3];
		cmd[3] = result[4];
	}

	return ret;
}

int
BlinkM::set_script(uint8_t len, uint8_t repeats)
{
	const uint8_t msg[4] = { 'L', 0, len, repeats };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkM::get_rgb(uint8_t &r, uint8_t &g, uint8_t &b)
{
	const uint8_t msg = 'g';
	uint8_t result[3];

	int ret = transfer(&msg, sizeof(msg), result, sizeof(result));

	if (ret == OK) {
		r = result[0];
		g = result[1];
		b = result[2];
	}

	return ret;
}

int
BlinkM::get_firmware_version(uint8_t version[2])
{
	const uint8_t msg = 'Z';

	return transfer(&msg, sizeof(msg), version, 2);
}

int
BlinkM::get_blinkm_address(uint8_t *addr)
{
	const uint8_t msg = 'a';
	int ret = transfer(&msg, sizeof(msg), addr, 1);

	return ret;
}

void blinkm_usage();

void blinkm_usage()
{
	warnx("missing command: try 'start', 'systemstate', 'ledoff', 'list' or a script name {options}");
	warnx("options:");
	warnx("\t-b --bus i2cbus (3)");
	warnx("\t-a --blinkmaddr blinkmaddr (9)");
}

int
blinkm_main(int argc, char *argv[])
{

	int i2cdevice = PX4_I2C_BUS_EXPANSION;
	int blinkmadr = 9;
	const char *path="/dev/blinkm0";

	int x;

	if (argc < 2) {
		blinkm_usage();
		return 1;
	}

	for (x = 1; x < argc; x++) {
		if (strcmp(argv[x], "-b") == 0 || strcmp(argv[x], "--bus") == 0) {
			if (argc > x + 1) {
				i2cdevice = atoi(argv[x + 1]);
			}
		}

		if (strcmp(argv[x], "-a") == 0 || strcmp(argv[x], "--blinkmaddr") == 0) {
			if (argc > x + 1) {
				blinkmadr = atoi(argv[x + 1]);
			}
		}

		if (strcmp(argv[x], "-p") == 0 || strcmp(argv[x], "--path") == 0) {
			if (argc > x + 1) {
				path = argv[x + 1];
			}
		}

	}

	if (!strcmp(argv[1], "start")) {

		if (g_blinkm != nullptr) {
			if (blinkmadr==g_blinkm->get_address()) {
				printf("already started, current address: %d, new address: %d", g_blinkm->get_address(), blinkmadr);
				return 1;
			}
		}

		g_blinkm = new BlinkM(i2cdevice, blinkmadr, path);
		blinkm_multi[blinkmadr-9]=g_blinkm;

		if (g_blinkm == nullptr) {
			warnx("new failed");
			return 1;
		}

		if (OK != g_blinkm->init()) {
			delete g_blinkm;
			g_blinkm = nullptr;
			warnx("init failed");
			return 1;
		}

		return 0;
	}


	if (g_blinkm == nullptr) {
		fprintf(stderr, "not started\n");
		blinkm_usage();
		return 0;
	}

	if (!strcmp(argv[1], "systemstate")) {
		g_blinkm->setMode(1);
		return 0;
	}

	if (!strcmp(argv[1], "ledoff")) {
		return g_blinkm->setMode(0);
		return 0;
	}

	if (!strcmp(argv[1], "switch")) {
		if (g_blinkm->get_address()==blinkmadr) {
			warn("already on BlinkM device");
			return 1;
		}
		if (blinkm_multi[blinkmadr-9]==nullptr) {
			warn("BlinkM device not started");
			return 1;
		}
		g_blinkm=blinkm_multi[blinkmadr-9];
		return 0;
	}

	if (!strcmp(argv[1], "list")) {
		for (unsigned i = 0; BlinkM::script_names[i] != nullptr; i++) {
			fprintf(stderr, "    %s\n", BlinkM::script_names[i]);
		}

		fprintf(stderr, "    <html color number>\n");
		return 0;
	}

	/* things that require access to the device */
	int fd = px4_open(path, 0);

	if (fd < 0) {
		warn("can't open BlinkM device");
		return 1;
	}

	g_blinkm->setMode(0);

	if (px4_ioctl(fd, BLINKM_PLAY_SCRIPT_NAMED, (unsigned long)argv[1]) == OK) {
		return 0;
	}

	px4_close(fd);

	blinkm_usage();
	return 0;
}
