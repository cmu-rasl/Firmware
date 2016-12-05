/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
 *   Author: Stefan Rado <px4@sradonia.net>
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
 * @file trtower.c
 * @author Stefan Rado <px4@sradonia.net>
 * @author Mark Whitehorn <kd0aij@github.com>
 *
 * FrSky D8 mode and SmartPort (D16 mode) telemetry implementation.
 *
 * This daemon emulates the FrSky Sensor Hub for D8 mode.
 * For X series receivers (D16 mode) it emulates SmartPort sensors by responding to polling
 * packets received from an attached FrSky X series receiver.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <termios.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/trtower.h>

/* thread state */
static volatile bool thread_should_exit = false;
static volatile bool thread_running = false;
static int trtower_task;

//teraranger cmds
static const char PRECISE_MODE[] = "PPP";
static const char FAST_MODE[] = "FFF";
static const char OUTDOOR_MODE[] = "OOO";

static const char BINARY_MODE[] = "BBB";
static const char TEXT_MODE[] = "TTT";

static const uint8_t crc_table[] = {0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23,
                                    0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41,
                                    0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf,
                                    0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd, 0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85,
                                    0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc,
                                    0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
                                    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a, 0x27, 0x20,
                                    0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
                                    0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74,
                                    0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8,
                                    0xad, 0xaa, 0xa3, 0xa4, 0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6,
                                    0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
                                    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10, 0x05, 0x02,
                                    0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34, 0x4e, 0x49, 0x40, 0x47,
                                    0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39,
                                    0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
                                    0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d,
                                    0x84, 0x83, 0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
                                    0xfa, 0xfd, 0xf4, 0xf3};

static orb_advert_t _trtower_topic = NULL;

/* functions */
static int sPort_open_uart(const char *uart_name, struct termios *uart_config, struct termios *uart_config_original);
static void usage(void);
static uint8_t crc8(uint8_t *p, uint8_t len);
static int trtower_thread_main(int argc, char *argv[]);
__EXPORT int trtower_main(int argc, char *argv[]);

#define DEBUG
#define BUFFER_SIZE 19

/**
 * Opens the UART device and sets all required serial parameters.
 */
static int sPort_open_uart(const char *uart_name, struct termios *uart_config, struct termios *uart_config_original)
{
    /* Open UART */
    const int uart = open(uart_name, O_RDWR | O_NOCTTY);

    if (uart < 0) {
        err(1, "Error opening port: %s", uart_name);
    }
    else {
        fcntl(uart, F_SETFL, 10);
    }

    /* Back up the original UART configuration to restore it after exit */
    int termios_state;

    if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
        warnx("ERR: tcgetattr%s: %d\n", uart_name, termios_state);
        close(uart);
        return -1;
    }

    /* Fill the struct for the new configuration */
    tcgetattr(uart, uart_config);

    uart_config->c_cflag &= ~PARENB;
    uart_config->c_cflag &= ~CSTOPB;
    uart_config->c_cflag &= ~CSIZE;
    uart_config->c_cflag |= CS8;

    uart_config->c_iflag = 0;
    uart_config->c_oflag = 0;
    uart_config->c_lflag = 0;

    /* Set baud rate */
    static const speed_t speed = B921600;

    if (cfsetispeed(uart_config, speed) < 0 || cfsetospeed(uart_config, speed) < 0) {
        warnx("ERR: %s: %d (cfsetispeed, cfsetospeed)\n", uart_name, termios_state);
        close(uart);
        return -1;
    }

    if ((termios_state = tcsetattr(uart, TCSANOW, uart_config)) < 0) {
        warnx("ERR: %s (tcsetattr)\n", uart_name);
        close(uart);
        return -1;
    }

    return uart;
}

/**
 * Print command usage information
 */
static void usage()
{
    fprintf(stderr,
        "usage: trtower start [-d <devicename>]\n"
        "       trtower stop\n"
        "       trtower status\n");
    exit(1);
}

/*checksum checker*/
static uint8_t crc8(uint8_t *p, uint8_t len)
{
  uint16_t i;
  uint16_t crc = 0x0;

  while (len--)
  {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
  }
  return crc & 0xFF;
}

/**
 * The daemon thread.
 */
static int trtower_thread_main(int argc, char *argv[])
{
    /* Default values for arguments */
    char *device_name = "/dev/ttyS1"; /* USART8 /dev/ttyS2 for v2*/

    /* Work around some stupidity in task_create's argv handling */
    argc -= 2;
    argv += 2;

    int ch;

    while ((ch = getopt(argc, argv, "d:")) != EOF) {
        switch (ch) {
        case 'd':
            device_name = optarg;
            break;

        default:
            usage();
            break;
        }
    }

    /* Open UART assuming D type telemetry */
    struct termios uart_config_original;
    struct termios uart_config;
    const int uart = sPort_open_uart(device_name, &uart_config, &uart_config_original);

    if (uart < 0) {
        warnx("could not open %s", device_name);
        err(1, "could not open %s", device_name);
    }

    thread_running = true;

    write(uart, (const void*)BINARY_MODE, 3);

    /* Main thread loop */
    uint8_t input_buffer[BUFFER_SIZE];
    int buffer_ctr = 0;
    char single_character;
    int16_t ranges[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    double inf = INFINITY;
    double DistanceToCenter = 0.06;

    struct trtower_s trt_report;
    trt_report.min_distance = 0.026;
    trt_report.max_distance = 14.0;

    _trtower_topic = orb_advertise(ORB_ID(trtower), &trt_report);

    while (!thread_should_exit) {
        read(uart, &single_character, 1);
        if (single_character != 'T' && buffer_ctr < 19) {
            // not begin of serial feed so add char to buffer
            input_buffer[buffer_ctr++] = single_character;
            continue;
        }
        else if (single_character == 'T') {
            if (buffer_ctr == BUFFER_SIZE) {
                // end of feed, calculate
                int16_t crc = crc8(input_buffer, 18);

                if (crc == input_buffer[18]) {
                    for (int i = 0; i < 8; i++)
                    {
                        ranges[i] = input_buffer[(i+1)*2] << 8;
                        ranges[i] |= input_buffer[(i+1)*2+1];
                    }   

                    for (int i = 0; i < 8; i++)
                    {
                        if (ranges[i] == 0)
                        {
                            trt_report.ranges[i] = inf;
                        }
                        else
                        {
                            trt_report.ranges[i] = ranges[i]*0.001 + DistanceToCenter;
                        }
                    }
		    trt_report.timestamp = hrt_absolute_time();
                    if (_trtower_topic != NULL) {
                        orb_publish(ORB_ID(trtower), _trtower_topic, &trt_report);
                    }
                }
                else {
                    warnx("crc mismatch");
                }                
            }
            else {
                warnx("reset buffer");
            }

        }
        else {
            warnx("buffer overflow, reset");
        }
          // reset
        buffer_ctr = 0;

        // clear struct
        bzero(&input_buffer, BUFFER_SIZE);

        // store T
        input_buffer[buffer_ctr++] = 'T';
    }

    /* Reset the UART flags to original state */
    tcsetattr(uart, TCSANOW, &uart_config_original);
    close(uart);

    thread_running = false;
    return 0;
}

/**
 * The main command function.
 * Processes command line arguments and starts the daemon.
 */
int trtower_main(int argc, char *argv[])
{
    if (argc < 2) {
        warnx("missing command");
        usage();
    }

    if (!strcmp(argv[1], "start")) {

        /* this is not an error */
        if (thread_running) {
            errx(0, "trtower already running");
        }

        thread_should_exit = false;
        trtower_task = px4_task_spawn_cmd("trtower",
                        SCHED_DEFAULT,
                        200,
                        1100,
                        trtower_thread_main,
                        (char *const *)argv);

        while (!thread_running) {
            usleep(200);
        }

        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {

        /* this is not an error */
        if (!thread_running) {
            errx(0, "trtower already stopped");
        }

        thread_should_exit = true;

        while (thread_running) {
            usleep(1000000);
            warnx(".");
        }

        warnx("terminated.");
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            errx(0, "running");
        } else {
            errx(1, "not running");
        }
    }

    warnx("unrecognized command");
    usage();
    /* not getting here */
    return 0;
}
