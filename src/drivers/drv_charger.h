/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file drv_charger.h
 *
 * charger driver API
 *
 * This could probably become a more generalised API for multi-colour LED
 * driver systems, or be merged with the generic LED driver.
 */

#pragma once

#include <px4_defines.h>
#include <stdint.h>
#include <sys/ioctl.h>

#define HSS0_DEVICE_PATH	"/dev/hss_adc"
#define HSS1_DEVICE_PATH	"/dev/hss_gpio"
#define UPS0_DEVICE_PATH	"/dev/ups0"

#define HSS_ADC_ADDRESS		0x68 >> 1
#define HSS_GPIO_ADDRESS	0x30 >> 1
#define UPS0_ADDRESS		0x70 >> 1

#define I2C_BALANCER_VOLTAGE 0x09
#define I2C_BALANCER_CURRENT 0x0A

#define HSS_GPIOCONF 0x03
#define HSS_GPIODAT 0x01
#define HSS_ADCCONF 0x60
#define HSS_ACTIVE 0x00
#define HSS_INACTIVE 0x01

#define VOLTAGE_ERR	50402
#define UPS_CURRENT_ERR	-15133

/*
 * ioctl() definitions
 */

#define _CHARGERIOCBASE			(0x1300)
#define _CHARGERIOC(_n)			(_PX4_IOC(_CHARGERIOCBASE, _n))

#define CHARGER_GET_VOLTAGE		_CHARGERIOC(1)
#define CHARGER_GET_HSS_CURRENT	_CHARGERIOC(2)
#define CHARGER_GET_UPS_CURRENT	_CHARGERIOC(3)

