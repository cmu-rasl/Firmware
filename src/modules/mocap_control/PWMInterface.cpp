#include <cstdio>
#include <cmath>
#include <fcntl.h>
#include <systemlib/err.h>
#include <drivers/drv_pwm_output.h>

#include "PWMInterface.h"

namespace pu = parameter_utils;

PWMInterface::PWMInterface() { }
PWMInterface::~PWMInterface() { }

bool PWMInterface::initialize()
{
  if (!loadParameters())
  {
    puts("[PWMInterface] failed to load parameters");
    return false;
  }

  if (!openDevice())
  {
    puts("[PWMInterface] failed to open device");
    return false;
  }

#if 1
  printParameters();
#endif

  return true;
}

void PWMInterface::printCommand(const pwm_cmd_t& p)
{
  printf("[PWMInterface] pwm cmd = %hu, %hu, %hu, %hu",
         p.motor[0], p.motor[1],
         p.motor[2], p.motor[3]);
}

unsigned short PWMInterface::convertRPMToPWM(float rpm)
{
  float t = (rpm - (float)rpm_min)/((float)rpm_max - (float)rpm_min)*((float)pwm_max - (float)pwm_min) + (float)pwm_min;
  return saturateCommand((unsigned short)roundf(t));
}

float PWMInterface::convertPWMToRPM(unsigned short pwm)
{
  return ((float)pwm - (float)pwm_min)/((float)pwm_max - (float)pwm_min)*((float)rpm_max - (float)rpm_min) + rpm_min;
}

bool PWMInterface::sendIOCTLCommand(int cmd, unsigned long arg)
{
  int ret = ioctl(fd, cmd, arg);
  if (ret != OK)
  {
    puts("[PWMInterface] ioctl failed");
    return false;
  }

  return true;
}

void PWMInterface::getZeroCommand(pwm_cmd_t& cmd)
{
  toCommand(pwm_zero, cmd);
}

void PWMInterface::getMinCommand(pwm_cmd_t& cmd)
{
  toCommand(pwm_min, cmd);
}

void PWMInterface::getMaxCommand(pwm_cmd_t& cmd)
{
  toCommand(pwm_max, cmd);
}

unsigned short PWMInterface::getZeroCommand()
{
  return pwm_zero;
}

unsigned short PWMInterface::getMinCommand()
{
  return pwm_min;
}

unsigned short PWMInterface::getMaxCommand()
{
  return pwm_max;
}

unsigned short PWMInterface::saturateCommand(unsigned short v)
{
  // Naive thresholding - should use fmu io max/min values
  unsigned short t;
  if (v > pwm_max)
    t = pwm_max;
  else if (v < pwm_zero)
    t = pwm_zero;
  else
    t = v;

  return t;
}

void PWMInterface::toCommand(unsigned short m, pwm_cmd_t& p)
{
  for (unsigned int i = 0; i < 4; i++)
    p.motor[i] = saturateCommand(m);
}

bool PWMInterface::sendCommand(const pwm_cmd_t& cmd)
{
  for (unsigned int i = 0; i < 4; i++)
  {
    if (!sendIOCTLCommand(PWM_SERVO_SET(pwm_map[i]), cmd.motor[i]))
    {
      printf("[PWMInterface] failed to set PWM_SERVO_SET(%d)", i);
      return false;
    }
  }

  return true;
}

bool PWMInterface::loadParameters()
{
  // Use the values associated with main
  pwm_zero = pu::getUShortParam("PWM_AUX_DISARMED");
  pwm_min = pu::getUShortParam("PWM_AUX_MIN");
  pwm_max = pu::getUShortParam("PWM_AUX_MAX");

  rpm_min = pu::getFloatParam("MCC_RPM_MIN");
  rpm_max = pu::getFloatParam("MCC_RPM_MAX");

  // Shift to index value
  pwm_map[0] = pu::getIntParam("MCC_PWM_MAP1") - 1;
  pwm_map[1] = pu::getIntParam("MCC_PWM_MAP2") - 1;
  pwm_map[2] = pu::getIntParam("MCC_PWM_MAP3") - 1;
  pwm_map[3] = pu::getIntParam("MCC_PWM_MAP4") - 1;

  pwm_rate = pu::getIntParam("MCC_PWM_RATE");

  return true;
}

void PWMInterface::printParameters()
{
  printf("[PWMInterface] pwm_zero = %hu\n", pwm_zero);
  printf("[PWMInterface] pwm_min = %hu\n", pwm_min);
  printf("[PWMInterface] pwm_max = %hu\n", pwm_max);
  printf("[PWMInterface] rpm_min = %0.2f\n", (double)rpm_min);
  printf("[PWMInterface] rpm_max = %0.2f\n", (double)rpm_max);
}

bool PWMInterface::openDevice()
{
  const char *pwm_dev = PWM_OUTPUT0_DEVICE_PATH;
  fd = open(pwm_dev, 0);
  if (fd < 0)
  {
    printf("[PWMInterface] can't open %s", pwm_dev);
    return false;
  }

  int ret = ioctl(fd, PWM_SERVO_SET_UPDATE_RATE, pwm_rate);
  if (ret != OK)
  {
    puts("[PWMInterface] ioctl set pwm update rate failed");
    return false;
  }

  return true;
}

bool PWMInterface::armMotors()
{
  if (!sendIOCTLCommand(PWM_SERVO_SET_ARM_OK, 0))
  {
    puts("[PWMInterface] failed to set PWM_SERVO_SET_ARM_OK");
    return false;
  }

  if (!sendIOCTLCommand(PWM_SERVO_ARM, 0))
  {
    puts("[PWMInterface] failed to set PWM_SERVO_ARM");
    return false;
  }

  return true;
}

bool PWMInterface::disarmMotors()
{
  if (!sendIOCTLCommand(PWM_SERVO_DISARM, 0))
  {
    puts("[PWMInterface] failed to set PWM_SERVO_DISARM");
    return false;
  }

  return true;
}
