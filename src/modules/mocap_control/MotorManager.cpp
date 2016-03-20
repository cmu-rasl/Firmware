#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "ParameterUtils.h"
#include "MotorManager.h"

namespace pu = parameter_utils;

MotorManager::MotorManager() :
  rpm_min(0.0f), rpm_start(0.0f), rpm_max(0.0f) { }

MotorManager::~MotorManager() { }

bool MotorManager::initialize()
{
  if (!pwm.initialize())
  {
    puts("[MotorManager] failed to initialize PWM interface");
    return false;
  }

  if (!loadParameters())
  {
    puts("[MotorManager] failed to load parameters");
    return false;
  }

  if (!registerCallbacks())
  {
    puts("[MotorManager] failed to register callbacks");
    return false;
  }

  if (!armMotors())
  {
    puts("[MotorManager] failed to arm motors");
    return false;
  }

  return true;
}

void MotorManager::finalize()
{
  if (!disarmMotors())
    puts("[MotorManager] failed to disarm motors");
}

bool MotorManager::loadParameters()
{
  rpm_start = pu::getFloatParam("MCC_RPM_START");
  rpm_min = pu::getFloatParam("MCC_RPM_MIN");
  rpm_max = pu::getFloatParam("MCC_RPM_MAX");

  return true;
}

bool MotorManager::registerCallbacks()
{
  // Indicate to other apps arming state
  memset(&armed, 0, sizeof(armed));
  armed_pub = orb_advertise(ORB_ID(actuator_armed), &armed);

  return true;
}

float MotorManager::getRPMCommand(const MotorManager::motor_mode_t& m)
{
  switch (m)
  {
    case OFF:
      return 0.0f;
    case MIN:
      return rpm_min;
    case START:
      return rpm_start;
    case MAX:
      return rpm_max;
  }

  return 0.0f;
}

MotorManager::rpm_cmd_t MotorManager::getRPMCommand()
{
  return current_cmd;
}

void MotorManager::setRPMCommand(const MotorManager::motor_mode_t& m)
{
  switch (m)
  {
    case OFF:
      setRPMCommand(0.0f);
      break;
    case MIN:
      setRPMCommand(rpm_min);
      break;
    case START:
      setRPMCommand(rpm_start);
      break;
    case MAX:
      setRPMCommand(rpm_max);
      break;
  }
}

void MotorManager::setRPMCommand(float v)
{
  // Internal function, saturate to [0, rpm_max] to allow for motor turn off
  float vs = fmaxf(fminf(rpm_max, v), 0.0f);
  for (unsigned int i = 0; i < 4; i++)
    current_cmd.motor[i] = vs;
}

void MotorManager::setRPMCommand(const MotorManager::rpm_cmd_t& in)
{
  // External function, saturate to [rpm_min, rpm_max]
  current_cmd = saturateRPMCommand(in);
}

MotorManager::rpm_cmd_t MotorManager::saturateRPMCommand(const MotorManager::rpm_cmd_t& in)
{
  rpm_cmd_t out;
  for (unsigned int i = 0; i < 4; i++)
    out.motor[i] = fmaxf(fminf(rpm_max, in.motor[i]), rpm_min);
  return out;
}

void MotorManager::printCurrentRPMCommand()
{
  current_cmd.print();
}

void MotorManager::sendCommand()
{
  PWMInterface::pwm_cmd_t out;
  for (unsigned int i = 0; i < 4; i++)
  {
    if (current_cmd.motor[i] < 1.0e-6f)
      out.motor[i] = pwm.getZeroCommand();
    else
      out.motor[i] = pwm.convertRPMToPWM(current_cmd.motor[i]);
  }
  pwm.sendCommand(out);
}

void MotorManager::publishArmState(bool arm_state)
{
  armed.timestamp = hrt_absolute_time();
  armed.armed = arm_state;
  armed.ready_to_arm = !arm_state;
  armed.lockdown = false;
  orb_publish(ORB_ID(actuator_armed), armed_pub, &armed);
}

bool MotorManager::armMotors()
{
  if (!pwm.armMotors())
  {
    puts("[MotorManager] failed to arm motors");
    return false;
  }

  // Set the motor speed to zero
  PWMInterface::pwm_cmd_t p;
  pwm.getZeroCommand(p);
  if (!pwm.sendCommand(p))
  {
    puts("[MotorManager] failed to send pwm command");
    return false;
  }

  publishArmState(true);

  return true;
}

bool MotorManager::disarmMotors()
{
  // Set the motor speed to zero
  PWMInterface::pwm_cmd_t p;
  pwm.getZeroCommand(p);
  if (!pwm.sendCommand(p))
  {
    puts("[MotorManager] failed to send pwm command");
    return false;
  }

  if (!pwm.disarmMotors())
  {
    puts("[MotorManager] failed to disarm motors");
    return false;
  }

  publishArmState(false);

  return true;
}
