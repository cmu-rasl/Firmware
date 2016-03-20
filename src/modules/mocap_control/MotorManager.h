#ifndef MOTOR_MANAGER
#define MOTOR_MANAGER

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_armed.h>

#include "PWMInterface.h"

class MotorManager
{
public:
  MotorManager();
  ~MotorManager();

  typedef struct RPMCommand
  {
    float motor[4];
    RPMCommand()
    {
      memset(motor, 0, sizeof(motor));
    }
    RPMCommand(const RPMCommand& rhs)
    {
      memcpy(motor, rhs.motor, sizeof(motor));
    }
    RPMCommand(float m1, float m2, float m3, float m4)
    {
      motor[0] = m1;
      motor[1] = m2;
      motor[2] = m3;
      motor[3] = m4;
    }

    void print()
    {
      printf("rpm cmd = %0.2f, %0.2f, %0.2f, %0.2f\n",
             (double)motor[0], (double)motor[1],
             (double)motor[2], (double)motor[3]);
    }

    float max()
    {
      float m = -FLT_MAX;
      for (unsigned int i = 0; i < 4; i++)
        m = fmaxf(m, motor[i]);
      return m;
    }

    float min()
    {
      float m = FLT_MAX;
      for (unsigned int i = 0; i < 4; i++)
        m = fminf(m, motor[i]);
      return m;
    }
  } rpm_cmd_t;

  typedef enum MotorMode
  {
    OFF,
    MIN,
    START,
    MAX
  } motor_mode_t;

  float getRPMCommand(const motor_mode_t& m);
  rpm_cmd_t getRPMCommand();

  void setRPMCommand(const motor_mode_t& m);
  void setRPMCommand(const rpm_cmd_t& in);

  bool initialize();
  void finalize();

  void printCurrentRPMCommand();

  void sendCommand();

private:
  bool loadParameters();
  bool registerCallbacks();

  bool armMotors();
  bool disarmMotors();
  void publishArmState(bool arm_state);

  rpm_cmd_t saturateRPMCommand(const rpm_cmd_t& in);
  void setRPMCommand(float v);

  PWMInterface pwm;
  rpm_cmd_t current_cmd;

  orb_advert_t armed_pub;
  struct actuator_armed_s armed;

  float rpm_min, rpm_start, rpm_max;
};

#endif
