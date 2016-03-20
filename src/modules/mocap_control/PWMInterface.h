#ifndef PWM_INTERFACE
#define PWM_INTERFACE

#include <cstring>
#include "ParameterUtils.h"

class PWMInterface
{
public:
  PWMInterface();
  ~PWMInterface();

  typedef struct PWMCommand
  {
    unsigned short motor[4];
    PWMCommand()
    {
      memset(&motor, 0, sizeof(motor));
    }
  } pwm_cmd_t;

  bool initialize();

  bool armMotors();
  bool disarmMotors();

  void printCommand(const pwm_cmd_t& p);

  void getZeroCommand(pwm_cmd_t& cmd);
  void getMinCommand(pwm_cmd_t& cmd);
  void getMaxCommand(pwm_cmd_t& cmd);

  unsigned short getZeroCommand();
  unsigned short getMinCommand();
  unsigned short getMaxCommand();

  unsigned short convertRPMToPWM(float rpm);
  float convertPWMToRPM(unsigned short pwm);

  bool sendCommand(const pwm_cmd_t& cmd);

private:
  bool loadParameters();
  bool openDevice();
  bool sendIOCTLCommand(int cmd, unsigned long arg);

  unsigned short saturateCommand(unsigned short v);
  void toCommand(unsigned short m, pwm_cmd_t& p);

  void printParameters();

  int fd;
  int pwm_map[4];

  unsigned short pwm_zero;
  unsigned short pwm_min;
  unsigned short pwm_max;

  float rpm_min;
  float rpm_max;

  int pwm_rate;
};
#endif
