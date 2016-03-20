/****************************************************************************
 *
 * Copyright (C) 2016 Nathan Michael
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above
 * copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <cmath>
#include <cstring>
#include <unistd.h>
#include <termios.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "MocapAttitudeController.h"

namespace pu = parameter_utils;

MocapAttitudeController::MocapAttitudeController() :
  ctrl_state_set(false),
  motor_mode(OFF),
  motor_start_set(false),
  in_nominal_flight(false),
  rpm_set(false),
  avl_set(false),
  time_set(false),
  l1_initialized(false),
  l1_engage_level(0.0f)
{
  memset(avlhat, 0.0f, sizeof(avlhat));
  memset(dsthat, 0.0f, sizeof(dsthat));
  memset(lpd, 0.0f, sizeof(lpd));
  memset(rpmhat, 0.0f, sizeof(rpmhat));
}

MocapAttitudeController::~MocapAttitudeController() { }

bool MocapAttitudeController::initialize()
{
  if (!loadParameters())
  {
    puts("[MAC] failed to load parameters");
    return false;
  }

  if (!registerCallbacks())
  {
    puts("[MAC] failed to register callbacks");
    return false;
  }

  if (!mm.initialize())
  {
    puts("[MAC] failed to initialize motor manager");
    return false;
  }

  return true;
}

void MocapAttitudeController::finalize()
{
  mm.finalize();

  closeSubscriptions();
}

void MocapAttitudeController::update()
{
  // wait for updates from file descriptor for 1000 ms
  // update loop is throttled based on sensor input
  int poll_ret = poll(fds, 2, 1000);

  static unsigned int error_counter = 0;

  bool hl_cmd_updated = false;
  bool enable_motors = false;

  /* handle the poll result */
  if (poll_ret < 0)
  {
    if (error_counter++ > 50)
    {
      printf("[MAC] ERROR return value from poll(): %d", poll_ret);
      error_counter = 0;
    }
  }
  else if (poll_ret > 0)
  {
    if (fds[0].revents & POLLIN)
    {
      struct vehicle_command_s cmd_in;
      orb_copy(ORB_ID(vehicle_command), hl_sub, &cmd_in);
      enable_motors = commandMessageCallback(cmd_in);
      hl_cmd_updated = true;
    }
    else if (fds[1].revents & POLLIN)
    {// Attitude input received
      struct control_state_s ctrl_state;
      orb_copy(ORB_ID(control_state), ctrl_state_sub, &ctrl_state);
      controlStateMessageCallback(ctrl_state);
    }
  }

  bool cmd_received = false;
  uint64_t tnow = hrt_absolute_time();

  if (hl_cmd_updated)
  {
    cmd_time = tnow;
    cmd_received = true;

    if (enable_motors && (motor_mode == OFF))
      setMotorMode(START);

    if (!enable_motors)
      setMotorMode(OFF);
  }

  switch (motor_mode)
  {
    case START:
    {
      if (!motor_start_set)
      {
        motor_start_set = true;
        tmotor_start = tnow;
      }

      mm.setRPMCommand(MotorManager::START);
      if (tnow - tmotor_start > motor_start_dt_us)
      {
        motor_start_set = false;
        setMotorMode(IDLE);
      }
      break;
    }
    case OFF:
    {
      mm.setRPMCommand(MotorManager::OFF);
      att_cmd_casc = att_cmd_cpu;
      break;
    }
    case IDLE:
    {
      if (cmd_received)
        setMotorMode(RUNNING);
      else
        mm.setRPMCommand(MotorManager::MIN);
      break;
    }
    case RUNNING:
    {
      if (tnow - cmd_time > cmd_ttl_us)
      {
        setMotorMode(IDLE);
        break;
      }

      switch (input_mode)
      {
        case RPM:
        {
          // RPM set in message callback
          break;
        }
        case CASCADED:
        {
          MotorManager::rpm_cmd_t cmd;
          if (updateCascadedAttitudeController(cmd))
            mm.setRPMCommand(cmd);
          else
            mm.setRPMCommand(MotorManager::MIN);
          break;
        }
      }
      break;
    }
  }

  mm.sendCommand();

#if 0
  static unsigned int debug_counter = 0;
  if (debug_counter++ > 100)
  {
    if (motor_mode != OFF)
      mm.printCurrentRPMCommand();
    switch (motor_mode)
    {
      case START:
        puts("[MAC] motor_mode = START");
        break;
      case OFF:
        puts("[MAC] motor_mode = OFF");
        break;
      case IDLE:
        puts("[MAC] motor_mode = IDLE");
        break;
      case RUNNING:
        puts("[MAC] motor_mode = RUNNING");
        break;
    }

    debug_counter = 0;
  }
#endif
}

float MocapAttitudeController::convertRPMToForce(float rpm)
{
  // Assumes quadratic force model
  // f_i = cT*w_i^2
  return rpm > 0.0f ? cT*rpm*rpm : 0.0f;
}

float MocapAttitudeController::convertForceToRPM(float force)
{
  // Assumes quadratic force model
  // f_i = cT*w_i^2
  float ws = force*cT_inv;
  return ws > 0.0f ? sqrtf(ws) : 0.0f;
}

void MocapAttitudeController::convertBodyForcesToRPM(const body_forces_t& b,
                                                     MotorManager::rpm_cmd_t& out)
{
  // Ensure that forces are at least at the level of the min command
  float fm_base = convertRPMToForce(rpm_min);

  // Lower bound thrust (NED frame!)
  float fbz = fminf(-4.0f*fm_base, b.fbz);

  // Force at each of the motors (NED frame!)
  float fm[4];
  fm[0] = m11*fbz + m12*b.Mb[0] + m13*b.Mb[1] + m14*b.Mb[2];
  fm[1] = m21*fbz + m22*b.Mb[0] + m23*b.Mb[1] + m24*b.Mb[2];
  fm[2] = m31*fbz + m32*b.Mb[0] + m33*b.Mb[1] + m34*b.Mb[2];
  fm[3] = m41*fbz + m42*b.Mb[0] + m43*b.Mb[1] + m44*b.Mb[2];

  // Convert the forces to RPM commands
  for (unsigned int i = 0; i < 4; i++)
  {
    // Ensure a positive force
    fm[i] = fmaxf(fm[i], 0.0f);
    // Convert the force to RPM and ensure it is in bounds
    out.motor[i] = fminf(fmaxf(convertForceToRPM(fm[i]), rpm_min), rpm_max);
  }

#if 0
  static unsigned int counter = 0;
  if (counter++ > 100)
  {
    printf("fm = [%0.2f, %0.2f, %0.2f, %0.2f]\n",
           (double)fm[0], (double)fm[1], (double)fm[2], (double)fm[3]);
    out.print();
    counter = 0;
  }
#endif
}

bool MocapAttitudeController::updateCascadedAttitudeController(MotorManager::rpm_cmd_t& out)
{
  if (!ctrl_state_set)
    return false;

  // Ensure that the full multi-part message is processed before proceeding
  // the first time or on a ui_mode reset
  if (!att_cmd_casc.thrust_setpoint_set ||
      !att_cmd_casc.attitude_setpoint_set ||
      !att_cmd_casc.velocities_set ||
      !att_cmd_casc.accelerations_set ||
      !att_cmd_casc.proportional_gains_set ||
      !att_cmd_casc.derivative_gains_set)
  {
#if 0
    static unsigned int counter1 = 0;
    if (counter1++ > 100)
    {
      att_cmd_casc.print();
      counter1 = 0;
    }
#endif
    return false;
  }

  static uint64_t tcmd_last = 0;
  uint64_t tnow = hrt_absolute_time();
  uint64_t dt_cmd = tnow - tcmd_last;

  // HACK: Throttle the cmd update rate to ensure that it is always
  // less than 500 Hz. If this is removed, function dt
  // can get very small (e.g., < 1e-5).
  if (dt_cmd < 2000)
  {
    out = mm.getRPMCommand();
    return true;
  }

  tcmd_last = tnow;

  // Name is misleading - actually body frame angular velocity (w_x, w_y, w_z)
  float wb[3];
  wb[0] = ctrl_state.roll_rate;
  wb[1] = ctrl_state.pitch_rate;
  wb[2] = ctrl_state.yaw_rate;

  float roll_des = att_cmd_casc.q.roll();
  float pitch_des = att_cmd_casc.q.pitch();
  float yaw_des = att_cmd_casc.q.yaw();

  math::Vector<3> euler_zyx = math::Quaternion(ctrl_state.q).to_euler();

  // orientation error
  float eR[3];
  eR[0] = euler_zyx(0) - roll_des;
  eR[1] = euler_zyx(1) - pitch_des;
  eR[2] = shortest_angular_distance(yaw_des, euler_zyx(2));

  // angular velocity error
  float eOm[3];
  eOm[0] = wb[0] - att_cmd_casc.ang_vel_x;
  eOm[1] = wb[1] - att_cmd_casc.ang_vel_y;
  eOm[2] = wb[2] - att_cmd_casc.ang_vel_z;

#if 0
  static unsigned int debug_counter = 0;
  if (debug_counter++ > 100)
  {
    att_cmd_casc.print();

    printf("wb[0] = %0.4f, wb[1] = %0.4f, wb[2] = %0.4f\n",
           double(wb[0]), double(wb[1]), double(wb[2]));

    printf("roll_des = %0.4f, pitch_des = %0.4f, yaw_des = %0.4f\n",
           double(roll_des), double(pitch_des), double(yaw_des));

    printf("eR[0] = %0.4f, eR[1] = %0.4f, eR[2] = %0.4f\n",
           double(eR[0]), double(eR[1]), double(eR[2]));

    printf("eOm[0] = %0.4f, eOm[1] = %0.4f, eOm[2] = %0.4f\n",
           double(eOm[0]), double(eOm[1]), double(eOm[2]));
    debug_counter = 0;
  }
#endif

  // angular acceleration command
  float angacc_cmd[3];
  angacc_cmd[0] =
    -att_cmd_casc.kR1*eR[0] - att_cmd_casc.kOm1*eOm[0] + att_cmd_casc.ang_acc_x;
  angacc_cmd[1] =
    -att_cmd_casc.kR2*eR[1] - att_cmd_casc.kOm2*eOm[1] + att_cmd_casc.ang_acc_y;
  angacc_cmd[2] =
    -att_cmd_casc.kR3*eR[2] - att_cmd_casc.kOm3*eOm[2] + att_cmd_casc.ang_acc_z;

  body_forces_t body_cmd;

  body_cmd.fbz = att_cmd_casc.thrust;

  // torque command
  body_cmd.Mb[0] = Ixx*angacc_cmd[0] + Ixy*angacc_cmd[1] + Ixz*angacc_cmd[2];
  body_cmd.Mb[1] = Ixy*angacc_cmd[0] + Iyy*angacc_cmd[1] + Iyz*angacc_cmd[2];
  body_cmd.Mb[2] = Ixz*angacc_cmd[0] + Iyz*angacc_cmd[1] + Izz*angacc_cmd[2];

  if(enable_l1)
  {
    // If in CPU only mode, this function is only called when commands come in
    // Prevent the possibility by ensuring that any intervals larger than TTL force
    // an L1 controller reset
    if (!l1_initialized ||
        (time_set && ((tnow - tprev) > cmd_ttl_us)))
      initializeL1Controller();

    MotorManager::rpm_cmd_t current_rpm = mm.getRPMCommand();

    float total_force = 0.0f;
    for (unsigned int i = 0; i < 4; i++)
      total_force += convertRPMToForce(current_rpm.motor[i]);

    if(total_force < l1_engage_level*mass*gravity_magnitude)
      initializeL1Controller();
    else
      in_nominal_flight = true;

    if(in_nominal_flight)
    {
      for(unsigned int i = 0; i < 3; i++)
        body_cmd.Mb[i] -= lpd[i];
    }
  }

  convertBodyForcesToRPM(body_cmd, out);

  float dt = 0.0f;

  if (enable_l1 && in_nominal_flight)
  {
    float sat_rpm[4];

    for(unsigned int i = 0; i < 4; i++)
      sat_rpm[i] = fminf(out.motor[i], rpm_max);

    if(!rpm_set)
    {
      for(unsigned int i = 0; i < 4; i++)
        rpmhat[i] = sat_rpm[i];
      rpm_set = true;
    }

    if(!avl_set)
    {
      for(unsigned int i = 0; i < 3; i++)
        avlhat[i] = wb[i];
      avl_set = true;
    }

    if(!time_set)
    {
      tprev = tnow;
      time_set = true;
    }

    dt = static_cast<float>(1e-6*double(tnow - tprev));
    tprev = tnow;

    updateStatePredictor(dt, wb[0], wb[1], wb[2],
                         sat_rpm[0], sat_rpm[1], sat_rpm[2], sat_rpm[3]);
  }

#if 0
  static unsigned int counter = 0;
  if (counter++ > 100)
  {
    printf("dt = %0.7f\n", double(dt));
    printf("angacc_cmd = [%0.2f, %0.2f, %0.2f]\n",
           (double)angacc_cmd[0],
           (double)angacc_cmd[1],
           (double)angacc_cmd[2]);
    body_cmd.print();
    out.print();
    counter = 0;
  }
#endif

  return true;
}

void MocapAttitudeController::controlStateMessageCallback(const control_state_s& ctrl_state_)
{
  memcpy(&ctrl_state, &ctrl_state_, sizeof(ctrl_state_));
  ctrl_state_set = true;
}

bool MocapAttitudeController::commandMessageCallback(const vehicle_command_s& in)
{
  unsigned int cmd_type = static_cast<unsigned int>(in.param5);
  unsigned int input_type = static_cast<unsigned int>(in.param6);

  switch (cmd_type)
  {
    case 1: //RPM
    {
      input_mode = RPM;
      mm.setRPMCommand(MotorManager::rpm_cmd_t(in.param1, in.param2,
                                               in.param3, in.param4));
      break;
    }
    case 2: //Cascaded command feedback control
    {
      input_mode = CASCADED;
      switch (input_type)
      {
        case 0: //THRUST_SETPOINT
        {
          att_cmd_casc.thrust = in.param1;
          att_cmd_casc.thrust_setpoint_set = true;
          break;
        }
        case 1: //ATTITUDE_SETPOINT
        {
          att_cmd_casc.q.set(in.param1, in.param2, in.param3, in.param4);
          att_cmd_casc.attitude_setpoint_set = true;
          break;
        }
        case 2://VELOCITIES:
        {
          att_cmd_casc.ang_vel_x = in.param1;
          att_cmd_casc.ang_vel_y = in.param2;
          att_cmd_casc.ang_vel_z = in.param3;
          att_cmd_casc.velocities_set = true;
          break;
        }
        case 3://VELOCITIES:
        {
          att_cmd_casc.ang_acc_x = in.param1;
          att_cmd_casc.ang_acc_y = in.param2;
          att_cmd_casc.ang_acc_z = in.param3;
          att_cmd_casc.accelerations_set = true;
          break;
        }
        case 4://PGAINS
        {
          att_cmd_casc.kR1 = in.param1;
          att_cmd_casc.kR2 = in.param2;
          att_cmd_casc.kR3 = in.param3;
          att_cmd_casc.proportional_gains_set = true;
          break;
        }
        case 5://DGAINS
        {
          att_cmd_casc.kOm1 = in.param1;
          att_cmd_casc.kOm2 = in.param2;
          att_cmd_casc.kOm3 = in.param3;
          att_cmd_casc.derivative_gains_set = true;
          break;
        }
      }
      break;
    }
  }

  return in.param7 > 0.5f ? true : false;
}

void MocapAttitudeController::closeSubscriptions()
{
  close(ctrl_state_sub);
  close(hl_sub);
}

void MocapAttitudeController::setMotorMode(const motor_modes_t& m)
{
  motor_mode = m;
}

bool MocapAttitudeController::loadParameters()
{
  cT = pu::getFloatParam("MCC_CT");
  m_scale = pu::getFloatParam("MCC_MOMENT_SCALE");
  length = pu::getFloatParam("MCC_ARM_LENGTH");
  mass = pu::getFloatParam("MCC_TOTAL_MASS");
  motor_spread_angle = pu::getFloatParam("MCC_MOTOR_SPREAD");
  gravity_magnitude = pu::getFloatParam("MCC_GRAVITY");

  cmd_ttl_us =
    static_cast<uint64_t>(pu::getFloatParam("MMC_CMD_TTL")*1.0e6f);

  motor_start_dt_us =
    static_cast<uint64_t>(pu::getFloatParam("MCC_MOT_START_DT")*1.0e6f);

  rpm_min = pu::getFloatParam("MCC_RPM_MIN");
  rpm_max = pu::getFloatParam("MCC_RPM_MAX");

  cT_inv = 1.0f/cT;

  m11 = m21 = m31 = m41 = -0.25f;

  float dsm = length*sinf(motor_spread_angle);
  float dcm = length*cosf(motor_spread_angle);

  m12 = m42 = -0.25f/dsm;
  m22 = m32 = 0.25f/dsm;

  m13 = m33 = 0.25f/dcm;
  m23 = m43 = -0.25f/dcm;

  m14 = m24 = 0.25f/m_scale;
  m34 = m44 = -0.25f/m_scale;

  Ixx = pu::getFloatParam("MCC_INERTIA_XX");
  Ixy = pu::getFloatParam("MCC_INERTIA_XY");
  Ixz = pu::getFloatParam("MCC_INERTIA_XZ");
  Iyy = pu::getFloatParam("MCC_INERTIA_YY");
  Iyz = pu::getFloatParam("MCC_INERTIA_YZ");
  Izz = pu::getFloatParam("MCC_INERTIA_ZZ");

  float t2 = Iyz*Iyz;
  float t3 = Ixx*t2;
  float t4 = Ixz*Ixz;
  float t5 = Iyy*t4;
  float t6 = Ixy*Ixy;
  float t7 = Izz*t6;
  float t10 = Ixy*Ixz*Iyz*2.0f;
  float t11 = Ixx*Iyy*Izz;
  float t8 = t3+t5+t7-t10-t11;
  float t9 = 1.0f/t8;
  float t12 = Ixz*Iyz;
  float t13 = t12-Ixy*Izz;
  float t14 = Ixy*Iyz;
  float t15 = t14-Ixz*Iyy;
  float t16 = Ixy*Ixz;
  float t17 = t16-Ixx*Iyz;

  // inertia inverse
  invIxx = t9*(t2-Iyy*Izz);
  invIxy = -t9*t13;
  invIxz = -t9*t15;
  invIyy = t9*(t4-Ixx*Izz);
  invIyz = -t9*t17;
  invIzz = t9*(t6-Ixx*Iyy);

  enable_l1 = static_cast<bool>(pu::getIntParam("MCC_ENABLE_L1"));

  if (enable_l1)
  {
    puts("[MAC] L1 is enabled!");
    motor_constant = pu::getFloatParam("MCC_MOTOR_CONST");
    bandwidth[0] = pu::getFloatParam("L1C_BW_X");
    bandwidth[1] = pu::getFloatParam("L1C_BW_Y");
    bandwidth[2] = pu::getFloatParam("L1C_BW_Z");

    Kspx = pu::getFloatParam("L1C_KSP_X");
    Kspy = pu::getFloatParam("L1C_KSP_Y");
    Kspz = pu::getFloatParam("L1C_KSP_Z");

    adaptation_gain = pu::getFloatParam("L1C_ADAPT_GAIN");

    init_disturbance[0] = pu::getFloatParam("L1C_INIT_DIST_X");
    init_disturbance[1] = pu::getFloatParam("L1C_INIT_DIST_Y");
    init_disturbance[2] = pu::getFloatParam("L1C_INIT_DIST_Z");

    l1_engage_level = pu::getFloatParam("L1C_ENGAGE_LEVEL");
  }

  return true;
}

bool MocapAttitudeController::registerCallbacks()
{
  // High-level commands (e.g., from mavlink)
  hl_sub = orb_subscribe(ORB_ID(vehicle_command));

  // Control state input from estimator
  ctrl_state_sub = orb_subscribe(ORB_ID(control_state));

  // Define the callback file devices
  fds[0].fd = hl_sub;
  fds[0].events = POLLIN;
  fds[1].fd = ctrl_state_sub;
  fds[1].events = POLLIN;

  return true;
}

float MocapAttitudeController::unroll(float x)
{
  x = fmodf(x, 2.0f*M_PI_F);
  if (x < 0.0f)
    x += 2.0f*M_PI_F;
  return x;
}

float MocapAttitudeController::normalize(float x)
{
  x = fmodf(x + M_PI_F, 2.0f*M_PI_F);
  if (x < 0.0f)
    x += 2.0f*M_PI_F;
  return x - M_PI_F;
}

float MocapAttitudeController::shortest_angular_distance(float from, float to)
{
  float result = unroll(unroll(to) - unroll(from));
  if (result > M_PI_F)
    result = -(2.0f*M_PI_F - result);
  return normalize(result);
}

void MocapAttitudeController::initializeL1Controller()
{
  in_nominal_flight = false;
  rpm_set = false;
  avl_set = false;
  time_set = false;

  initializeStatePredictor();

  l1_initialized = true;
}

void MocapAttitudeController::initializeStatePredictor()
{
  for(unsigned int i = 0; i < 3; i++)
  {
    dsthat[i] = init_disturbance[i];
    lpd[i] = init_disturbance[i];
  }
}

void MocapAttitudeController::updateStatePredictor(float dt, float wbx, float wby, float wbz, float rpm1_des, float rpm2_des, float rpm3_des, float rpm4_des)
{
  float avlerr_x = avlhat[0] - wbx;
  float avlerr_y = avlhat[1] - wby;
  float avlerr_z = avlhat[2] - wbz;

  float f1 = convertRPMToForce(rpmhat[0]);
  float f2 = convertRPMToForce(rpmhat[1]);
  float f3 = convertRPMToForce(rpmhat[2]);
  float f4 = convertRPMToForce(rpmhat[3]);
  float tau_x = length*(f2 - f4);
  float tau_y = -length*(f1 - f3);
  float tau_z = -m_scale*(f1 - f2 + f3 - f4);

  float avlrate[3];

  float t2 = Ixx*wbx;
  float t3 = Ixy*wby;
  float t4 = Ixz*wbz;
  float t5 = t2+t3+t4;
  float t6 = Ixy*wbx;
  float t7 = Iyy*wby;
  float t8 = Iyz*wbz;
  float t9 = t6+t7+t8;
  float t10 = Ixz*wbx;
  float t11 = Iyz*wby;
  float t12 = Izz*wbz;
  float t13 = t10+t11+t12;
  float t14 = t5*wby;
  float t20 = t9*wbx;
  float t15 = dsthat[2]+t14-t20+tau_z;
  float t16 = t13*wbx;
  float t21 = t5*wbz;
  float t17 = dsthat[1]+t16-t21+tau_y;
  float t18 = t9*wbz;
  float t22 = t13*wby;
  float t19 = dsthat[0]+t18-t22+tau_x;

  avlrate[0] = -Kspx*avlerr_x + invIxz*t15+invIxy*t17+invIxx*t19;
  avlrate[1] = -Kspy*avlerr_y + invIyz*t15+invIyy*t17+invIxy*t19;
  avlrate[2] = -Kspz*avlerr_z + invIzz*t15+invIyz*t17+invIxz*t19;

  float dstrate[3];
  dstrate[0] = Ixx*avlerr_x + Ixy*avlerr_y + Ixz*avlerr_z;
  dstrate[1] = Ixy*avlerr_x + Iyy*avlerr_y + Iyz*avlerr_z;
  dstrate[2] = Ixz*avlerr_x + Iyz*avlerr_y + Izz*avlerr_z;

  for(unsigned int i = 0; i < 3; i++)
  {
    dstrate[i] *= -adaptation_gain;
    avlhat[i] += dt*avlrate[i];
    dsthat[i] += dt*dstrate[i];
    lpd[i] += dt*bandwidth[i]*(dsthat[i] - lpd[i]);
  }

  rpmhat[0] += dt*(motor_constant*(rpm1_des - rpmhat[0]));
  rpmhat[1] += dt*(motor_constant*(rpm2_des - rpmhat[1]));
  rpmhat[2] += dt*(motor_constant*(rpm3_des - rpmhat[2]));
  rpmhat[3] += dt*(motor_constant*(rpm4_des - rpmhat[3]));

#if 0
  static unsigned int counter = 0;
  if (counter++ > 100)
  {
    puts("[MAC] L1 Iteration:");
    printf("rpm_des = [%0.2f, %0.2f, %0.2f, %0.2f]\n",
           (double)rpm1_des, (double)rpm2_des,
           (double)rpm3_des, (double)rpm4_des);

    printf("avlerr = [%0.4f, %0.4f, %0.4f]\n",
           double(avlerr_x), double(avlerr_y), double(avlerr_z));

    printf("tau = [%0.5f, %0.5f, %0.5f]\n",
           double(tau_x), double(tau_y), double(tau_z));

    printf("avlrate = [%0.4f, %0.4f, %0.4f]\n",
           double(avlrate[0]), double(avlrate[1]), double(avlrate[2]));

    printf("dstrate = [%0.4f, %0.4f, %0.4f]\n",
           double(dstrate[0]), double(dstrate[1]), double(dstrate[2]));

    printf("avlhat = [%0.4f, %0.4f, %0.4f]\n",
           double(avlhat[0]), double(avlhat[1]), double(avlhat[2]));

    printf("dsthat = [%0.4f, %0.4f, %0.4f]\n",
           double(dsthat[0]), double(dsthat[1]), double(dsthat[2]));

    printf("lpd = [%0.4f, %0.4f, %0.4f]\n",
           double(lpd[0]), double(lpd[1]), double(lpd[2]));

    printf("rpmhat = [%0.2f, %0.2f, %0.2f, %0.2f]\n",
           double(rpmhat[0]), double(rpmhat[1]),
           double(rpmhat[2]), double(rpmhat[3]));
    counter = 0;
  }
#endif
}
