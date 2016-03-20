#ifndef MOCAP_ATTITUDE_CONTROL
#define MOCAP_ATTITUDE_CONTROL
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
#include <cstdio>
#include <poll.h>

#include <mathlib/mathlib.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/control_state.h>

#include "ParameterUtils.h"
#include "MotorManager.h"

typedef struct Quaternion
{
  Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
  Quaternion(float w_, float x_, float y_, float z_)
  {
    set(w_, x_, y_, z_);
  }

  void print()
  {
    printf("quat = [%0.2f, %0.2f, %0.2f, %0.2f]\n",
          (double)w, (double)x, (double)y, (double)z);
  }

  float roll()
  {
    float R31 = 2.0f*(x*z - w*y);
    float theta = -asinf(R31);
    float psi = 0.0f;
    if(cosf(theta) > 1e-6f)
    {
      float R32 = 2.0f*(y*z + w*x);
      float R33 = w*w - x*x - y*y + z*z;
      psi = atan2f(R32, R33);
    }
    return psi;
  }

  float pitch()
  {
    float R31 = 2.0f*(x*z - w*y);
    return -asinf(R31);
  }

  float yaw()
  {
    float R31 = 2.0f*(x*z - w*y);
    float theta = -asinf(R31);
    float psi = 0.0f;
    if(cosf(theta) > 1e-6f)
    {
      float R11 = w*w + x*x - y*y - z*z;
      float R21 = 2.0f*(x*y + w*z);
      psi = atan2f(R21, R11);
    }
    return psi;
  }

  void set(float w_, float x_, float y_, float z_)
  {
    float mag = w_*w_ + x_*x_ + y_*y_ + z_*z_;

    if (mag > 1.0e-6f)
    {
      w = w_/mag;
      x = x_/mag;
      y = y_/mag;
      z = z_/mag;
    }
    else
    {
      w = 1.0f;
      x = y = z = 0.0f;
    }
  }

  void set(float roll_, float pitch_, float yaw_)
  {
    double cph_2 = cos(0.5*double(roll_));
    double sph_2 = sin(0.5*double(roll_));
    double cth_2 = cos(0.5*double(pitch_));
    double sth_2 = sin(0.5*double(pitch_));
    double cps_2 = cos(0.5*double(yaw_));
    double sps_2 = sin(0.5*double(yaw_));

    double w_ = cph_2*cth_2*cps_2 + sph_2*sth_2*sps_2;
    double x_ = sph_2*cth_2*cps_2 - cph_2*sth_2*sps_2;
    double y_ = cph_2*sth_2*cps_2 + sph_2*cth_2*sps_2;
    double z_ = cph_2*cth_2*sps_2 - sph_2*sth_2*cps_2;

    double mag = w_*w_ + x_*x_ + y_*y_ + z_*z_;
    w_ /= mag; x_ /= mag; y_ /= mag; z_ /= mag;

    w = static_cast<float>(w_);
    x = static_cast<float>(x_);
    y = static_cast<float>(y_);
    z = static_cast<float>(z_);
  }

  float w, x, y, z;
} quaternion_t;

class MocapAttitudeController
{
public:
  MocapAttitudeController();
  ~MocapAttitudeController();

  bool initialize();
  void finalize();
  void update();

private:
  typedef struct BodyForcesCommand
  {
    float fbz;
    float Mb[3];

    BodyForcesCommand() : fbz(0.0f)
    {
      memset(Mb, 0, sizeof(Mb));
    }

    void print()
    {
      printf("body cmd = fz=%0.5f, Mx=%0.5f, My=%0.5f, Mz=%0.5f\n",
            double(fbz), double(Mb[0]), double(Mb[1]), double(Mb[2]));
    }
  } body_forces_t;

  typedef enum
  {
    START = 0,
    OFF,
    IDLE,
    RUNNING
  } motor_modes_t;

  typedef enum
  {
    RPM = 0,
    CASCADED
  } input_modes_t;

  typedef struct CascadedAttitudeCommand
  {
    float thrust;
    quaternion_t q;
    float ang_vel_x, ang_vel_y, ang_vel_z;
    float ang_acc_x, ang_acc_y, ang_acc_z;
    float kR1, kR2, kR3;
    float kOm1, kOm2, kOm3;

    bool thrust_setpoint_set;
    bool attitude_setpoint_set;
    bool velocities_set;
    bool accelerations_set;
    bool proportional_gains_set;
    bool derivative_gains_set;

    CascadedAttitudeCommand() :
      thrust(0.0f),
      ang_vel_x(0.0f), ang_vel_y(0.0f), ang_vel_z(0.0f),
      ang_acc_x(0.0f), ang_acc_y(0.0f), ang_acc_z(0.0f),
      kR1(0.0f), kR2(0.0f), kR3(0.0f),
      kOm1(0.0f), kOm2(0.0f), kOm3(0.0f),
      thrust_setpoint_set(false),
      attitude_setpoint_set(false),
      velocities_set(false),
      accelerations_set(false),
      proportional_gains_set(false),
      derivative_gains_set(false) { }

    void print()
    {
      printf("casc att cmd:\n"
             "\tT = %0.2f\n"
             "\tQuat = [%0.2f, %0.2f, %0.2f, %0.2f]\n"
             "\tAng Vel= [%0.2f, %0.2f, %0.2f]\n"
             "\tAng Acc= [%0.2f, %0.2f, %0.2f]\n"
             "\tKR gains = [%0.2f, %0.2f, %0.2f]\n"
             "\tKOm gains = [%0.2f, %0.2f, %0.2f]\n",
             double(thrust),
             double(q.w), double(q.x), double(q.y), double(q.z),
             double(ang_vel_x), double(ang_vel_y), double(ang_vel_z),
             double(ang_acc_x), double(ang_acc_y), double(ang_acc_z),
             double(kR1), double(kR2), double(kR3),
             double(kOm1), double(kOm2), double(kOm3));
      printf("\tthrust_setpoint_set = %s\n"
             "\tattitude_setpoint_set = %s\n"
             "\tvelocities_set = %s\n"
             "\taccelerations_set = %s\n"
             "\tproportional_gains_set = %s\n"
             "\tderivative_gains_set = %s\n",
             thrust_setpoint_set ? "true" : "false",
             attitude_setpoint_set ? "true" : "false",
             velocities_set ? "true" : "false",
             accelerations_set ? "true" : "false",
             proportional_gains_set ? "true" : "false",
             derivative_gains_set ? "true" : "false");
    }
  } cascaded_attitude_command_t;

  // Start-up/cleanup calls
  bool loadParameters();
  bool registerCallbacks();
  void closeSubscriptions();

  // RPM <-> Force
  float convertRPMToForce(float rpm);
  float convertForceToRPM(float force);

  // Convert all inputs, RPM <-> Force
  void convertBodyForcesToRPM(const body_forces_t& b,
                              MotorManager::rpm_cmd_t& cmd);

  // Alt Controller w.r.t. cascaded attitude command inputs
  bool updateCascadedAttitudeController(MotorManager::rpm_cmd_t& cmd);

  // Input Callbacks
  bool commandMessageCallback(const vehicle_command_s& cmd);
  void controlStateMessageCallback(const control_state_s& ctrl_state);

  void setMotorMode(const motor_modes_t& m);
  void resetDesiredSetPoints();

  float unroll(float x);
  float normalize(float x);
  float shortest_angular_distance(float from, float to);

  // L1 Adaptive Control Helper Functions
  void initializeStatePredictor();
  void updateStatePredictor(float dt,
                            float wbx, float wby, float wbz,
                            float rpm1_des, float rpm2_des,
                            float rpm3_des, float rpm4_des);
  void initializeL1Controller();

  // Model parameters
  float cT, cT_inv;
  float gravity_magnitude, length, m_scale, motor_constant;
  float mass;
  float motor_spread_angle;

  // Mixing matrix
  float m21, m22, m23, m24;
  float m31, m32, m33, m34;
  float m41, m42, m43, m44;

  // Inverse mixing matrix
  float invm11, invm12, invm13, invm14;
  float invm21, invm22, invm23, invm24;
  float invm31, invm32, invm33, invm34;
  float invm41, invm42, invm43, invm44;

  uint64_t cmd_time;
  bool ctrl_state_set;

  int hl_sub;
  int ctrl_state_sub;

  struct pollfd fds[2];

  struct control_state_s ctrl_state;
  cascaded_attitude_command_t att_cmd_casc, att_cmd_cpu;

  motor_modes_t motor_mode;
  input_modes_t input_mode;

  float Ixx, Ixy, Ixz, Iyy, Iyz, Izz;
  float invIxx, invIxy, invIxz, invIyy, invIyz, invIzz;

  bool motor_start_set;
  uint64_t motor_start_dt_us;
  uint64_t tmotor_start;

  // L1AC Variables
  bool in_nominal_flight, rpm_set, avl_set, time_set;
  bool l1_initialized;
  float avlhat[3];
  float dsthat[3];
  float lpd[3];
  float rpmhat[4];
  float l1_engage_level;
  uint64_t tprev;

  // L1AC Parameters
  bool enable_l1;
  float bandwidth[3];
  float Kspx, Kspy, Kspz;
  float adaptation_gain;
  float init_disturbance[3];

  MotorManager mm;
  float rpm_min, rpm_max;

  uint64_t cmd_ttl_us;
};
#endif
