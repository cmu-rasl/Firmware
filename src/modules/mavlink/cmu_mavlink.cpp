#include <mathlib/mathlib.h>
#include <uORB/topics/time_offset.h>
#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/cascaded_command.h>
#include <uORB/topics/cascaded_command_gains.h>
#include <uORB/topics/mocap_motor_state.h>
#include <uORB/topics/mocap_rpm_command.h>
#include <uORB/topics/mocap_position_command.h>
#include <uORB/topics/mocap_position_command_gains.h>
#include <uORB/topics/blinkm_control.h>
#include <uORB/topics/charger_gpio.h>

#include "mavlink_main.h"
#include "cmu_mavlink.h"

CMUMavlink::CMUMavlink(Mavlink* parent) :
  mavlink(parent),
  system_id(0),
  time_offset_avg_alpha(0.6),
  time_offset(0),
  cascaded_command_pub(nullptr),
  cascaded_command_gains_pub(nullptr),
  mocap_motor_state_pub(nullptr),
  mocap_rpm_cmd_pub(nullptr),
  mocap_position_command_pub(nullptr),
  mocap_position_command_gains_pub(nullptr),
  blinkm_control_pub(nullptr),
  time_offset_pub(nullptr),
  att_pos_mocap_pub(nullptr),
  charger_gpio_pub(nullptr)
{
  return;
}

CMUMavlink::~CMUMavlink() { }

void CMUMavlink::set_system_id(int id)
{
  system_id = id;
}

void CMUMavlink::handle_message(const mavlink_message_t *msg)
{
  switch (msg->msgid)
  {
    case MAVLINK_MSG_ID_CASCADED_CMD:
      handle_message_cascaded_cmd(msg);
      break;
    case MAVLINK_MSG_ID_CASCADED_CMD_GAINS:
      handle_message_cascaded_cmd_gains(msg);
      break;
    case MAVLINK_MSG_ID_MOCAP_MOTOR_STATE:
      handle_message_mocap_motor_state(msg);
      break;
    case MAVLINK_MSG_ID_MOCAP_RPM_CMD:
      handle_message_mocap_rpm_cmd(msg);
      break;
    case MAVLINK_MSG_ID_MOCAP_TIMESYNC:
      handle_message_mocap_timesync(msg);
      break;
    case MAVLINK_MSG_ID_MOCAP_MULTI_POSE:
      handle_message_mocap_multi_pose(msg);
      break;
    case MAVLINK_MSG_ID_MOCAP_POSITION_CMD:
      handle_message_mocap_position_cmd(msg);
      break;
    case MAVLINK_MSG_ID_MOCAP_POSITION_CMD_GAINS:
      handle_message_mocap_position_cmd_gains(msg);
      break;
    case MAVLINK_MSG_ID_BLINKM_CONTROL:
      handle_message_blinkm_control(msg);
      break;
    case MAVLINK_MSG_ID_CHARGER_GPIO:
      handle_message_charger_gpio(msg);
  }
}

void CMUMavlink::handle_message_cascaded_cmd(const mavlink_message_t *msg)
{
  mavlink_cascaded_cmd_t mavlink_cascaded_cmd;
  mavlink_msg_cascaded_cmd_decode(msg, &mavlink_cascaded_cmd);

  if (mavlink_cascaded_cmd.target_system != system_id)
    return;

  struct cascaded_command_s cascaded_command;
  memset(&cascaded_command, 0, sizeof(cascaded_command));

  cascaded_command.timestamp = mavlink_cascaded_cmd.time_usec;

  cascaded_command.thrust = mavlink_cascaded_cmd.thrust;
  math::Quaternion q(mavlink_cascaded_cmd.q);
  q.normalize();

  for (unsigned int i = 0; i < 4; i++)
    cascaded_command.q[i] = q(i);

  for (unsigned int i = 0; i < 3; i++)
    cascaded_command.ang_vel[i] = mavlink_cascaded_cmd.ang_vel[i];

  for (unsigned int i = 0; i < 3; i++)
    cascaded_command.ang_acc[i] = mavlink_cascaded_cmd.ang_acc[i];

  int inst; // Not used
  orb_publish_auto(ORB_ID(cascaded_command), &cascaded_command_pub,
                   &cascaded_command, &inst, ORB_PRIO_HIGH);
}

void CMUMavlink::handle_message_cascaded_cmd_gains(const mavlink_message_t *msg)
{
  mavlink_cascaded_cmd_gains_t mavlink_cascaded_cmd_gains;
  mavlink_msg_cascaded_cmd_gains_decode(msg, &mavlink_cascaded_cmd_gains);

  if (mavlink_cascaded_cmd_gains.target_system != system_id)
    return;

  struct cascaded_command_gains_s cascaded_command_gains;
  memset(&cascaded_command_gains, 0, sizeof(cascaded_command_gains));

  cascaded_command_gains.timestamp = mavlink_cascaded_cmd_gains.time_usec;

  for (unsigned int i = 0; i < 3; i++)
    cascaded_command_gains.kR[i] = mavlink_cascaded_cmd_gains.kR[i];

  for (unsigned int i = 0; i < 3; i++)
    cascaded_command_gains.kOm[i] = mavlink_cascaded_cmd_gains.kOm[i];

  int inst; // Not used
  orb_publish_auto(ORB_ID(cascaded_command_gains), &cascaded_command_gains_pub,
                   &cascaded_command_gains, &inst, ORB_PRIO_HIGH);
}

void CMUMavlink::handle_message_mocap_motor_state(const mavlink_message_t *msg)
{
  mavlink_mocap_motor_state_t mavlink_mocap_motor_state;
  mavlink_msg_mocap_motor_state_decode(msg, &mavlink_mocap_motor_state);

  if (mavlink_mocap_motor_state.target_system != system_id)
    return;

  struct mocap_motor_state_s mocap_motor_state;
  memset(&mocap_motor_state, 0, sizeof(mocap_motor_state));

  mocap_motor_state.timestamp = mavlink_mocap_motor_state.time_usec;
  mocap_motor_state.state = mavlink_mocap_motor_state.state;

  int inst; // Not used
  orb_publish_auto(ORB_ID(mocap_motor_state), &mocap_motor_state_pub,
                   &mocap_motor_state, &inst, ORB_PRIO_HIGH);
}

void CMUMavlink::handle_message_mocap_rpm_cmd(const mavlink_message_t *msg)
{
  mavlink_mocap_rpm_cmd_t mavlink_mocap_rpm_cmd;
  mavlink_msg_mocap_rpm_cmd_decode(msg, &mavlink_mocap_rpm_cmd);

  if (mavlink_mocap_rpm_cmd.target_system != system_id)
    return;

  struct mocap_rpm_command_s mocap_rpm_cmd;
  memset(&mocap_rpm_cmd, 0, sizeof(mocap_rpm_cmd));

  mocap_rpm_cmd.timestamp = mavlink_mocap_rpm_cmd.time_usec;
  mocap_rpm_cmd.ninputs = mavlink_mocap_rpm_cmd.ninputs;

  for (unsigned int i = 0; i < mavlink_mocap_rpm_cmd.ninputs; i++)
    mocap_rpm_cmd.input[i] = mavlink_mocap_rpm_cmd.input[i];

  int inst; // Not used
  orb_publish_auto(ORB_ID(mocap_rpm_command), &mocap_rpm_cmd_pub,
                   &mocap_rpm_cmd, &inst, ORB_PRIO_HIGH);
}

void CMUMavlink::handle_message_mocap_timesync(const mavlink_message_t *msg)
{
  // Specialized handler to ensure that time sync is wrt specific systems
  mavlink_mocap_timesync_t tsync;
  mavlink_msg_mocap_timesync_decode(msg, &tsync);

  if (tsync.target_system != system_id)
    return;

  struct time_offset_s tsync_offset;
  memset(&tsync_offset, 0, sizeof(tsync_offset));

  uint64_t now_ns = hrt_absolute_time() * 1000LL;

  if (tsync.tc1 == 0)
  {
    mavlink_timesync_t rsync; // return timestamped sync message

    rsync.tc1 = now_ns;
    rsync.ts1 = tsync.ts1;

    mavlink->send_message(MAVLINK_MSG_ID_TIMESYNC, &rsync);
    return;
  }
  else if (tsync.tc1 > 0)
  {
    int64_t offset_ns = (tsync.ts1 + now_ns - tsync.tc1*2)/2;
    int64_t dt = time_offset - offset_ns;

    if (dt > 10000000LL || dt < -10000000LL)
    {
      time_offset = offset_ns;
      //printf("[timesync] Hard setting offset (%lld)\n", dt);
    }
    else
      smooth_time_offset(offset_ns);
  }

  tsync_offset.offset_ns = time_offset;

  int inst; // Not used
  orb_publish_auto(ORB_ID(time_offset), &time_offset_pub,
                   &tsync_offset, &inst, ORB_PRIO_HIGH);
}

void CMUMavlink::handle_message_mocap_multi_pose(const mavlink_message_t *msg)
{
  mavlink_mocap_multi_pose_t mpose;
  mavlink_msg_mocap_multi_pose_decode(msg, &mpose);

  int indx = -1;
  for (unsigned int i = 0; i < mpose.npose; i++)
    if (mpose.ids[i] == system_id)
    {
      indx = i;
      break;
    }

  if (indx == -1)
    return;

  struct att_pos_mocap_s att_pos_mocap;
  memset(&att_pos_mocap, 0, sizeof(att_pos_mocap));

  // Use the component ID to identify the mocap system
  att_pos_mocap.id = msg->compid;

  att_pos_mocap.timestamp_boot = hrt_absolute_time(); // Monotonic time
  att_pos_mocap.timestamp_computer = sync_stamp(mpose.time_usec); // Synced time

  unsigned int k = 4*indx;
  att_pos_mocap.x = mpose.pose[k++]/1000.0f;
  att_pos_mocap.y = mpose.pose[k++]/1000.0f;
  att_pos_mocap.z = mpose.pose[k++]/1000.0f;

  float heading = mpose.pose[k++]/10000.0f;
  math::Quaternion mq;
  mq.from_yaw(heading);

#if 0
  // Hack to address NuttX printf issue re: handling of float/double
  char buf[128];
  sprintf(buf, "position = %0.5f, %0.5f, %0.5f, heading = %0.5f",
          (double)att_pos_mocap.x, (double)att_pos_mocap.y,
          (double)att_pos_mocap.z, (double)heading);
  printf("%s\n", buf);
#endif

  att_pos_mocap.q[0] = mq(0);
  att_pos_mocap.q[1] = mq(1);
  att_pos_mocap.q[2] = mq(2);
  att_pos_mocap.q[3] = mq(3);

  int inst; // Not used
  orb_publish_auto(ORB_ID(att_pos_mocap), &att_pos_mocap_pub,
                   &att_pos_mocap, &inst, ORB_PRIO_HIGH);
}

void CMUMavlink::handle_message_mocap_position_cmd(const mavlink_message_t *msg)
{
  mavlink_mocap_position_cmd_t mcmd;
  mavlink_msg_mocap_position_cmd_decode(msg, &mcmd);

  if (mcmd.target_system != system_id)
    return;

  struct mocap_position_command_s cmd;
  memset(&cmd, 0, sizeof(cmd));

  cmd.timestamp = mcmd.time_usec;
  for (unsigned int i = 0; i < 3; i++)
  {
    cmd.pos[i] = static_cast<float>(mcmd.pos[i])*1e-3f;
    cmd.vel[i] = static_cast<float>(mcmd.vel[i])*1e-3f;
    cmd.acc[i] = static_cast<float>(mcmd.acc[i])*1e-3f;
    cmd.jerk[i] = static_cast<float>(mcmd.jerk[i])*1e-3f;
    cmd.heading[i] = static_cast<float>(mcmd.heading[i])*1e-4f;
  }

  int inst; // Not used
  orb_publish_auto(ORB_ID(mocap_position_command), &mocap_position_command_pub,
                   &cmd, &inst, ORB_PRIO_HIGH);
}

void CMUMavlink::handle_message_mocap_position_cmd_gains(const mavlink_message_t *msg)
{
  mavlink_mocap_position_cmd_gains_t mcmd_gains;
  mavlink_msg_mocap_position_cmd_gains_decode(msg, &mcmd_gains);

  if (mcmd_gains.target_system != system_id)
    return;

  struct mocap_position_command_gains_s cmd_gains;
  memset(&cmd_gains, 0, sizeof(cmd_gains));

  cmd_gains.timestamp = mcmd_gains.time_usec;
  for (unsigned int i = 0; i < 3; i++)
  {
    cmd_gains.kp[i] = mcmd_gains.kp[i];
    cmd_gains.kd[i] = mcmd_gains.kd[i];
  }

  int inst; // Not used
  orb_publish_auto(ORB_ID(mocap_position_command_gains),
                   &mocap_position_command_gains_pub,
                   &mcmd_gains, &inst, ORB_PRIO_HIGH);
}

void CMUMavlink::handle_message_blinkm_control(const mavlink_message_t *msg)
{
  mavlink_blinkm_control_t mb;
  mavlink_msg_blinkm_control_decode(msg, &mb);

  if (mb.target_system != system_id)
    return;

  struct blinkm_control_s bc;
  memset(&bc, 0, sizeof(bc));

  bc.control = mb.control;

  int inst; // Not used
  orb_publish_auto(ORB_ID(blinkm_control), &blinkm_control_pub,
                   &bc, &inst, ORB_PRIO_HIGH);
}

void CMUMavlink::handle_message_charger_gpio(const mavlink_message_t *msg)
{
  mavlink_charger_gpio_t mc;
  mavlink_msg_charger_gpio_decode(msg, &mc);

  if (mc.target_system != system_id)
    return;

  struct charger_gpio_s cg;
  memset(&cg, 0, sizeof(cg));

  cg.on = mc.on;

  int inst; // Not used
  orb_publish_auto(ORB_ID(charger_gpio), &charger_gpio_pub, 
                    &cg, &inst, ORB_PRIO_HIGH);

}
