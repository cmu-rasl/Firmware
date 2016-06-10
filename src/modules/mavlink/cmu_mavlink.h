#pragma once

#include <uORB/uORB.h>
#include <v1.0/cmu_mavlink/mavlink.h>

class Mavlink;

class CMUMavlink
{
public:
  ~CMUMavlink();

  static CMUMavlink *new_instance(Mavlink* mavlink)
  {
    return new CMUMavlink(mavlink);
  }

  void set_system_id(int id);
  void handle_message(const mavlink_message_t *msg);

private:
  explicit CMUMavlink(Mavlink* parent);

  void handle_message_cascaded_cmd(const mavlink_message_t *msg);
  void handle_message_cascaded_cmd_gains(const mavlink_message_t *msg);
  void handle_message_mocap_motor_state(const mavlink_message_t *msg);
  void handle_message_mocap_rpm_cmd(const mavlink_message_t *msg);
  void handle_message_mocap_timesync(const mavlink_message_t *msg);
  void handle_message_mocap_multi_pose(const mavlink_message_t *msg);
  void handle_message_mocap_position_cmd(const mavlink_message_t *msg);
  void handle_message_mocap_position_cmd_gains(const mavlink_message_t *msg);
  void handle_message_blinkm_control(const mavlink_message_t *msg);
  void handle_message_charger_gpio(const mavlink_message_t *msg);

  // Copied from MavlinkReceiver
  uint64_t sync_stamp(uint64_t usec)
  {
    if (time_offset > 0)
      return usec - (time_offset / 1000);
    else
      return hrt_absolute_time();
  }

  void smooth_time_offset(uint64_t offset_ns)
  {
    /* alpha = 0.6 fixed for now. The closer alpha is to 1.0,
     * the faster the moving average updates in response to
     * new offset samples.
     */
    time_offset =
      (time_offset_avg_alpha*offset_ns) + (1.0 - time_offset_avg_alpha)*time_offset;
  }

  Mavlink* mavlink;
  int system_id;
  double time_offset_avg_alpha;
  uint64_t time_offset;

  orb_advert_t cascaded_command_pub;
  orb_advert_t cascaded_command_gains_pub;
  orb_advert_t mocap_motor_state_pub;
  orb_advert_t mocap_rpm_cmd_pub;
  orb_advert_t mocap_position_command_pub;
  orb_advert_t mocap_position_command_gains_pub;
  orb_advert_t blinkm_control_pub;
  orb_advert_t time_offset_pub;
  orb_advert_t att_pos_mocap_pub;
  orb_advert_t charger_gpio_pub;
};
