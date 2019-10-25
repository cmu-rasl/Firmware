#ifndef CMU_MAVLINK_HANDLER_H
#define CMU_MAVLINK_HANDLER_H

#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>

#include "mavlink_timesync.h"

class CMUMavlinkHandler
{
public:
  ~CMUMavlinkHandler();

  static CMUMavlinkHandler *new_instance()
  {
    return new CMUMavlinkHandler();
  }

  void set_system_id(int id);
  void set_channel(const mavlink_channel_t& channel);

  void handle_message(const mavlink_message_t *msg);

private:
  explicit CMUMavlinkHandler();

  void handle_message_cascaded_cmd(const mavlink_message_t *msg);
  void handle_message_cascaded_cmd_gains(const mavlink_message_t *msg);
  void handle_message_mocap_motor_state(const mavlink_message_t *msg);
  void handle_message_mocap_rpm_cmd(const mavlink_message_t *msg);
  void handle_message_mocap_pwm_debug(const mavlink_message_t *msg);
  void handle_message_mocap_timesync(const mavlink_message_t *msg);
  void handle_message_mocap_pose(const mavlink_message_t *msg);
  void handle_message_mocap_multi_pose(const mavlink_message_t *msg);
  void handle_message_mocap_position_cmd(const mavlink_message_t *msg);
  void handle_message_mocap_position_cmd_gains(const mavlink_message_t *msg);

  void pack_publish_mocap(uint64_t time,
                          float x, float y, float z, float heading);

  int system_id;
  mavlink_channel_t channel;

	MavlinkTimesync	mavlink_timesync;

  orb_advert_t cascaded_command_pub;
  orb_advert_t cascaded_command_gains_pub;
  orb_advert_t mocap_motor_state_pub;
  orb_advert_t mocap_rpm_cmd_pub;
  orb_advert_t mocap_pwm_cmd_pub;
  orb_advert_t mocap_position_command_pub;
  orb_advert_t mocap_position_command_gains_pub;
  orb_advert_t vehicle_mocap_odometry_pub;
};
#endif
