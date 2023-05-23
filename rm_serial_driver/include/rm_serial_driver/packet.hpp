// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t enemy_color : 1;     // 敌方颜色
  uint8_t game_status : 4;     // 比赛状态
  uint8_t outpost_status : 1;  // 前哨站状态
  uint8_t reserved : 2;
  float curr_pitch;
  float curr_yaw;
  // **********  new  ************//
  float aim_x;
  float aim_y;
  float aim_z;
  // **********  new  ************//
  int16_t v_liner_x;    // Linear speed in x direction. >0: forward | <0: backward (cm/s)
  int16_t v_liner_y;    // Linear speed in y direction. 0 if differential.(cm/s)
  int16_t v_angular_z;  // Angular speed in z direction >0: turn left | <0: turn right (0.01 rad/s)
  int16_t odom_yaw;     // Odometry theta (0.01 rad)
  uint16_t checksum = 0;
} __attribute__((packed));

// TODO 
struct SendPacketArmor
{
  uint8_t header = 0xA5;
  // auto_aim_interfaces/msg/target
  bool tracking : 1;
  // 0: Armor_state  1: Outpost   2: Balace_state
  uint8_t task_mode : 2;
  // bool suggest_fire : 1;
  uint8_t reserve : 5;
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacketMove
{
  uint8_t header = 0xB5;
  // geometry_msgs/msg/Twist
  float linear_x;   // (m/s)
  float linear_y;   // (m/s)
  float angular_z;  // (rad/s)
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVectorArmor(const SendPacketArmor & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacketArmor));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacketArmor), packet.begin());
  return packet;
}

inline std::vector<uint8_t> toVectorMove(const SendPacketMove & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacketMove));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacketMove), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
