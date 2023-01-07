#pragma once

namespace alpha_driver
{

/**
 * @brief A Packet ID is a unique identifer used to determine how to interpret
 * the packet data. For more information regarding each packet ID and its
 * respective data, please refer to the Reach System Communication Protocol
 * specification.
 *
 */
enum class PacketId : unsigned char
{
  kMode = 0x01,
  kVelocity = 0X02,
  kPosition = 0x03,
  kCurrent = 0x05,
  kRelativePosition = 0x0E,
  kIndexedPosition = 0x0D,
  kRequest = 0x60,
  kSerialNumber = 0x61,
  kModelNumber = 0x62,
  kTemperature = 0x66,
  kSoftwareVersion = 0x6C,
  kKmEndPos = 0xA1,
  kKmEndVel = 0xA2,
  kKmEndVelLocal = 0xCB,
  kKmBoxObstacle02 = 0xA5,
  kKmBoxObstacle03 = 0xA5,
  kKmBoxObstacle04 = 0xA5,
  kKmBoxObstacle05 = 0xA5,
  kKmCylinderObstacle02 = 0xAB,
  kKmCylinderObstacle03 = 0xAC,
  kKmCylinderObstacle04 = 0xAD,
  kKmCylinderObstacle05 = 0xAE,
  kVoltage = 0x90,
  kSave = 0x50,
  kHeartbeatFreqency = 0x92,
  kHeartbeatSet = 0x91,
  kPositionLimits = 0x10,
  kVelocityLimits = 0x11,
  kCurrentLimits = 0x12,
  kAtiFtReading = 0xD8,
  kBootloader = 0xFF,
  kVoltageThresholdParameters = 0x99,
};

}  // namespace alpha_driver
