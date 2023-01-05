#pragma once

namespace alpha_driver {

/**
 * @brief A Packet ID is a unique identifer used to determine how to interpret
 * the packet data. For more information regarding each packet ID and its
 * respective data, please refer to the Reach System Communication Protocol
 * specification.
 *
 */
enum class PacketId : unsigned char {
  mode = 0x01,
  velocity = 0X02,
  position = 0x03,
  current = 0x05,
  relativePosition = 0x0E,
  indexedPosition = 0x0D,
  request = 0x60,
  serialNumber = 0x61,
  modelNumber = 0x62,
  temperature = 0x66,
  softwareVersion = 0x6C,
  kmEndPos = 0xA1,
  kmEndVel = 0xA2,
  kmEndVelLocal = 0xCB,
  kmBoxObstacle02 = 0xA5,
  kmBoxObstacle03 = 0xA5,
  kmBoxObstacle04 = 0xA5,
  kmBoxObstacle05 = 0xA5,
  kmCylinderObstacle02 = 0xAB,
  kmCylinderObstacle03 = 0xAC,
  kmCylinderObstacle04 = 0xAD,
  kmCylinderObstacle05 = 0xAE,
  voltage = 0x90,
  save = 0x50,
  heartbeatFreqency = 0x92,
  heartbeatSet = 0x91,
  positionLimits = 0x10,
  velocityLimits = 0x11,
  currentLimits = 0x12,
  atiFtReading = 0xD8,
  bootloader = 0xFF,
  voltageThresholdParameters = 0x99,
};

}  // namespace alpha_driver
