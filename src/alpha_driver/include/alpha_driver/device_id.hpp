#pragma once

namespace alpha_driver {

/**
 * @brief The device ID is a unique identifier used to denote each of the
 * manipulator's joints.
 *
 */
enum class DeviceId : unsigned char {
  linearJaws = 0x01,
  rotateEndEffector = 0x02,
  bendElbow = 0x03,
  bendShoulder = 0x04,
  rotateBase = 0x05,
  allJoints = 0xFF,
};

}  // namespace alpha_driver
