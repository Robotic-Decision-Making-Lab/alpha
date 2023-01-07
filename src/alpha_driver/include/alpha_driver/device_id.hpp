#pragma once

namespace alpha_driver
{

/**
 * @brief The device ID is a unique identifier used to denote each of the
 * manipulator's joints.
 *
 */
enum class DeviceId : unsigned char
{
  kLinearJaws = 0x01,
  kRotateEndEffector = 0x02,
  kBendElbow = 0x03,
  kBendShoulder = 0x04,
  kRotateBase = 0x05,
  kAllJoints = 0xFF,
};

}  // namespace alpha_driver
