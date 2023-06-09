// Copyright 2023, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

namespace alpha::driver
{

/**
 * @brief A unique identifier used to determine how to interpret the packet data.
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
  kKmBoxObstacle03 = 0xA6,
  kKmBoxObstacle04 = 0xA7,
  kKmBoxObstacle05 = 0xA8,
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

}  // namespace alpha::driver
