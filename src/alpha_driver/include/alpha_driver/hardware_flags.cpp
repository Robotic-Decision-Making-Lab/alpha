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

namespace alpha_driver
{

/**
 * @brief Hardware status flags that are transmitted at a frequence of 1 Hz if
 * any errors are set.
 *
 */
enum class HardwareStatusFlag : unsigned char
{
  // Byte A
  kFlashFailedRead = 0x80,
  kHardwareOverHumidity = 0x40,
  kHardwareOverTemperature = 0x20,
  kCommsSerialError = 0x10,
  kCommsCrcError = 0x08,
  kMotorDriverFault = 0x04,
  kEncoderPositionError = 0x02,
  kEncoderNotDetected = 0x01,
  // Byte B
  kDeviceAxisConflict = 0x80,
  kMotorNotConnected = 0x40,
  kMotorOverCurrent = 0x20,
  kInnerEncoderPositionError = 0x10,
  kDeviceIdConflict = 0x08,
  kHardwareOverPressure = 0x04,
  kMotorDriverOverCurrentAndUnderVoltage = 0x02,
  kMotorDriverOverTemperature = 0x01,
  // Byte D (Byte C is unused)
  kInvalidFirmware = 0x04,
  kCanbusError = 0x02,
  kPositionReportNotReceived = 0x01
};

}  // namespace alpha_driver
