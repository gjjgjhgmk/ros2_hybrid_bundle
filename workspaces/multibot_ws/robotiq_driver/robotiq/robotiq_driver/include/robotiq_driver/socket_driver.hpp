// Copyright (c) 2022 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <memory>
#include <string>
#include <mutex>

#include <robotiq_driver/driver.hpp>

/**
 * @brief This class is responsible for communicating with the gripper via Socket (URScript commands)
 * through Robotiq Grippers URCap.
 *
 */
namespace robotiq_driver
{
class SocketDriver : public Driver
{
public:
  /**
   * @brief Constructor
   * @param robot_ip IP address of the UR robot controller
   * @param port Socket port (default: 30002 for URScript)
   * @param max_position_mm Maximum gripper position in millimeters (85 for 2F-85, 140 for 2F-140)
   */
  explicit SocketDriver(const std::string& robot_ip, uint16_t port = 30002, double max_position_mm = 85.0);

  bool connect() override;
  void disconnect() override;

  void set_slave_address(uint8_t slave_address) override;

  /** Activate the gripper using URCap function. */
  void activate() override;

  /** Deactivate the gripper (not typically used with URCap). */
  void deactivate() override;

  /**
   * @brief Commands the gripper to move to the desired position.
   * @param pos A value between 0x00 (fully open) and 0xFF (fully closed).
   */
  void set_gripper_position(uint8_t pos) override;

  /**
   * @brief Return the current position of the gripper.
   * @return uint8_t A value between 0x00 (fully open) and 0xFF (fully closed).
   * @note URCap may not provide position query, so we track the last commanded position.
   */
  uint8_t get_gripper_position() override;

  /**
   * @brief Returns true if the gripper is currently moving, false otherwise.
   * @note URCap functions are blocking, so we assume movement completes when command returns.
   */
  bool gripper_is_moving() override;

  /**
   * @brief Set the speed of the gripper.
   * @param speed A value between 0x00 (stopped) and 0xFF (full speed).
   * @note URCap may not support speed control, this is a placeholder.
   */
  void set_speed(uint8_t speed) override;

  /**
   * @brief Set how forcefully the gripper opens or closes.
   * @param force A value between 0x00 (no force) or 0xFF (maximum force).
   * @note URCap may not support force control, this is a placeholder.
   */
  void set_force(uint8_t force) override;

private:
  /**
   * @brief Send URScript command through Socket and wait for response.
   * @param command URScript command string
   * @return true if command was sent successfully, false otherwise
   */
  bool send_script_command(const std::string& command);

  /**
   * @brief Convert position from 0x00-0xFF to millimeters.
   * @param pos Position value (0x00-0xFF)
   * @return Position in millimeters
   */
  double position_to_mm(uint8_t pos) const;

  /**
   * @brief Convert position from millimeters to 0x00-0xFF.
   * @param position_mm Position in millimeters
   * @return Position value (0x00-0xFF)
   */
  uint8_t mm_to_position(double position_mm) const;

  std::string robot_ip_;
  uint16_t port_;
  double max_position_mm_;
  int socket_fd_;
  bool connected_;
  std::mutex socket_mutex_;

  uint8_t slave_address_;  // Not used with URCap, but required by interface
  uint8_t gripper_position_;  // Track last commanded position
  uint8_t commanded_gripper_speed_;  // Track speed setting
  uint8_t commanded_gripper_force_;  // Track force setting
  bool is_moving_;  // Track movement status
};
}  // namespace robotiq_driver

