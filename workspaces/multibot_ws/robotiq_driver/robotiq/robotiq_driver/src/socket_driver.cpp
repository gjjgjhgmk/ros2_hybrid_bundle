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

#include <robotiq_driver/socket_driver.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <stdexcept>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

namespace robotiq_driver
{
const auto kLogger = rclcpp::get_logger("SocketDriver");

constexpr int kSocketTimeoutSec = 5;
constexpr int kSocketTimeoutUsec = 0;

SocketDriver::SocketDriver(const std::string& robot_ip, uint16_t port, double max_position_mm)
  : robot_ip_(robot_ip)
  , port_(port)
  , max_position_mm_(max_position_mm)
  , socket_fd_(-1)
  , connected_(false)
  , slave_address_(0x09)  // Default slave address (not used with URCap)
  , gripper_position_(0x00)  // Start with gripper open
  , commanded_gripper_speed_(0x80)  // Default speed
  , commanded_gripper_force_(0x80)  // Default force
  , is_moving_(false)
{
}

bool SocketDriver::connect()
{
  std::lock_guard<std::mutex> lock(socket_mutex_);

  if (connected_)
  {
    RCLCPP_WARN(kLogger, "Already connected to %s:%d", robot_ip_.c_str(), port_);
    return true;
  }

  // Create socket
  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0)
  {
    RCLCPP_ERROR(kLogger, "Failed to create socket: %s", strerror(errno));
    return false;
  }

  // Set socket timeout
  struct timeval timeout;
  timeout.tv_sec = kSocketTimeoutSec;
  timeout.tv_usec = kSocketTimeoutUsec;
  if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
  {
    RCLCPP_WARN(kLogger, "Failed to set receive timeout: %s", strerror(errno));
  }
  if (setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0)
  {
    RCLCPP_WARN(kLogger, "Failed to set send timeout: %s", strerror(errno));
  }

  // Connect to robot
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port_);
  if (inet_pton(AF_INET, robot_ip_.c_str(), &server_addr.sin_addr) <= 0)
  {
    RCLCPP_ERROR(kLogger, "Invalid IP address: %s", robot_ip_.c_str());
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  if (::connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
  {
    RCLCPP_ERROR(kLogger, "Failed to connect to %s:%d: %s", robot_ip_.c_str(), port_, strerror(errno));
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  connected_ = true;
  RCLCPP_INFO(kLogger, "Connected to UR robot at %s:%d", robot_ip_.c_str(), port_);
  return true;
}

void SocketDriver::disconnect()
{
  std::lock_guard<std::mutex> lock(socket_mutex_);

  if (!connected_)
  {
    return;
  }

  if (socket_fd_ >= 0)
  {
    close(socket_fd_);
    socket_fd_ = -1;
  }

  connected_ = false;
  RCLCPP_INFO(kLogger, "Disconnected from UR robot");
}

void SocketDriver::set_slave_address(uint8_t slave_address)
{
  slave_address_ = slave_address;
  // In URCap mode, slave_address is used as gripper ID for function calls
  RCLCPP_INFO(kLogger, "Gripper ID set to: %d", slave_address_);
}

bool SocketDriver::send_script_command(const std::string& command)
{
  std::lock_guard<std::mutex> lock(socket_mutex_);

  if (!connected_ || socket_fd_ < 0)
  {
    RCLCPP_ERROR(kLogger, "Not connected to robot");
    return false;
  }

  // URScript commands need to end with newline
  std::string command_with_newline = command + "\n";

  // Send command
  ssize_t bytes_sent = send(socket_fd_, command_with_newline.c_str(), command_with_newline.length(), 0);
  if (bytes_sent < 0)
  {
    RCLCPP_ERROR(kLogger, "Failed to send command '%s': %s", command.c_str(), strerror(errno));
    return false;
  }

  if (static_cast<size_t>(bytes_sent) != command_with_newline.length())
  {
    RCLCPP_WARN(kLogger, "Partial send: sent %zd of %zu bytes", bytes_sent, command_with_newline.length());
  }

  // Read response (URScript typically sends back the command or a response)
  char buffer[1024];
  ssize_t bytes_received = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
  if (bytes_received < 0)
  {
    // Timeout or error - this is OK for URScript commands that don't return immediately
    if (errno == EAGAIN || errno == EWOULDBLOCK)
    {
      RCLCPP_INFO(kLogger, "No response received (timeout), command may still be executing");
      return true;
    }
    RCLCPP_WARN(kLogger, "Failed to receive response: %s", strerror(errno));
    return false;
  }

  buffer[bytes_received] = '\0';
  std::string response_str(buffer);
  
  // Log response (may be empty if command is executing)
  if (response_str.empty() || response_str.find_first_not_of(" \t\n\r") == std::string::npos)
  {
    RCLCPP_INFO(kLogger, "Received empty response (command may be executing asynchronously)");
  }
  else
  {
    RCLCPP_INFO(kLogger, "Received response: %s", response_str.c_str());
  }

  return true;
}

void SocketDriver::activate()
{
  RCLCPP_INFO(kLogger, "Activating gripper ID %d via URCap", slave_address_);
  // URCap function format: rq_activate_and_wait(id)
  std::string command = "rq_activate_and_wait(" + std::to_string(slave_address_) + ")";
  if (send_script_command(command))
  {
    RCLCPP_INFO(kLogger, "Gripper activation command sent");
  }
  else
  {
    RCLCPP_ERROR(kLogger, "Failed to activate gripper");
    throw std::runtime_error("Failed to activate gripper via URCap");
  }
}

void SocketDriver::deactivate()
{
  RCLCPP_WARN(kLogger, "Deactivate not typically used with URCap, sending reset command");
  // URCap may not have a deactivate function, so we just log a warning
  // In practice, deactivation might not be needed
}

void SocketDriver::set_gripper_position(uint8_t pos)
{
  // Convert position from 0x00-0xFF to millimeters
  double position_mm = position_to_mm(pos);
  
  RCLCPP_INFO(kLogger, "Setting gripper position: 0x%02X (%.2f mm) for gripper ID %d", pos, position_mm, slave_address_);
  
  // Try different URCap function formats (different URCap versions may use different function names)
  // Format 1: rq_move_and_wait_mm(id, position_mm) - most common
  std::string command = "rq_move_and_wait_mm(" + std::to_string(slave_address_) + ", " + std::to_string(position_mm) + ")";
  
  RCLCPP_INFO(kLogger, "Sending URScript command: %s", command.c_str());
  
  is_moving_ = true;
  if (send_script_command(command))
  {
    gripper_position_ = pos;
    // URCap functions are blocking, so movement should be complete when command returns
    // However, we wait a bit to ensure the command is processed
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    is_moving_ = false;
    RCLCPP_INFO(kLogger, "Gripper position command sent successfully to ID %d", slave_address_);
  }
  else
  {
    // If the first format fails, try alternative formats
    RCLCPP_WARN(kLogger, "First command format failed, trying alternative: rq_move_mm");
    command = "rq_move_mm(" + std::to_string(slave_address_) + ", " + std::to_string(position_mm) + ")";
    RCLCPP_INFO(kLogger, "Trying alternative command: %s", command.c_str());
    
    if (send_script_command(command))
    {
      gripper_position_ = pos;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      is_moving_ = false;
      RCLCPP_INFO(kLogger, "Gripper position command sent successfully (alternative format) to ID %d", slave_address_);
    }
    else
    {
      is_moving_ = false;
      RCLCPP_ERROR(kLogger, "Failed to set gripper position with all tried formats. Please check URCap function name.");
      RCLCPP_ERROR(kLogger, "Common URCap function names: rq_move_and_wait_mm, rq_move_mm, rq_move_and_wait, rq_move");
      throw std::runtime_error("Failed to set gripper position via URCap");
    }
  }
}

uint8_t SocketDriver::get_gripper_position()
{
  // For socket driver, querying actual position requires URScript evaluation
  // Robotiq URCap provides rq_get_pos_mm() function that returns position in mm
  // However, URScript socket communication doesn't easily support return values
  // 
  // Current limitation: When gripper is moved manually via teach pendant,
  // we can't detect the change because we only track last commanded position.
  // 
  // Workaround: Return last commanded position. For proper position tracking,
  // users should control gripper through ROS2 interface, not directly via teach pendant.
  return gripper_position_;
}

bool SocketDriver::gripper_is_moving()
{
  // URCap functions are blocking, so we track movement status
  return is_moving_;
}

void SocketDriver::set_speed(uint8_t speed)
{
  commanded_gripper_speed_ = speed;
  RCLCPP_INFO(kLogger, "Speed set to 0x%02X (URCap may not support speed control)", speed);
  // URCap may not support speed control, so we just store the value
}

void SocketDriver::set_force(uint8_t force)
{
  commanded_gripper_force_ = force;
  RCLCPP_INFO(kLogger, "Force set to 0x%02X (URCap may not support force control)", force);
  // URCap may not support force control, so we just store the value
}

double SocketDriver::position_to_mm(uint8_t pos) const
{
  // Convert 0x00-0xFF to 0-max_position_mm
  return (static_cast<double>(pos) / 255.0) * max_position_mm_;
}

uint8_t SocketDriver::mm_to_position(double position_mm) const
{
  // Convert millimeters to 0x00-0xFF
  if (position_mm < 0.0)
    position_mm = 0.0;
  if (position_mm > max_position_mm_)
    position_mm = max_position_mm_;
  
  return static_cast<uint8_t>((position_mm / max_position_mm_) * 255.0);
}

}  // namespace robotiq_driver

