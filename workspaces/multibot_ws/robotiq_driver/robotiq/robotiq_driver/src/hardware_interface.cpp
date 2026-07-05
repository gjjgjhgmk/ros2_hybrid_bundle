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

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <robotiq_driver/default_driver_factory.hpp>
#include <robotiq_driver/hardware_interface.hpp>

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <rclcpp/rclcpp.hpp>

const auto kLogger = rclcpp::get_logger("RobotiqGripperHardwareInterface");

constexpr uint8_t kGripperMinPos = 3;
constexpr uint8_t kGripperMaxPos = 230;
constexpr double kGripperMaxSpeed = 0.150;  // mm/s
constexpr double kGripperMaxforce = 235;    // N
constexpr uint8_t kGripperRange = kGripperMaxPos - kGripperMinPos;

constexpr auto kGripperCommsLoopPeriod = std::chrono::milliseconds{ 10 };

namespace robotiq_driver
{
RobotiqGripperHardwareInterface::RobotiqGripperHardwareInterface()
{
  driver_factory_ = std::make_unique<DefaultDriverFactory>();
}

RobotiqGripperHardwareInterface::~RobotiqGripperHardwareInterface()
{
  communication_thread_is_running_.store(false);
  if (communication_thread_.joinable())
  {
    communication_thread_.join();
  }
}

// This constructor is use for testing only.
RobotiqGripperHardwareInterface::RobotiqGripperHardwareInterface(std::unique_ptr<DriverFactory> driver_factory)
  : driver_factory_{ std::move(driver_factory) }
{
}

hardware_interface::CallbackReturn
RobotiqGripperHardwareInterface::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
  RCLCPP_DEBUG(kLogger, "on_init");

  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Read parameters.
  gripper_closed_pos_ = stod(info_.hardware_parameters.at("gripper_closed_position"));
  gripper_max_speed_ = info_.hardware_parameters.count("gripper_max_speed") ?
                           stod(info_.hardware_parameters.at("gripper_max_speed")) :
                           kGripperMaxSpeed;
  gripper_max_force_ = info_.hardware_parameters.count("gripper_max_force") ?
                           stod(info_.hardware_parameters.at("gripper_max_force")) :
                           kGripperMaxforce;
  gripper_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_velocity_ = std::numeric_limits<double>::quiet_NaN();
  gripper_position_command_ = std::numeric_limits<double>::quiet_NaN();
  reactivate_gripper_cmd_ = NO_NEW_CMD_;
  reactivate_gripper_async_cmd_.store(false);

  const hardware_interface::ComponentInfo& joint = info_.joints.at(0);

  // There is one command interface: position.
  if (joint.command_interfaces.size() != 1)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                 joint.command_interfaces.size());
    return CallbackReturn::ERROR;
  }

  if (joint.command_interfaces.at(0).name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has %s command interfaces found. '%s' expected.", joint.name.c_str(),
                 joint.command_interfaces.at(0).name.c_str(), hardware_interface::HW_IF_POSITION);
    return CallbackReturn::ERROR;
  }

  // There are two state interfaces: position and velocity.
  if (joint.state_interfaces.size() != 2)
  {
    RCLCPP_FATAL(kLogger, "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                 joint.state_interfaces.size());
    return CallbackReturn::ERROR;
  }

  for (int i = 0; i < 2; ++i)
  {
    if (!(joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[i].name == hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_FATAL(kLogger, "Joint '%s' has %s state interface. Expected %s or %s.", joint.name.c_str(),
                   joint.state_interfaces.at(i).name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  try
  {
    driver_ = driver_factory_->create(info_);
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(kLogger, "Failed to create a driver: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RobotiqGripperHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(kLogger, "on_configure");
  try
  {
    if (hardware_interface::SystemInterface::on_configure(previous_state) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Open the serial port and handshake.
    bool connected = driver_->connect();
    if (!connected)
    {
      RCLCPP_ERROR(kLogger, "Cannot connect to the Robotiq gripper");
      return CallbackReturn::ERROR;
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Cannot configure the Robotiq gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotiqGripperHardwareInterface::export_state_interfaces()
{
  RCLCPP_DEBUG(kLogger, "export_state_interfaces");

  std::vector<hardware_interface::StateInterface> state_interfaces;

  // 为每个关节创建独立的状态变量，根据 mimic 关系在 read() 中更新
  for (const auto& joint : info_.joints)
  {
    if (joint_positions_.find(joint.name) == joint_positions_.end())
    {
      joint_positions_[joint.name] = 0.0;
      joint_velocities_[joint.name] = 0.0;
    }
    
    for (const auto& state_interface : joint.state_interfaces)
    {
      if (state_interface.name == hardware_interface::HW_IF_POSITION)
      {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_POSITION, &joint_positions_[joint.name]));
      }
      else if (state_interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[joint.name]));
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotiqGripperHardwareInterface::export_command_interfaces()
{
  RCLCPP_DEBUG(kLogger, "export_command_interfaces");

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &gripper_position_command_));

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[0].name, "set_gripper_max_velocity", &gripper_speed_));
  gripper_speed_ = kGripperMaxSpeed * (info_.hardware_parameters.count("gripper_speed_multiplier") ?
                                           std::stod(info_.hardware_parameters.at("gripper_speed_multiplier")) :
                                           1.0);

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[0].name, "set_gripper_max_effort", &gripper_force_));
  gripper_force_ = kGripperMaxforce * (info_.hardware_parameters.count("gripper_force_multiplier") ?
                                           std::stod(info_.hardware_parameters.at("gripper_force_multiplier")) :
                                           1.0);

  // 从 URDF 中读取 GPIO 名称
  std::string reactivate_gripper_name = "reactivate_gripper";
  if (!info_.gpios.empty())
  {
    reactivate_gripper_name = info_.gpios[0].name;
  }

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(reactivate_gripper_name, "reactivate_gripper_cmd", &reactivate_gripper_cmd_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      reactivate_gripper_name, "reactivate_gripper_response", &reactivate_gripper_response_));

  return command_interfaces;
}

hardware_interface::CallbackReturn
RobotiqGripperHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_DEBUG(kLogger, "on_activate");

  // set some default values for joints
  if (std::isnan(gripper_position_))
  {
    gripper_position_ = 0;
    gripper_velocity_ = 0;
    gripper_position_command_ = 0;
  }

  // Activate the gripper.
  try
  {
    driver_->deactivate();
    driver_->activate();

    communication_thread_is_running_.store(true);
    communication_thread_ = std::thread([this] { this->background_task(); });
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(kLogger, "Failed to communicate with the Robotiq gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(kLogger, "Robotiq Gripper successfully activated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
RobotiqGripperHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_DEBUG(kLogger, "on_deactivate");

  communication_thread_is_running_.store(false);
  communication_thread_.join();
  if (communication_thread_.joinable())
  {
    communication_thread_.join();
  }

  try
  {
    driver_->deactivate();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(kLogger, "Failed to deactivate the Robotiq gripper: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(kLogger, "Robotiq Gripper successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotiqGripperHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                                      const rclcpp::Duration& /*period*/)
{
  gripper_position_ = gripper_closed_pos_ * (gripper_current_state_.load() - kGripperMinPos) / kGripperRange;
  gripper_velocity_ = 0.0;

  // 根据 mimic 关系更新所有关节的状态
  std::string main_joint_name = info_.joints[0].name;
  double main_position = gripper_position_;
  double main_velocity = gripper_velocity_;
  
  // 确保所有关节的状态变量都已初始化
  for (const auto& joint : info_.joints)
  {
    if (joint_positions_.find(joint.name) == joint_positions_.end())
    {
      joint_positions_[joint.name] = 0.0;
      joint_velocities_[joint.name] = 0.0;
    }
  }
  
  // 更新所有关节的状态，根据 mimic 关系
  for (auto& [joint_name, position] : joint_positions_)
  {
    if (joint_name == main_joint_name)
    {
      position = main_position;
    }
    else if (joint_name.find("right_knuckle_joint") != std::string::npos)
    {
      position = -main_position;
    }
    else if (joint_name.find("left_inner_knuckle_joint") != std::string::npos)
    {
      position = main_position;
    }
    else if (joint_name.find("right_inner_knuckle_joint") != std::string::npos)
    {
      position = -main_position;
    }
    else if (joint_name.find("left_finger_tip_joint") != std::string::npos)
    {
      position = -main_position;
    }
    else if (joint_name.find("right_finger_tip_joint") != std::string::npos)
    {
      position = main_position;
    }
    else
    {
      position = main_position;
    }
  }
  
  // 更新速度
  for (auto& [joint_name, velocity] : joint_velocities_)
  {
    if (joint_name.find("right_knuckle_joint") != std::string::npos ||
        joint_name.find("right_inner_knuckle_joint") != std::string::npos ||
        joint_name.find("left_finger_tip_joint") != std::string::npos)
    {
      velocity = -main_velocity;
    }
    else
    {
      velocity = main_velocity;
    }
  }

  if (!std::isnan(reactivate_gripper_cmd_))
  {
    RCLCPP_INFO(kLogger, "Sending gripper reactivation request.");
    reactivate_gripper_async_cmd_.store(true);
    reactivate_gripper_cmd_ = NO_NEW_CMD_;
  }

  if (reactivate_gripper_async_response_.load().has_value())
  {
    reactivate_gripper_response_ = reactivate_gripper_async_response_.load().value();
    reactivate_gripper_async_response_.store(std::nullopt);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotiqGripperHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                                       const rclcpp::Duration& /*period*/)
{
  double gripper_pos = (gripper_position_command_ / gripper_closed_pos_) * kGripperRange + kGripperMinPos;
  gripper_pos = std::max(std::min(gripper_pos, 255.0), 0.0);
  write_command_.store(uint8_t(gripper_pos));
  const auto gripper_speed_multiplier = std::clamp(fabs(gripper_speed_) / gripper_max_speed_, 0.0, 1.0);
  write_speed_.store(uint8_t(gripper_speed_multiplier * 0xFF));
  const auto gripper_force_multiplier = std::clamp(fabs(gripper_force_) / gripper_max_force_, 0.0, 1.0);
  write_force_.store(uint8_t(gripper_force_multiplier * 0xFF));

  return hardware_interface::return_type::OK;
}

void RobotiqGripperHardwareInterface::background_task()
{
  while (communication_thread_is_running_.load())
  {
    try
    {
      // Re-activate the gripper
      // (this can be used, for example, to re-run the auto-calibration).
      if (reactivate_gripper_async_cmd_.load())
      {
        this->driver_->deactivate();
        this->driver_->activate();
        reactivate_gripper_async_cmd_.store(false);
        reactivate_gripper_async_response_.store(true);
      }

      // Write the latest command to the gripper.
      this->driver_->set_gripper_position(write_command_.load());
      this->driver_->set_speed(write_speed_.load());
      this->driver_->set_force(write_force_.load());

      // Read the state of the gripper.
      gripper_current_state_.store(this->driver_->get_gripper_position());
    }
    catch (std::exception& e)
    {
      RCLCPP_ERROR(kLogger, "Error: %s", e.what());
    }

    std::this_thread::sleep_for(kGripperCommsLoopPeriod);
  }
}

}  // namespace robotiq_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robotiq_driver::RobotiqGripperHardwareInterface, hardware_interface::SystemInterface)
