// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef G1_HARDWARE__G1_HARDWARE_HPP_
#define G1_HARDWARE__G1_HARDWARE_HPP_


#include <map>
#include <vector>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "g1_hardware/visiblity_control.h"
#include "rclcpp/macros.hpp"

#include <memory>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace g1_hardware
{
struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint
{
  JointValue state{};
  JointValue command{};
  JointValue prev_command{};
};

enum class ControlMode
{
  Position,
  Velocity,
  Torque,
};

class G1Hardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(G1Hardware)

  G1_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  G1_HARDWARE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  G1_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  G1_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  G1_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  G1_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  G1_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  G1_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> commanded_positions_;  // Intermediate buffer
  double max_joint_velocity_ = 0.5;         // rad/s (adjust per joint)
  double control_period_ = 0.002;           // 500Hz (match your update rate)
  // Communication members
  std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>> low_state_subscriber_;
  std::shared_ptr<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>> low_cmd_publisher_;

  // Última mensagem recebida do robô
  unitree_hg::msg::dds_::LowState_ state_msg_;
  unitree_hg::msg::dds_::LowCmd_ cmd_msg_;
  // Mutex para proteger acesso concorrente à mensagem
  std::mutex state_mutex_;
  // Control gains 

  const float kp_{60.0};  // Position gain
  const float kd_{1.5};  // Velocity gain
  const float tau_ff_{0.0};  // Feed-forward torque
  return_type enable_torque(const bool enabled);

  return_type set_control_mode(const ControlMode & mode, const bool force_set = false);

  return_type reset_command();

  CallbackReturn set_joint_positions();
  CallbackReturn set_joint_velocities();
  CallbackReturn set_joint_params();


  std::vector<Joint> joints_;
  std::vector<uint8_t> joint_ids_;
  bool torque_enabled_{false};
  ControlMode control_mode_{ControlMode::Position};
};
}  // namespace g1_hardware

#endif  // G1_HARDWARE__G1_HARDWARE_HPP_