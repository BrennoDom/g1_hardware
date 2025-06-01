// Copyright 2024 Brenno Domingues <brennohdomingues@gmail.com>
//

#include "g1_hardware/g1_hardware.hpp"
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/idl/go2/MotorState_.hpp>

static const std::string kTopicArmSDK = "rt/arm_sdk";
static const std::string kTopicState = "rt/lowstate";
constexpr float kPi = 3.141592654;
constexpr float kPi_2 = 1.57079632;
#include <unitree/idl/hg/LowState_.hpp>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>


#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

/*###############################################DECLARATION#################################################
####################################TODO: PASS TO HEADER FILE################################################
#############################################################################################################
#############################################################################################################*/
float weight_ = 0.0f;
float weight = 0.f;
const float weight_rate_ = 0.2f;
const char* networkInterface = "enp0s31f6";  

namespace g1_hardware
{

  
enum JointIndex {
    // Left leg
    kLeftHipPitch,
    kLeftHipRoll,
    kLeftHipYaw,
    kLeftKnee,
    kLeftAnkle,
    kLeftAnkleRoll,

    // Right leg
    kRightHipPitch,
    kRightHipRoll,
    kRightHipYaw,
    kRightKnee,
    kRightAnkle,
    kRightAnkleRoll,

    kWaistYaw,
    kWaistRoll,
    kWaistPitch,

    // Left arm
    kLeftShoulderPitch,
    kLeftShoulderRoll,
    kLeftShoulderYaw,
    kLeftElbow,
    kLeftWristRoll,
    kLeftWristPitch,
    kLeftWristYaw,
    // Right arm
    kRightShoulderPitch,
    kRightShoulderRoll,
    kRightShoulderYaw,
    kRightElbow,
    kRightWristRoll,
    kRightWristPitch,
    kRightWristYaw,

    kNotUsedJoint,
    kNotUsedJoint1,
    kNotUsedJoint2,
    kNotUsedJoint3,
    kNotUsedJoint4,
    kNotUsedJoint5
};

std::array<JointIndex, 17> arm_joints = {
      JointIndex::kLeftShoulderPitch,  JointIndex::kLeftShoulderRoll,
      JointIndex::kLeftShoulderYaw,    JointIndex::kLeftElbow,
      JointIndex::kLeftWristRoll,       JointIndex::kLeftWristPitch,
      JointIndex::kLeftWristYaw,
      JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
      JointIndex::kRightShoulderYaw,   JointIndex::kRightElbow,
      JointIndex::kRightWristRoll,      JointIndex::kRightWristPitch,
      JointIndex::kRightWristYaw,
      JointIndex::kWaistYaw,
      JointIndex::kWaistRoll,
      JointIndex::kWaistPitch};
/*############################################INIT INTERFACE###########################################
#############################################################################################################
#############################################################################################################
#############################################################################################################*/

CallbackReturn G1Hardware::on_init(const hardware_interface::HardwareInfo & info)
{
  
  RCLCPP_DEBUG(rclcpp::get_logger("G1"), "configure");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, networkInterface);
  RCLCPP_INFO(rclcpp::get_logger("G1"), "G1 Enabled");
  joints_.resize(info_.joints.size(), Joint());

  for (uint i = 0; i < info_.joints.size(); i++) {
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
  }

  
  low_state_subscriber_.reset(
      new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(kTopicState));

  low_state_subscriber_->InitChannel([this](const void* msg) {
      auto s = static_cast<const unitree_hg::msg::dds_::LowState_*>(msg);
      std::lock_guard<std::mutex> lock(state_mutex_);
      std::memcpy(&state_msg_, s, sizeof(unitree_hg::msg::dds_::LowState_));
  }, 1);

    // Initialize command publisher
  low_cmd_publisher_.reset(
      new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(kTopicArmSDK));
  low_cmd_publisher_->InitChannel();
  return CallbackReturn::SUCCESS;
}

/*############################################CONFIGURE INTERFACE###########################################*/

CallbackReturn G1Hardware::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  
  RCLCPP_INFO(rclcpp::get_logger("G1"), "G1 Enabled");

  return CallbackReturn::SUCCESS;
}
/*############################################STATES INTERFACE###########################################*/

std::vector<hardware_interface::StateInterface> G1Hardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("G1"), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }

  return state_interfaces;
}
/*############################################COMMANDS INTERFACE###########################################*/

std::vector<hardware_interface::CommandInterface> G1Hardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("G1"), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }

  return command_interfaces;
}


/*############################################ACTIVATE INTERFACE###########################################*/

CallbackReturn G1Hardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  //START DDS

  const float activation_time = 2.0; // seconds
  const int steps = activation_time / 0.002; // 2ms control period

  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  commanded_positions_.resize(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i) {
    commanded_positions_[i] = joints_[i].state.position;
    joints_[i].command.position = joints_[i].state.position; // Avoid jumps
    
  }

  //reset_command();
  weight = 1.0;
  cmd_msg_.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight);
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}


/*############################################DEACTIVATE INTERFACE###########################################*/

CallbackReturn G1Hardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger("G1"), "stop");
  return CallbackReturn::SUCCESS;
}

/*##################################READ INTERFACE AND SEND TO STATE#########################################*/

return_type G1Hardware::read(const rclcpp::Time & /* time */,const rclcpp::Duration & /* period */)
{

  unitree_hg::msg::dds_::LowState_ local_copy;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::memcpy(&local_copy, &state_msg_, sizeof(local_copy));
  }

  std::cout << local_copy.motor_state().size() << std::endl;
  for (size_t i = 0; i < joints_.size(); ++i) {
    joints_[i].state.position = local_copy.motor_state().at(arm_joints.at(i)).q();
    joints_[i].state.velocity = local_copy.motor_state().at(arm_joints.at(i)).dq();
    joints_[i].state.effort = local_copy.motor_state().at(arm_joints.at(i)).tau_est();
    
  }


  return return_type::OK;
}

/*##################################WRITE INTERFACE AND SEND TO COMMAND######################################*/

return_type G1Hardware::write(const rclcpp::Time & /* time */,const rclcpp::Duration & /* period */)
{
  // Update command message with new joint commands

  float dq = 0.f;

  const double max_delta = max_joint_velocity_ * 0.1;


  for (size_t i = 0; i < joints_.size(); ++i) {
    const double target = joints_[i].command.position;
    const double error = target - commanded_positions_[i];
    commanded_positions_[i] += std::clamp(error, -max_delta, max_delta);
    size_t motor_idx = arm_joints[i];  // Same offset as in read()
    RCLCPP_INFO(rclcpp::get_logger("G1"), 
    "Joint %zu: Target=%.3f, Commanded=%.3f, Delta=%.4f, Actual=%.3f, VEL =%.3f",
    i,
    joints_[i].command.position,
    commanded_positions_[i],
    std::clamp(error, -max_delta, max_delta),
    joints_[i].state.position,
    dq);
    
    cmd_msg_.motor_cmd().at(arm_joints.at(i)).q(joints_[i].command.position);
    cmd_msg_.motor_cmd().at(arm_joints.at(i)).dq(dq);
    cmd_msg_.motor_cmd().at(arm_joints.at(i)).kp(kp_);
    cmd_msg_.motor_cmd().at(arm_joints.at(i)).kd(kd_);
    cmd_msg_.motor_cmd().at(arm_joints.at(i)).tau(tau_ff_);
    
    // Store previous command
    //'joints_[i].prev_command = joints_[i].command;
  }

  // Send command to robot
  low_cmd_publisher_->Write(cmd_msg_);

  return return_type::OK;

}

/*###############################################TODO: ENABLE TORQUE#########################################*/

return_type G1Hardware::enable_torque(const bool enabled)
{

  if (enabled && !torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {

    }
    reset_command();
    RCLCPP_INFO(rclcpp::get_logger("G1"), "Torque enabled");
  } else if (!enabled && torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {

    }
    RCLCPP_INFO(rclcpp::get_logger("G1"), "Torque disabled");
  }

  torque_enabled_ = enabled;
  return return_type::OK;
}

/*##############################################TODO: RESET COMMANDS#########################################*/
return_type G1Hardware::reset_command()
{
  for (uint i = 0; i < info_.joints.size(); i++) {
    joints_[i].command.position = joints_[i].state.position;
    joints_[i].command.velocity = 0.0;
    joints_[i].command.effort = 0.0;
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
  }

  return return_type::OK;
}


//TODO: remove from write function and put inside their respective functions


/*#####################################SET JOINT POSITION ON COMMAND#########################################*/

CallbackReturn G1Hardware::set_joint_positions()
{
  std::vector<int32_t> commands(info_.joints.size(), 0);

  for (uint i = 0; i < info_.joints.size(); i++) {
    //joints_[i].prev_command.position = joints_[i].command.position;
    //commands[i] = 0.0;
  }


  return CallbackReturn::SUCCESS;
}


//TODO:

/*##################################$$SET JOINT VELOCITY ON COMMAND#########################################*/

CallbackReturn G1Hardware::set_joint_velocities()
{
  std::vector<int32_t> commands(info_.joints.size(), 0);

  for (uint i = 0; i < info_.joints.size(); i++) {
    //joints_[i].prev_command.velocity = joints_[i].command.velocity;
    //commands[i] = 0.0;
  }

  return CallbackReturn::SUCCESS;
}

/*##########################TODO: IF HAVE ANY PARAMETERS TO BE SET ON G1#####################################
#############TODO: MAYBE SELECT SPORT OR NORMAL MODE, ARM OR ALL JOINTS MODE TO CONTROL######################
#############################################################################################################
#############################################################################################################*/

CallbackReturn G1Hardware::set_joint_params()
{
  for (uint i = 0; i < info_.joints.size(); ++i) {
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace g1_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(g1_hardware::G1Hardware, hardware_interface::SystemInterface)