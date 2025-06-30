#include "unitree_controller/unitree_controller.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <cmath>

namespace unitree_controller
{

UnitreeController::UnitreeController()
: UnitreeControllerInterface(), 
  joint_names_({}), 
  sensor_names_({}),
  control_rate_(0), 
  control_period_(0),
  control_mode_(ControlMode::ZeroTorque),
  zero_torque_controller_(PDController::ZeroTorqueController()), 
  standing_up_controller_(PDController::StandingUpController()), 
  idling_controller_(PDController::ZeroTorqueController()),  // Use zero torque for idling
  sitting_down_controller_(PDController::SittingDownController())
{
  // Initialize the realtime buffer with ZeroTorque mode
  control_mode_rt_buffer_.writeFromNonRT(ControlMode::ZeroTorque);
  
  // Initialize cmd_vel buffer with zero velocity
  geometry_msgs::msg::Twist zero_twist;
  cmd_vel_buffer_.writeFromNonRT(zero_twist);
}

void UnitreeController::declare_parameters() 
{
  // interfaces
  auto_declare<std::vector<std::string>>("joints", joint_names_);
  auto_declare<std::vector<std::string>>("sensors", sensor_names_);
  // node parameters
  auto_declare<int>("control_rate", 400);
}

controller_interface::CallbackReturn UnitreeController::read_parameters() 
{
  // interfaces
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  sensor_names_ = get_node()->get_parameter("sensors").as_string_array();
  // node parameters
  control_rate_  = static_cast<double>(get_node()->get_parameter("control_rate").get_value<int>());

  if (joint_names_.size() != 12)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter has wrong size");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (sensor_names_.size() != 5)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'sensors' parameter has wrong size");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Controller will be updated at %.2f Hz.", control_rate_);
  if (control_rate_ > 0.0)
  {
    control_period_ = 1.0 / control_rate_; // seconds
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'control_rate_' must be positive, got %lf.", control_rate_);
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnitreeController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  auto ret = UnitreeControllerInterface::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Create service after the node is initialized
  set_control_mode_srv_ = get_node()->create_service<unitree_msgs::srv::SetControlMode>(
      "set_control_mode", std::bind(&UnitreeController::setControlModeCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Create cmd_vel subscriber
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "key_vel", 10, std::bind(&UnitreeController::cmdVelCallback, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<std::string> UnitreeController::get_joint_names() const {
  return joint_names_;  
}

std::vector<std::string> UnitreeController::get_sensor_names() const {
  return sensor_names_;  
}

controller_interface::return_type UnitreeController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period,
    const UnitreeStates & states, UnitreeCommands & commands) 
{
  control_mode_ = *control_mode_rt_buffer_.readFromRT();

  switch (control_mode_)
  {
    case ControlMode::ZeroTorque: {
      commands.qJ_cmd   = zero_torque_controller_.qJ_cmd();
      commands.dqJ_cmd  = zero_torque_controller_.dqJ_cmd();
      commands.tauJ_cmd = zero_torque_controller_.tauJ_cmd();
      commands.Kp_cmd   = zero_torque_controller_.Kp_cmd();
      commands.Kd_cmd   = zero_torque_controller_.Kd_cmd();
      return controller_interface::return_type::OK;
      break;
    }
    case ControlMode::StandingUp: {
      commands.qJ_cmd   = standing_up_controller_.qJ_cmd();
      commands.dqJ_cmd  = standing_up_controller_.dqJ_cmd();
      commands.tauJ_cmd = standing_up_controller_.tauJ_cmd();
      commands.Kp_cmd   = standing_up_controller_.Kp_cmd();
      commands.Kd_cmd   = standing_up_controller_.Kd_cmd();
      return controller_interface::return_type::OK;
      break;
    }
    case ControlMode::Idling: {
      commands.qJ_cmd   = idling_controller_.qJ_cmd();
      commands.dqJ_cmd  = idling_controller_.dqJ_cmd();
      commands.tauJ_cmd = idling_controller_.tauJ_cmd();
      commands.Kp_cmd   = idling_controller_.Kp_cmd();
      commands.Kd_cmd   = idling_controller_.Kd_cmd();
      return controller_interface::return_type::OK;
      break;
    }
    case ControlMode::SittingDown: {
      commands.qJ_cmd   = sitting_down_controller_.qJ_cmd();
      commands.dqJ_cmd  = sitting_down_controller_.dqJ_cmd();
      commands.tauJ_cmd = sitting_down_controller_.tauJ_cmd();
      commands.Kp_cmd   = sitting_down_controller_.Kp_cmd();
      commands.Kd_cmd   = sitting_down_controller_.Kd_cmd();
      return controller_interface::return_type::OK;
      break;
    }
    case ControlMode::Control: {
      // Read current cmd_vel
      auto cmd_vel = *cmd_vel_buffer_.readFromRT();
      
      // Simple locomotion: convert Twist to joint commands
      // This is a basic implementation - you can make it more sophisticated
      double linear_x = cmd_vel.linear.x;   // Forward/backward
      double angular_z = cmd_vel.angular.z; // Turn left/right
      
      // Base standing position with gains
      Vector12d base_positions;
      base_positions << 
        0.0, 0.9, -1.8,  // FL: hip, thigh, calf
        0.0, 0.9, -1.8,  // FR: hip, thigh, calf
        0.0, 0.9, -1.8,  // RL: hip, thigh, calf
        0.0, 0.9, -1.8;  // RR: hip, thigh, calf
      
      // Apply simple walking motion
      double walking_amplitude = 0.3 * linear_x;  // Scale with forward velocity
      double turning_amplitude = 0.2 * angular_z; // Scale with turn velocity
      
      // Simple leg coordination for walking
      double phase = fmod(time.seconds() * 2.0, 2.0 * M_PI); // 1 Hz walking frequency
      
      // Basic trot gait: FL+RR together, FR+RL together
      double fl_rr_phase = sin(phase) * walking_amplitude;
      double fr_rl_phase = sin(phase + M_PI) * walking_amplitude;
      
      commands.qJ_cmd = base_positions;
      
      // Apply walking motion to thigh joints (index 1, 4, 7, 10)
      commands.qJ_cmd[1] += fl_rr_phase;  // FL thigh
      commands.qJ_cmd[4] += fr_rl_phase;  // FR thigh  
      commands.qJ_cmd[7] += fr_rl_phase;  // RL thigh
      commands.qJ_cmd[10] += fl_rr_phase; // RR thigh
      
      // Apply turning motion to hip joints (index 0, 3, 6, 9)
      commands.qJ_cmd[0] += turning_amplitude;   // FL hip
      commands.qJ_cmd[3] -= turning_amplitude;   // FR hip
      commands.qJ_cmd[6] += turning_amplitude;   // RL hip
      commands.qJ_cmd[9] -= turning_amplitude;   // RR hip
      
      // Set velocity and torque commands
      commands.dqJ_cmd = Vector12d::Zero();
      commands.tauJ_cmd = Vector12d::Zero();
      
      // Set gains for position control
      commands.Kp_cmd = Vector12d::Constant(50.0);  // Position gain
      commands.Kd_cmd = Vector12d::Constant(2.0);   // Velocity gain
      
      return controller_interface::return_type::OK;
      break;
    }
    default: {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid ControlMode in update: %d", static_cast<int>(control_mode_));
      return controller_interface::return_type::ERROR;
      break;
    }
  }

  return controller_interface::return_type::ERROR;
}

void UnitreeController::setControlModeCallback(const std::shared_ptr<unitree_msgs::srv::SetControlMode::Request> request,
                                               std::shared_ptr<unitree_msgs::srv::SetControlMode::Response> response) {
  try {
    response->current_control_mode = FromControlModeToString(*control_mode_rt_buffer_.readFromNonRT());
    ControlMode new_mode = FromStringToControlMode(request->control_mode);
    control_mode_rt_buffer_.writeFromNonRT(new_mode);
    response->accept = true;
    RCLCPP_INFO(get_node()->get_logger(), "Control mode changed to: %s", request->control_mode.c_str());
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to set control mode: %s", e.what());
    response->current_control_mode = FromControlModeToString(*control_mode_rt_buffer_.readFromNonRT());
    response->accept = false;
  }
}

void UnitreeController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd_vel_buffer_.writeFromNonRT(*msg);
}

} // namespace unitree_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  unitree_controller::UnitreeController, 
  controller_interface::ControllerInterface)