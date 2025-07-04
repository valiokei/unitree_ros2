#ifndef UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_
#define UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_

#include "unitree_controller/unitree_controller_interface.hpp"
#include "unitree_controller/types.hpp"
#include "unitree_controller/visibility_control.h"
#include "unitree_controller/pd_controller.hpp"

#include "unitree_msgs/srv/set_control_mode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "realtime_tools/realtime_buffer.hpp"


namespace unitree_controller
{

using namespace std::chrono_literals;  // NOLINT

class UnitreeController : public UnitreeControllerInterface
{
public:
  UNITREE_CONTROLLER_PUBLIC 
  UnitreeController();

private:
  void declare_parameters() override;

  controller_interface::CallbackReturn read_parameters() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<std::string> get_joint_names() const override;

  std::vector<std::string> get_sensor_names() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period,
    const UnitreeStates & states, UnitreeCommands & commands) override;

  // interfaces
  std::vector<std::string> joint_names_, sensor_names_;

  // node parameters
  double control_rate_, control_period_;

  // Runtime controllers 
  ControlMode control_mode_;
  PDController zero_torque_controller_, standing_up_controller_, idling_controller_, sitting_down_controller_;

  // Services
  rclcpp::Service<unitree_msgs::srv::SetControlMode>::SharedPtr set_control_mode_srv_;
  realtime_tools::RealtimeBuffer<ControlMode> control_mode_rt_buffer_;
  void setControlModeCallback(const std::shared_ptr<unitree_msgs::srv::SetControlMode::Request> request,
                              std::shared_ptr<unitree_msgs::srv::SetControlMode::Response> response);

  // Twist command subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> cmd_vel_buffer_;
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
};

} // namespace unitree_controller

#endif // UNITREE_CONTROLLER__UNITREE_CONTROLLER_HPP_