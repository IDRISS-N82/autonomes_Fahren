#ifndef PSAF_FIRSTSTEPS__CONTROLLER_NODE_HPP_
#define PSAF_FIRSTSTEPS__CONTROLLER_NODE_HPP_

#include <optional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "psaf_configuration/configuration.hpp"
#include "psaf_firststeps/msg/trajectory.hpp"
#include "psaf_firststeps/polynomial_utils.hpp"

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode();

private:
  void trajectoryCallback(const psaf_firststeps::msg::Trajectory::SharedPtr msg);
  void speedCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void controlStep();
  void publishSteering(double steering_rad);
  void publishSpeed(double desired_speed);

  rclcpp::Subscription<psaf_firststeps::msg::Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr speed_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr speed_forward_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr speed_backward_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  psaf_firststeps::msg::Trajectory::SharedPtr latest_traj_;
  double current_speed_{0.0};
  double wheel_base_;
  double max_steering_rad_;
  double curvature_lookahead_;
  double k_d_;
  double k_psi_;
  double k_speed_;
  double target_speed_;
  double max_steering_rate_;
  double control_rate_;
  double previous_steering_{0.0};
  double max_lateral_error_;
};

#endif  // PSAF_FIRSTSTEPS__CONTROLLER_NODE_HPP_
