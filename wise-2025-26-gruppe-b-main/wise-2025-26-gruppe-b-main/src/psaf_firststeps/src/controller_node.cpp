#include "psaf_firststeps/controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <functional>

#include "geometry_msgs/msg/point.hpp"
#include "psaf_configuration/configuration.hpp"

ControllerNode::ControllerNode()
: Node{"controller"}
{
  wheel_base_ = this->declare_parameter("wheel_base", 0.26);
  const double max_steer_deg = this->declare_parameter("max_steering_deg", 30.0);
  max_steering_rad_ = max_steer_deg * M_PI / 180.0;
  curvature_lookahead_ = this->declare_parameter("curvature_lookahead", 0.4);
  k_d_ = this->declare_parameter("k_d", 1.0);
  k_psi_ = this->declare_parameter("k_psi", 0.8);
  k_speed_ = this->declare_parameter("k_speed", 0.5);
  target_speed_ = this->declare_parameter("target_speed", 0.5);
  max_steering_rate_ = this->declare_parameter("max_steering_rate", 0.5);
  max_lateral_error_ = this->declare_parameter("max_lateral_error", 1.5);
  control_rate_ = this->declare_parameter("control_rate", 30.0);

  const auto qos = rclcpp::QoS{rclcpp::KeepLast(1)};
  traj_sub_ = this->create_subscription<psaf_firststeps::msg::Trajectory>(
    "trajectory/trajectory", qos,
    std::bind(&ControllerNode::trajectoryCallback, this, std::placeholders::_1));

  speed_sub_ = this->create_subscription<std_msgs::msg::Int16>(
    GET_SPEED_TOPIC, qos,
    std::bind(&ControllerNode::speedCallback, this, std::placeholders::_1));

  state_sub_ = this->create_subscription<psaf_firststeps::msg::State>(
    "state_machine/state", qos,
    std::bind(&ControllerNode::stateCallback, this, std::placeholders::_1));

  steering_pub_ = this->create_publisher<std_msgs::msg::Int16>(SET_STEERING_TOPIC, qos);
  speed_forward_pub_ = this->create_publisher<std_msgs::msg::Int16>(SET_SPEED_FORWARD_TOPIC, qos);
  speed_backward_pub_ = this->create_publisher<std_msgs::msg::Int16>(SET_SPEED_BACKWARD_TOPIC, qos);

  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / control_rate_),
    std::bind(&ControllerNode::controlStep, this));
}

void ControllerNode::stateCallback(const psaf_firststeps::msg::State::SharedPtr msg)
{
  active_ = msg->state == psaf_firststeps::msg::State::DRIVING;
  if (!active_) {
    publishSpeed(0.0);
    publishSteering(0.0);
  }
}

void ControllerNode::trajectoryCallback(const psaf_firststeps::msg::Trajectory::SharedPtr msg)
{
  latest_traj_ = msg;
}

void ControllerNode::speedCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_speed_ = static_cast<double>(msg->data) / 100.0;
}

void ControllerNode::controlStep()
{
  if (!active_ || !latest_traj_) {
    return;
  }

  std::vector<geometry_msgs::msg::Point> usable_points;
  for (const auto & p : latest_traj_->points) {
    if (p.x >= 0.0 && isValidPoint(p, max_lateral_error_)) {
      usable_points.push_back(p);
    }
  }

  auto poly = fitQuadratic(usable_points);
  if (!poly.has_value()) {
    publishSpeed(target_speed_);
    publishSteering(0.0);
    return;
  }

  const double curvature = curvatureAt(*poly, curvature_lookahead_);
  const double phi_ff = std::atan(curvature * wheel_base_);
  const double d = poly->c;
  const double psi = std::atan(poly->b);
  const double phi_fb = k_d_ * d + k_psi_ * psi;
  double phi_cmd = phi_ff + phi_fb;

  phi_cmd = std::clamp(phi_cmd, -max_steering_rad_, max_steering_rad_);
  const double max_delta = max_steering_rate_ * 1.0 / control_rate_;
  const double delta = std::clamp(phi_cmd - previous_steering_, -max_delta, max_delta);
  phi_cmd = previous_steering_ + delta;
  previous_steering_ = phi_cmd;

  publishSteering(phi_cmd);

  const double speed_error = target_speed_ - current_speed_;
  publishSpeed(k_speed_ * speed_error);
}

void ControllerNode::publishSteering(double steering_rad)
{
  const double scale = 300.0 / max_steering_rad_;
  int16_t value = static_cast<int16_t>(std::lround(steering_rad * scale));
  value = std::clamp<int16_t>(value, -300, 300);
  std_msgs::msg::Int16 msg;
  msg.data = value;
  steering_pub_->publish(msg);
}

void ControllerNode::publishSpeed(double desired_speed)
{
  const int16_t cmd = static_cast<int16_t>(std::lround(desired_speed * 100.0));
  if (cmd >= 0) {
    std_msgs::msg::Int16 msg;
    msg.data = std::clamp<int16_t>(cmd, -500, 1000);
    speed_forward_pub_->publish(msg);
  } else {
    std_msgs::msg::Int16 msg;
    msg.data = static_cast<int16_t>(std::clamp(-cmd, static_cast<int16_t>(0), static_cast<int16_t>(500)));
    speed_backward_pub_->publish(msg);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
