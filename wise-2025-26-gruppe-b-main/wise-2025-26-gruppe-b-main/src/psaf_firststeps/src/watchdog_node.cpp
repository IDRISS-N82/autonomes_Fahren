#include "psaf_firststeps/watchdog_node.hpp"

#include <functional>

WatchdogNode::WatchdogNode()
: Node{"watchdog"}
{
  const auto qos = rclcpp::QoS{rclcpp::KeepLast(1)};
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "color/image_raw", qos,
    std::bind(&WatchdogNode::imageCallback, this, std::placeholders::_1));

  lane_sub_ = this->create_subscription<psaf_firststeps::msg::LaneMarkings>(
    "lane_detection/lane_markings", qos,
    std::bind(&WatchdogNode::laneCallback, this, std::placeholders::_1));

  traj_sub_ = this->create_subscription<psaf_firststeps::msg::Trajectory>(
    "trajectory/trajectory", qos,
    std::bind(&WatchdogNode::trajectoryCallback, this, std::placeholders::_1));

  error_pub_ = this->create_publisher<std_msgs::msg::String>("watchdog/error_message", qos);

  const double timeout_s = this->declare_parameter("timeout", 0.5);
  timeout_ = rclcpp::Duration::from_seconds(timeout_s);

  const auto now = this->now();
  last_image_stamp_ = now;
  last_lane_stamp_ = now;
  last_traj_stamp_ = now;

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&WatchdogNode::timerCheck, this));
}

void WatchdogNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  last_image_stamp_ = msg->header.stamp;
}

void WatchdogNode::laneCallback(const psaf_firststeps::msg::LaneMarkings::SharedPtr msg)
{
  last_lane_stamp_ = msg->stamp;
}

void WatchdogNode::trajectoryCallback(const psaf_firststeps::msg::Trajectory::SharedPtr msg)
{
  last_traj_stamp_ = msg->stamp;
}

void WatchdogNode::timerCheck()
{
  const auto now = this->now();
  if (now - last_image_stamp_ > timeout_) {
    reportError("Camera timeout");
  } else if (now - last_lane_stamp_ > timeout_) {
    reportError("Lane detection timeout");
  } else if (now - last_traj_stamp_ > timeout_) {
    reportError("Trajectory timeout");
  }
}

void WatchdogNode::reportError(const std::string & message)
{
  std_msgs::msg::String msg;
  msg.data = message;
  error_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WatchdogNode>());
  rclcpp::shutdown();
  return 0;
}
