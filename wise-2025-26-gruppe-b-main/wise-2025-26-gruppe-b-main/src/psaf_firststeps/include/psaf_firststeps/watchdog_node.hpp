#ifndef PSAF_FIRSTSTEPS__WATCHDOG_NODE_HPP_
#define PSAF_FIRSTSTEPS__WATCHDOG_NODE_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "psaf_firststeps/msg/lane_markings.hpp"
#include "psaf_firststeps/msg/trajectory.hpp"
#include "std_msgs/msg/string.hpp"

class WatchdogNode : public rclcpp::Node
{
public:
  WatchdogNode();

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void laneCallback(const psaf_firststeps::msg::LaneMarkings::SharedPtr msg);
  void trajectoryCallback(const psaf_firststeps::msg::Trajectory::SharedPtr msg);
  void timerCheck();
  void reportError(const std::string & message);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<psaf_firststeps::msg::LaneMarkings>::SharedPtr lane_sub_;
  rclcpp::Subscription<psaf_firststeps::msg::Trajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_image_stamp_;
  rclcpp::Time last_lane_stamp_;
  rclcpp::Time last_traj_stamp_;
  rclcpp::Duration timeout_;
};

#endif  // PSAF_FIRSTSTEPS__WATCHDOG_NODE_HPP_
