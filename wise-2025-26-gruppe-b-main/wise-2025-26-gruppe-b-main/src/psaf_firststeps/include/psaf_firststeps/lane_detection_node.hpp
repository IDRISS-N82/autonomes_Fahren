#ifndef PSAF_FIRSTSTEPS__LANE_DETECTION_NODE_HPP_
#define PSAF_FIRSTSTEPS__LANE_DETECTION_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "psaf_firststeps/msg/lane_markings.hpp"
#include "psaf_firststeps/msg/state.hpp"

class LaneDetectionNode : public rclcpp::Node
{
public:
  LaneDetectionNode();

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void stateCallback(const psaf_firststeps::msg::State::SharedPtr msg);
  void publishLane(const rclcpp::Time & stamp);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<psaf_firststeps::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<psaf_firststeps::msg::LaneMarkings>::SharedPtr lane_pub_;

  psaf_firststeps::msg::LaneMarkings last_lane_;
  bool active_{false};
  double lookahead_distance_;
  size_t lane_points_;
};

#endif  // PSAF_FIRSTSTEPS__LANE_DETECTION_NODE_HPP_
