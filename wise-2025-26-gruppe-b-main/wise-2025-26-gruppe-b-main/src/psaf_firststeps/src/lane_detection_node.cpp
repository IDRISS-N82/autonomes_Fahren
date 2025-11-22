#include "psaf_firststeps/lane_detection_node.hpp"

#include <algorithm>
#include <functional>

#include "geometry_msgs/msg/point.hpp"

LaneDetectionNode::LaneDetectionNode()
: Node{"lane_detection"}
{
  lookahead_distance_ = this->declare_parameter("lookahead_distance", 1.0);
  lane_points_ = static_cast<size_t>(this->declare_parameter("lane_points", 10));

  const auto qos = rclcpp::QoS{rclcpp::KeepLast(1)};
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "color/image_raw", qos,
    std::bind(&LaneDetectionNode::imageCallback, this, std::placeholders::_1));

  lane_pub_ = this->create_publisher<psaf_firststeps::msg::LaneMarkings>(
    "lane_detection/lane_markings", qos);

  last_lane_.center_valid = false;
}

void LaneDetectionNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  publishLane(msg->header.stamp);
}

void LaneDetectionNode::publishLane(const rclcpp::Time & stamp)
{
  psaf_firststeps::msg::LaneMarkings lane_msg;
  lane_msg.center_lane.clear();
  lane_msg.center_lane.reserve(lane_points_);
  const double step = lookahead_distance_ / static_cast<double>(std::max<size_t>(1, lane_points_));

  for (size_t i = 1; i <= lane_points_; ++i) {
    geometry_msgs::msg::Point p;
    p.x = static_cast<double>(i) * step;
    p.y = 0.0;
    lane_msg.center_lane.push_back(p);
  }

  lane_msg.center_valid = true;
  lane_msg.stamp = stamp;
  last_lane_ = lane_msg;
  lane_pub_->publish(lane_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
