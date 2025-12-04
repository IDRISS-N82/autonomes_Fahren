#ifndef PSAF_FIRSTSTEPS__LANE_DETECTION_NODE_HPP_
#define PSAF_FIRSTSTEPS__LANE_DETECTION_NODE_HPP_

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/point.hpp"
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
  void publishLane(const rclcpp::Time & stamp, const std::vector<geometry_msgs::msg::Point> & right_lane);

  static double computeOrientation(const cv::Mat & points);
  static cv::Mat extract10EdgePoints(const cv::Mat & points);
  static cv::Mat getLaneContour(const cv::Mat & labels, int component_id);
  std::vector<geometry_msgs::msg::Point> detectLane(const cv::Mat & image);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<psaf_firststeps::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<psaf_firststeps::msg::LaneMarkings>::SharedPtr lane_pub_;

  psaf_firststeps::msg::LaneMarkings last_lane_;
  bool active_{false};
  bool save_debug_images_{false};
};

#endif  // PSAF_FIRSTSTEPS__LANE_DETECTION_NODE_HPP_
