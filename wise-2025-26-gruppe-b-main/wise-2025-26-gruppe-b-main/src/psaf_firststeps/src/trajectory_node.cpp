#include "psaf_firststeps/trajectory_node.hpp"

#include <algorithm>
#include <functional>

#include "geometry_msgs/msg/point.hpp"

TrajectoryNode::TrajectoryNode()
: Node{"trajectory"}
{
  x_min_ = this->declare_parameter("x_min", 0.05);
  x_max_ = this->declare_parameter("x_max", 1.0);
  max_lateral_ = this->declare_parameter("max_lateral", 1.0);
  sample_resolution_ = this->declare_parameter("sample_resolution", 0.05);

  const auto qos = rclcpp::QoS{rclcpp::KeepLast(1)};
  lane_sub_ = this->create_subscription<psaf_firststeps::msg::LaneMarkings>(
    "lane_detection/lane_markings", qos,
    std::bind(&TrajectoryNode::laneCallback, this, std::placeholders::_1));

  traj_pub_ = this->create_publisher<psaf_firststeps::msg::Trajectory>(
    "trajectory/trajectory", qos);
}

void TrajectoryNode::laneCallback(const psaf_firststeps::msg::LaneMarkings::SharedPtr msg)
{
  publishTrajectory(*msg);
}

void TrajectoryNode::publishTrajectory(const psaf_firststeps::msg::LaneMarkings & lane)
{
  std::vector<geometry_msgs::msg::Point> usable_points;
  if (lane.center_valid) {
    for (const auto & p : lane.center_lane) {
      if (p.x >= x_min_ && p.x <= x_max_ && isValidPoint(p, max_lateral_)) {
        usable_points.push_back(p);
      }
    }
  } else {
    const size_t n = std::min(lane.left_lane.size(), lane.right_lane.size());
    for (size_t i = 0; i < n; ++i) {
      geometry_msgs::msg::Point p;
      p.x = lane.left_lane[i].x;
      p.y = 0.5 * (lane.left_lane[i].y + lane.right_lane[i].y);
      if (p.x >= x_min_ && p.x <= x_max_ && isValidPoint(p, max_lateral_)) {
        usable_points.push_back(p);
      }
    }
  }

  if (usable_points.size() < 2) {
    usable_points.clear();
    for (double x = x_min_; x <= x_max_; x += sample_resolution_) {
      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = 0.0;
      usable_points.push_back(p);
    }
  }

  std::optional<QuadraticPolynomial> poly = fitQuadratic(usable_points);
  psaf_firststeps::msg::Trajectory traj_msg;
  traj_msg.stamp = lane.stamp;

  if (poly.has_value()) {
    for (double x = x_min_; x <= x_max_ + 1e-6; x += sample_resolution_) {
      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = poly->a * x * x + poly->b * x + poly->c;
      traj_msg.points.push_back(p);
    }
  } else {
    traj_msg.points = usable_points;
  }

  traj_pub_->publish(traj_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}
