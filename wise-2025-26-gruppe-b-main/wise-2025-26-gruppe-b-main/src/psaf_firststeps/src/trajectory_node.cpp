#include "psaf_firststeps/trajectory_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

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

  state_sub_ = this->create_subscription<psaf_firststeps::msg::State>(
    "state_machine/state", qos,
    std::bind(&TrajectoryNode::stateCallback, this, std::placeholders::_1));

  traj_pub_ = this->create_publisher<psaf_firststeps::msg::Trajectory>(
    "trajectory/trajectory", qos);
}

void TrajectoryNode::stateCallback(const psaf_firststeps::msg::State::SharedPtr msg)
{
  active_ = msg->state == psaf_firststeps::msg::State::DRIVING;
}

void TrajectoryNode::laneCallback(const psaf_firststeps::msg::LaneMarkings::SharedPtr msg)
{
  if (!active_) {
    return;
  }
  publishTrajectory(*msg);
}

void TrajectoryNode::publishTrajectory(const psaf_firststeps::msg::LaneMarkings & lane)
{
  std::vector<geometry_msgs::msg::Point> usable_points;

  auto add_if_valid = [this, &usable_points](const geometry_msgs::msg::Point & point) {
    if (point.x >= x_min_ && point.x <= x_max_ && isValidPoint(point, max_lateral_)) {
      usable_points.push_back(point);
    }
  };

  if (lane.center_valid && !lane.center_lane.empty()) {
    for (const auto & p : lane.center_lane) {
      add_if_valid(p);
    }
  } else if (!lane.left_lane.empty() && !lane.right_lane.empty()) {
    const size_t n = std::min(lane.left_lane.size(), lane.right_lane.size());
    for (size_t i = 0; i < n; ++i) {
      geometry_msgs::msg::Point p;
      p.x = lane.left_lane[i].x;
      p.y = 0.5 * (lane.left_lane[i].y + lane.right_lane[i].y);
      add_if_valid(p);
    }
  } else if (!lane.right_lane.empty()) {
    for (const auto & p : lane.right_lane) {
      add_if_valid(p);
    }
  } else if (!lane.left_lane.empty()) {
    for (const auto & p : lane.left_lane) {
      add_if_valid(p);
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
    // Reparameterize approximately by arc length s between x_min_ and x_max_.
    // Arc length along y(x) is approximated with small Δx steps via
    //   ds = sqrt(1 + (dy/dx)^2) * Δx, where dy/dx = 2 a x + b for a quadratic.
    // We build a lookup table (s_i, x_i, y_i) and later sample equidistantly in s.
    struct ArcSample
    {
      double s;
      double x;
      double y;
    };

    std::vector<ArcSample> arc_samples;
    const double dx = std::max(0.01, sample_resolution_ * 0.25);
    double current_s = 0.0;
    double x = x_min_;

    while (x <= x_max_ + 1e-6) {
      const double y = poly->a * x * x + poly->b * x + poly->c;
      arc_samples.push_back({current_s, x, y});

      // Advance arc length using the local derivative.
      const double derivative = 2.0 * poly->a * x + poly->b;
      const double ds = std::sqrt(1.0 + derivative * derivative) * dx;
      current_s += ds;
      x += dx;
    }

    if (arc_samples.size() < 2 || current_s < std::numeric_limits<double>::epsilon()) {
      // Degenerate case: fall back to original x-based sampling.
      for (double x_fallback = x_min_; x_fallback <= x_max_ + 1e-6; x_fallback += sample_resolution_) {
        geometry_msgs::msg::Point p;
        p.x = x_fallback;
        p.y = poly->a * x_fallback * x_fallback + poly->b * x_fallback + poly->c;
        traj_msg.points.push_back(p);
      }
    } else {
      const double total_s = arc_samples.back().s;
      const size_t num_points = std::max<size_t>(2, static_cast<size_t>(std::floor(total_s / sample_resolution_)) + 1);

      // Sample equidistantly in s across the stored arc length range.
      for (size_t i = 0; i < num_points; ++i) {
        const double target_s = (i == num_points - 1) ? total_s : static_cast<double>(i) * total_s /
          static_cast<double>(num_points - 1);

        auto upper = std::lower_bound(
          arc_samples.begin(), arc_samples.end(), target_s,
          [](const ArcSample & sample, double value) {return sample.s < value;});

        if (upper == arc_samples.begin()) {
          geometry_msgs::msg::Point p;
          p.x = arc_samples.front().x;
          p.y = arc_samples.front().y;
          p.z = 0.0;
          traj_msg.points.push_back(p);
          continue;
        }
        if (upper == arc_samples.end()) {
          geometry_msgs::msg::Point p;
          p.x = arc_samples.back().x;
          p.y = arc_samples.back().y;
          p.z = 0.0;
          traj_msg.points.push_back(p);
          continue;
        }

        const ArcSample & prev = *(upper - 1);
        const ArcSample & next = *upper;
        const double span = next.s - prev.s;
        const double ratio = span > std::numeric_limits<double>::epsilon() ? (target_s - prev.s) / span : 0.0;

        geometry_msgs::msg::Point p;
        p.x = prev.x + ratio * (next.x - prev.x);
        p.y = prev.y + ratio * (next.y - prev.y);
        p.z = 0.0;
        traj_msg.points.push_back(p);
      }
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
