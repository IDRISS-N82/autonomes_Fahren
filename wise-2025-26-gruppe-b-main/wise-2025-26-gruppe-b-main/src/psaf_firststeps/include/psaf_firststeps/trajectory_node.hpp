#ifndef PSAF_FIRSTSTEPS__TRAJECTORY_NODE_HPP_
#define PSAF_FIRSTSTEPS__TRAJECTORY_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "psaf_firststeps/msg/lane_markings.hpp"
#include "psaf_firststeps/msg/state.hpp"
#include "psaf_firststeps/msg/trajectory.hpp"
#include "psaf_firststeps/polynomial_utils.hpp"

class TrajectoryNode : public rclcpp::Node
{
public:
  TrajectoryNode();

private:
  void laneCallback(const psaf_firststeps::msg::LaneMarkings::SharedPtr msg);
  void stateCallback(const psaf_firststeps::msg::State::SharedPtr msg);
  void publishTrajectory(const psaf_firststeps::msg::LaneMarkings & lane);

  rclcpp::Subscription<psaf_firststeps::msg::LaneMarkings>::SharedPtr lane_sub_;
  rclcpp::Subscription<psaf_firststeps::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<psaf_firststeps::msg::Trajectory>::SharedPtr traj_pub_;

  bool active_{false};
  double x_min_;
  double x_max_;
  double max_lateral_;
  double sample_resolution_;
};

#endif  // PSAF_FIRSTSTEPS__TRAJECTORY_NODE_HPP_
