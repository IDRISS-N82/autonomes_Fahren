#ifndef PSAF_FIRSTSTEPS__STATE_MACHINE_NODE_HPP_
#define PSAF_FIRSTSTEPS__STATE_MACHINE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "psaf_firststeps/msg/state.hpp"
#include "psaf_firststeps/msg/status_info.hpp"
#include "std_msgs/msg/string.hpp"

class StateMachineNode : public rclcpp::Node
{
public:
  StateMachineNode();

private:
  void statusCallback(const psaf_firststeps::msg::StatusInfo::SharedPtr msg);
  void watchdogCallback(const std_msgs::msg::String::SharedPtr msg);
  void publishState();

  rclcpp::Subscription<psaf_firststeps::msg::StatusInfo>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr watchdog_sub_;
  rclcpp::Publisher<psaf_firststeps::msg::State>::SharedPtr state_pub_;

  psaf_firststeps::msg::State state_msg_;
};

#endif  // PSAF_FIRSTSTEPS__STATE_MACHINE_NODE_HPP_
