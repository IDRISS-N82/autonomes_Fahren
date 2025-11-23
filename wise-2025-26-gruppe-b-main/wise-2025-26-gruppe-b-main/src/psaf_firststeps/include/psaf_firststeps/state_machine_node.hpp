#ifndef PSAF_FIRSTSTEPS__STATE_MACHINE_NODE_HPP_
#define PSAF_FIRSTSTEPS__STATE_MACHINE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <array>

#include "psaf_firststeps/msg/state.hpp"
#include "psaf_firststeps/msg/status_info.hpp"
#include "psaf_ucbridge_msgs/msg/pbs.hpp"
#include "psaf_configuration/configuration.hpp"
#include "std_msgs/msg/string.hpp"

class StateMachineNode : public rclcpp::Node
{
public:
  StateMachineNode();

private:
  void statusCallback(const psaf_firststeps::msg::StatusInfo::SharedPtr msg);
  void watchdogCallback(const std_msgs::msg::String::SharedPtr msg);
  void buttonCallback(const psaf_ucbridge_msgs::msg::Pbs::SharedPtr msg);
  void publishState();

  void toggleDrivingState();

  rclcpp::Subscription<psaf_firststeps::msg::StatusInfo>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr watchdog_sub_;
  rclcpp::Subscription<psaf_ucbridge_msgs::msg::Pbs>::SharedPtr button_sub_;
  rclcpp::Publisher<psaf_firststeps::msg::State>::SharedPtr state_pub_;

  std::array<uint16_t, 3> prev_button_values_ {0xFFFF, 0xFFFF, 0xFFFF};
  psaf_firststeps::msg::State state_msg_;
};

#endif  // PSAF_FIRSTSTEPS__STATE_MACHINE_NODE_HPP_
