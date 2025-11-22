#include "psaf_firststeps/state_machine_node.hpp"

#include <functional>

StateMachineNode::StateMachineNode()
: Node{"state_machine"}
{
  const auto qos = rclcpp::QoS{rclcpp::KeepLast(1)};
  status_sub_ = this->create_subscription<psaf_firststeps::msg::StatusInfo>(
    "status/status_info", qos,
    std::bind(&StateMachineNode::statusCallback, this, std::placeholders::_1));

  watchdog_sub_ = this->create_subscription<std_msgs::msg::String>(
    "watchdog/error_message", qos,
    std::bind(&StateMachineNode::watchdogCallback, this, std::placeholders::_1));

  state_pub_ = this->create_publisher<psaf_firststeps::msg::State>("state_machine/state", qos);

  state_msg_.state = psaf_firststeps::msg::State::STARTBOX;
  publishState();
}

void StateMachineNode::statusCallback(const psaf_firststeps::msg::StatusInfo::SharedPtr msg)
{
  if (msg->code == psaf_firststeps::msg::StatusInfo::STARTBOX_OPEN &&
    state_msg_.state == psaf_firststeps::msg::State::STARTBOX)
  {
    state_msg_.state = psaf_firststeps::msg::State::DRIVING;
  } else if (msg->code == psaf_firststeps::msg::StatusInfo::WATCHDOG_TIMEOUT) {
    state_msg_.state = psaf_firststeps::msg::State::EMERGENCY_STOP;
  }
  publishState();
}

void StateMachineNode::watchdogCallback(const std_msgs::msg::String::SharedPtr msg)
{
  (void)msg;
  state_msg_.state = psaf_firststeps::msg::State::EMERGENCY_STOP;
  publishState();
}

void StateMachineNode::publishState()
{
  state_pub_->publish(state_msg_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateMachineNode>());
  rclcpp::shutdown();
  return 0;
}
