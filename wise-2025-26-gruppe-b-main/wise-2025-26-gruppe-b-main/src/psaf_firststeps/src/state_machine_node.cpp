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

  button_sub_ = this->create_subscription<psaf_ucbridge_msgs::msg::Pbs>(
    GET_PUSHBUTTONS_TOPIC, qos,
    std::bind(&StateMachineNode::buttonCallback, this, std::placeholders::_1));

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

void StateMachineNode::buttonCallback(const psaf_ucbridge_msgs::msg::Pbs::SharedPtr msg)
{
  const uint16_t button_a = msg->pba;

  // On the first message just initialise the state without triggering.
  if (prev_button_values_[0] == 0xFFFF) {
    prev_button_values_[0] = button_a;
    return;
  }

  // Detect a button press on release (ucbridge counts presses; even values mean released).
  if (button_a != prev_button_values_[0] && (button_a % 2 == 0)) {
    prev_button_values_[0] = button_a;
    toggleDrivingState();
  }
}

void StateMachineNode::publishState()
{
  state_pub_->publish(state_msg_);
}

void StateMachineNode::toggleDrivingState()
{
  if (state_msg_.state == psaf_firststeps::msg::State::DRIVING) {
    state_msg_.state = psaf_firststeps::msg::State::EMERGENCY_STOP;
    RCLCPP_INFO(this->get_logger(), "Stopping autonomous driving via push button");
  } else {
    state_msg_.state = psaf_firststeps::msg::State::DRIVING;
    RCLCPP_INFO(this->get_logger(), "Starting autonomous driving via push button");
  }

  publishState();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateMachineNode>());
  rclcpp::shutdown();
  return 0;
}
