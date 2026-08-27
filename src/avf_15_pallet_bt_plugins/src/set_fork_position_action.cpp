#include "avf_15_pallet_bt_plugins/set_fork_position_action.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace avf_15_pallet_bt_plugins
{

SetForkPositionAction::SetForkPositionAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("set_fork_position_action_" + name);

  cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/fork_controller/commands", rclcpp::QoS(10));

  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(10),
    std::bind(&SetForkPositionAction::jointStateCallback, this, std::placeholders::_1));
}

void SetForkPositionAction::jointStateCallback(sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "J_main_villas") {
      current_main_villas_ = msg->position[i];
      have_joint_state_ = true;
    } else if (msg->name[i] == "J_second_villas") {
      current_second_villas_ = msg->position[i];
      have_joint_state_ = true;
    }
  }
}

BT::NodeStatus SetForkPositionAction::onStart()
{
  if (!getInput("main_villas_pos", target_main_villas_) ||
      !getInput("second_villas_pos", target_second_villas_))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "SetForkPositionAction: missing main_villas_pos / second_villas_pos input");
    return BT::NodeStatus::FAILURE;
  }
  getInput("tolerance", tolerance_);
  getInput("timeout", timeout_s_);

  std_msgs::msg::Float64MultiArray cmd;
  // Order MUST match fork_controller.joints in controllers.yaml:
  // [J_main_villas, J_second_villas]
  cmd.data = {target_main_villas_, target_second_villas_};
  cmd_pub_->publish(cmd);

  start_time_ = node_->now();
  RCLCPP_INFO(node_->get_logger(),
    "SetForkPositionAction: commanding main=%.3f second=%.3f",
    target_main_villas_, target_second_villas_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetForkPositionAction::onRunning()
{
  rclcpp::spin_some(node_);

  bool have_state;
  double main_err, second_err;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    have_state = have_joint_state_;
    main_err = std::abs(current_main_villas_ - target_main_villas_);
    second_err = std::abs(current_second_villas_ - target_second_villas_);
  }

  if (have_state && main_err < tolerance_ && second_err < tolerance_) {
    RCLCPP_INFO(node_->get_logger(), "SetForkPositionAction: target reached");
    return BT::NodeStatus::SUCCESS;
  }

  double elapsed = (node_->now() - start_time_).seconds();
  if (elapsed > timeout_s_) {
    RCLCPP_WARN(node_->get_logger(),
      "SetForkPositionAction: timeout after %.1fs (main_err=%.3f second_err=%.3f)",
      elapsed, main_err, second_err);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void SetForkPositionAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "SetForkPositionAction: halted");
}

}  // namespace avf_15_pallet_bt_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<avf_15_pallet_bt_plugins::SetForkPositionAction>("SetForkPosition");
}
