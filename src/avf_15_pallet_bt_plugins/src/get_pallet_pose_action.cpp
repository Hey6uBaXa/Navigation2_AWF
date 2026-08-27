#include "avf_15_pallet_bt_plugins/get_pallet_pose_action.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace avf_15_pallet_bt_plugins
{

GetPalletPoseAction::GetPalletPoseAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("get_pallet_pose_action_" + name);

  std::string topic = "/pallet_detector/dock_pose";
  getInput("pose_topic", topic);

  sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    topic, rclcpp::QoS(10),
    std::bind(&GetPalletPoseAction::poseCallback, this, std::placeholders::_1));
}

void GetPalletPoseAction::poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_pose_ = *msg;
  last_stamp_ = node_->now();
  have_pose_ = true;
}

BT::NodeStatus GetPalletPoseAction::tick()
{
  rclcpp::spin_some(node_);

  double max_age = 1.0;
  getInput("max_age", max_age);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_pose_) {
    return BT::NodeStatus::FAILURE;
  }
  double age = (node_->now() - last_stamp_).seconds();
  if (age > max_age) {
    RCLCPP_WARN(node_->get_logger(), "GetPalletPoseAction: pallet pose stale (%.2fs)", age);
    return BT::NodeStatus::FAILURE;
  }

  setOutput("pose", last_pose_);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace avf_15_pallet_bt_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<avf_15_pallet_bt_plugins::GetPalletPoseAction>("GetPalletPose");
}
