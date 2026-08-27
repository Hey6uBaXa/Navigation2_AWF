#ifndef AVF_15_PALLET_BT_PLUGINS__GET_PALLET_POSE_ACTION_HPP_
#define AVF_15_PALLET_BT_PLUGINS__GET_PALLET_POSE_ACTION_HPP_

#include <string>
#include <mutex>

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

namespace avf_15_pallet_bt_plugins
{

/**
 * SyncActionNode: reads the last message from pallet_detector_node
 * (dock_pose + found flag) and writes it to the "pose" output port.
 * Returns SUCCESS if a pallet was found recently (within max_age),
 * FAILURE otherwise, so it can be used inside a RetryUntilSuccessful /
 * fallback in the tree while the robot approaches the staging pose.
 *
 * Ports:
 *   pose_topic  (string) default "/pallet_detector/dock_pose"
 *   max_age     (double) seconds, default 1.0
 *   pose        (output PoseStamped)
 */
class GetPalletPoseAction : public BT::SyncActionNode
{
public:
  GetPalletPoseAction(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("pose_topic", "/pallet_detector/dock_pose", "topic"),
      BT::InputPort<double>("max_age", 1.0, "seconds"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose"),
    };
  }

  BT::NodeStatus tick() override;

private:
  void poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped last_pose_;
  rclcpp::Time last_stamp_;
  bool have_pose_{false};
};

}  // namespace avf_15_pallet_bt_plugins

#endif  // AVF_15_PALLET_BT_PLUGINS__GET_PALLET_POSE_ACTION_HPP_
