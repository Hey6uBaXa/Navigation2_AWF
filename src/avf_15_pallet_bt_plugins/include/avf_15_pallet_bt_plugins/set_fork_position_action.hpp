#ifndef AVF_15_PALLET_BT_PLUGINS__SET_FORK_POSITION_ACTION_HPP_
#define AVF_15_PALLET_BT_PLUGINS__SET_FORK_POSITION_ACTION_HPP_

#include <string>
#include <vector>
#include <mutex>

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace avf_15_pallet_bt_plugins
{

/**
 * BT.CPP StatefulActionNode that commands the fork_controller
 * (position_controllers/JointGroupPositionController) via its
 * "<controller>/commands" topic and returns SUCCESS once both
 * joints (J_main_villas, J_second_villas) reach the requested
 * position within tolerance, or FAILURE on timeout.
 *
 * Ports:
 *   main_villas_pos   (double)  target position for J_main_villas
 *   second_villas_pos (double)  target position for J_second_villas
 *   tolerance         (double)  position tolerance, default 0.01
 *   timeout           (double)  seconds, default 8.0
 */
class SetForkPositionAction : public BT::StatefulActionNode
{
public:
  SetForkPositionAction(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("main_villas_pos"),
      BT::InputPort<double>("second_villas_pos"),
      BT::InputPort<double>("tolerance", 0.01, "position tolerance [m or rad]"),
      BT::InputPort<double>("timeout", 8.0, "timeout [s]"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void jointStateCallback(sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::mutex mutex_;
  double current_main_villas_{0.0};
  double current_second_villas_{0.0};
  bool have_joint_state_{false};

  double target_main_villas_{0.0};
  double target_second_villas_{0.0};
  double tolerance_{0.01};
  rclcpp::Time start_time_;
  double timeout_s_{8.0};
};

}  // namespace avf_15_pallet_bt_plugins

#endif  // AVF_15_PALLET_BT_PLUGINS__SET_FORK_POSITION_ACTION_HPP_
