#ifndef AVF_15_PALLET_BT_PLUGINS__UNDOCK_FROM_PALLET_ACTION_HPP_
#define AVF_15_PALLET_BT_PLUGINS__UNDOCK_FROM_PALLET_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/undock_robot.hpp"

namespace avf_15_pallet_bt_plugins
{

/**
 * Thin wrapper around opennav_docking's UndockRobot action.
 * Ports:
 *   dock_type (string) same dock_type used to dock, default "pallet_dock"
 *   max_undocking_time (double) seconds, default 30.0
 */
class UndockFromPalletAction
  : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::UndockRobot>
{
public:
  UndockFromPalletAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("dock_type", "pallet_dock", "docking_server dock_type"),
      BT::InputPort<double>("max_undocking_time", 30.0, "seconds"),
    });
  }
};

}  // namespace avf_15_pallet_bt_plugins

#endif  // AVF_15_PALLET_BT_PLUGINS__UNDOCK_FROM_PALLET_ACTION_HPP_
