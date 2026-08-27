#ifndef AVF_15_PALLET_BT_PLUGINS__DOCK_AT_PALLET_ACTION_HPP_
#define AVF_15_PALLET_BT_PLUGINS__DOCK_AT_PALLET_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "opennav_docking_msgs/action/dock_robot.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace avf_15_pallet_bt_plugins
{

/**
 * Thin wrapper around opennav_docking's DockRobot action, reused for
 * pallet docking. docking_server must have a dock instance configured
 * (dock_type, e.g. "pallet_dock") whose plugin refines the goal pose
 * using the external detection topic published by pallet_detector_node
 * (use_external_detection_pose: true), exactly like it already does
 * for the charging dock.
 *
 * Ports:
 *   dock_pose   (PoseStamped) initial pose estimate (from pallet_detector)
 *   dock_type   (string)      docking_server dock_type name, default "pallet_dock"
 *   max_staging_time (double) seconds, default 60.0
 */
class DockAtPalletAction
  : public nav2_behavior_tree::BtActionNode<opennav_docking_msgs::action::DockRobot>
{
public:
  DockAtPalletAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<geometry_msgs::msg::PoseStamped>("dock_pose"),
      BT::InputPort<std::string>("dock_type", "pallet_dock", "docking_server dock_type"),
      BT::InputPort<double>("max_staging_time", 60.0, "seconds"),
    });
  }
};

}  // namespace avf_15_pallet_bt_plugins

#endif  // AVF_15_PALLET_BT_PLUGINS__DOCK_AT_PALLET_ACTION_HPP_
