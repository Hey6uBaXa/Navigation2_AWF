#include <string>
#include <memory>

#include "avf_15_pallet_bt_plugins/dock_at_pallet_action.hpp"

namespace avf_15_pallet_bt_plugins
{

DockAtPalletAction::DockAtPalletAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<opennav_docking_msgs::action::DockRobot>(
    xml_tag_name, action_name, conf)
{
}

void DockAtPalletAction::on_tick()
{
  geometry_msgs::msg::PoseStamped dock_pose;
  std::string dock_type = "pallet_dock";
  double max_staging_time = 60.0;

  getInput("dock_pose", dock_pose);
  getInput("dock_type", dock_type);
  getInput("max_staging_time", max_staging_time);

  goal_.use_dock_id = false;          // we localize dynamically, not by fixed dock_id
  goal_.dock_pose = dock_pose;        // initial estimate; plugin refines via external pose
  goal_.dock_type = dock_type;
  goal_.max_staging_time = static_cast<float>(max_staging_time);
  goal_.navigate_to_staging_pose = true;
}

BT::NodeStatus DockAtPalletAction::on_success()
{
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DockAtPalletAction::on_aborted()
{
  return BT::NodeStatus::FAILURE;
}

}  // namespace avf_15_pallet_bt_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<avf_15_pallet_bt_plugins::DockAtPalletAction>(
        name, "dock_robot", config);
    };
  factory.registerBuilder<avf_15_pallet_bt_plugins::DockAtPalletAction>(
    "DockAtPallet", builder);
}
