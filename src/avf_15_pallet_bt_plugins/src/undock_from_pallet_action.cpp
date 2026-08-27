#include <string>
#include <memory>

#include "avf_15_pallet_bt_plugins/undock_from_pallet_action.hpp"

namespace avf_15_pallet_bt_plugins
{

UndockFromPalletAction::UndockFromPalletAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<opennav_docking_msgs::action::UndockRobot>(
    xml_tag_name, action_name, conf)
{
}

void UndockFromPalletAction::on_tick()
{
  std::string dock_type = "pallet_dock";
  double max_undocking_time = 30.0;
  getInput("dock_type", dock_type);
  getInput("max_undocking_time", max_undocking_time);

  goal_.dock_type = dock_type;
  goal_.max_undocking_time = static_cast<float>(max_undocking_time);
}

BT::NodeStatus UndockFromPalletAction::on_success()
{
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus UndockFromPalletAction::on_aborted()
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
      return std::make_unique<avf_15_pallet_bt_plugins::UndockFromPalletAction>(
        name, "undock_robot", config);
    };
  factory.registerBuilder<avf_15_pallet_bt_plugins::UndockFromPalletAction>(
    "UndockFromPallet", builder);
}
