#include <string>
#include <iostream>

#include "robot_bt_pet_tracker/BeginTracker.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"

namespace robot_bt_pet_tracker
{

using namespace std::chrono_literals;
using namespace std::placeholders;

BeginTracker::BeginTracker(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  track_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/track_pet", 10,
        std::bind(&BeginTracker::trackCallback, this, _1));

  track_pet_ = false;
}

void BeginTracker::trackCallback(const std_msgs::msg::Bool &msg)
{
  std::lock_guard<std::mutex> lock(msg_mutex_);
  track_pet_ = msg.data;
}

BT::NodeStatus
BeginTracker::tick()
{
  std::lock_guard<std::mutex> lock(msg_mutex_);

  return track_pet_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace robot_bt_pet_tracker

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robot_bt_pet_tracker::BeginTracker>("BeginTracker");
}
