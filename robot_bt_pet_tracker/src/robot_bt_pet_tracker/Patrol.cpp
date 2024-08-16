#include <string>
#include <iostream>

#include "robot_bt_pet_tracker/Patrol.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace robot_bt_pet_tracker
{

using namespace std::chrono_literals;

Patrol::Patrol(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void
Patrol::halt()
{
  std::cout << "Patrol halt" << std::endl;
}

BT::NodeStatus
Patrol::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
  }

  geometry_msgs::msg::Twist vel_msgs;
  vel_msgs.angular.z = 0.5;
  vel_pub_->publish(vel_msgs);

  auto elapsed = node_->now() - start_time_;

  if (elapsed < 15s) {
    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace robot_bt_pet_tracker

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robot_bt_pet_tracker::Patrol>("Patrol");
}
