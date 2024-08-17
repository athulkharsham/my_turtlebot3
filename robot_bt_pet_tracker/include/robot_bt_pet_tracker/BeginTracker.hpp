#ifndef ROBOT_BT_PETTRACKER_BEGINTRACKER_HPP_
#define ROBOT_BT_PETTRACKER_BEGINTRACKER_HPP_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <std_msgs/msg/bool.hpp>
#include "rclcpp/rclcpp.hpp"

namespace robot_bt_pet_tracker
{

class BeginTracker : public BT::ConditionNode
{
public:
  explicit BeginTracker(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr track_sub_;
  std::mutex msg_mutex_;
  bool track_pet_;

  void trackCallback(const std_msgs::msg::Bool &msg);
};

}  // namespace robot_bt_pet_tracker

#endif  // ROBOT_BT_PETTRACKER_BEGINTRACKER_HPP_