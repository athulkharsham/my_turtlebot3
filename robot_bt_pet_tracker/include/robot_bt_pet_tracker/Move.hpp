#ifndef ROBOT_BT_PETTRACKER_MOVE_HPP_
#define ROBOT_BT_PETTRACKER_MOVE_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "robot_bt_pet_tracker/ctrl_support/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace robot_bt_pet_tracker
{

class Move : public robot_bt_pet_tracker::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  explicit Move(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")
    };
  }
};

}  // namespace robot_bt_pet_tracker

#endif  // ROBOT_BT_PETTRACKER_MOVE_HPP_
