#ifndef ROBOT_BT_PETTRACKER_GETWAYPOINT_HPP_
#define ROBOT_BT_PETTRACKER_GETWAYPOINT_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot_bt_pet_tracker
{

class GetWaypoint : public BT::ActionNodeBase
{
public:
  explicit GetWaypoint(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("wp_id"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("waypoint")
      });
  }

private:
  geometry_msgs::msg::PoseStamped recharge_point_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  static int current_;
};

}  // namespace robot_bt_pet_tracker

#endif  // ROBOT_BT_PETTRACKER_GETWAYPOINT_HPP_
