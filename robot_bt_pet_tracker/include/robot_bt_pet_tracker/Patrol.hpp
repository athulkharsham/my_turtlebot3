#ifndef ROBOT_BT_PETTRACKER_PATROL_HPP_
#define ROBOT_BT_PETTRACKER_PATROL_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/twist.hpp"
#include "yolov8_msgs/msg/yolov8_inference.hpp"

#include "rclcpp/rclcpp.hpp"

namespace robot_bt_pet_tracker
{

class Patrol : public BT::ActionNodeBase
{
public:
  explicit Patrol(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

  void yoloCallback(const yolov8_msgs::msg::Yolov8Inference &msg);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  bool is_pet_found_;
  rclcpp::Subscription<yolov8_msgs::msg::Yolov8Inference>::SharedPtr yolo_sub_;
};

}  // namespace robot_bt_pet_tracker

#endif  // ROBOT_BT_PETTRACKER_PATROL_HPP_
