#include <string>
#include <iostream>

#include "robot_bt_pet_tracker/FindPet.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace robot_bt_pet_tracker
{

using namespace std::chrono_literals;
using namespace std::placeholders;

FindPet::FindPet(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  yolo_sub_ = node_->create_subscription<yolov8_msgs::msg::Yolov8Inference>("/Yolov8_Inference",
    10, bind(&FindPet::yoloCallback, this, _1));

}

void FindPet::yoloCallback(const yolov8_msgs::msg::Yolov8Inference &msg)
{
  std::lock_guard<std::mutex> lock(msg_mutex_);
  last_inference_msg_ = msg;
}

void
FindPet::halt()
{
  std::cout << "FindPet halt" << std::endl;
}

BT::NodeStatus
FindPet::tick()
{
  std::lock_guard<std::mutex> lock(msg_mutex_);
  for (long unsigned int i = 0; i < last_inference_msg_.yolov8_inference.size(); i++)
  {
    if (last_inference_msg_.yolov8_inference[i].class_name == "cat" ||
        last_inference_msg_.yolov8_inference[i].class_name == "dog")
    {
      RCLCPP_INFO(node_->get_logger(), "Pet Found!!");
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace robot_bt_pet_tracker

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robot_bt_pet_tracker::FindPet>("FindPet");
}
