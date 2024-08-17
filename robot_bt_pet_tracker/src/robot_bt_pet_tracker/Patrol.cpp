#include <string>
#include <iostream>

#include "robot_bt_pet_tracker/Patrol.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace robot_bt_pet_tracker
{

using namespace std::chrono_literals;
using namespace std::placeholders;

Patrol::Patrol(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  yolo_sub_ = node_->create_subscription<yolov8_msgs::msg::Yolov8Inference>("/Yolov8_Inference",
    10, bind(&Patrol::yoloCallback, this, _1));
}

void Patrol::yoloCallback(const yolov8_msgs::msg::Yolov8Inference &msg)
{
  std::lock_guard<std::mutex> lock(msg_mutex_);
  last_inference_msg_ = msg;
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

  auto elapsed = node_->now() - start_time_;

  if (elapsed < 20s) 
  {
    geometry_msgs::msg::Twist vel_msgs;
    vel_msgs.angular.z = 0.5;
    vel_pub_->publish(vel_msgs);

    std::lock_guard<std::mutex> lock(msg_mutex_);
    for (long unsigned int i = 0; i < last_inference_msg_.yolov8_inference.size(); i++)
    {
      if (last_inference_msg_.yolov8_inference[i].class_name == "cat" ||
          last_inference_msg_.yolov8_inference[i].class_name == "dog")
      {
        vel_msgs.angular.z = 0.0;
        vel_pub_->publish(vel_msgs);
        RCLCPP_INFO(node_->get_logger(), "Pet Found!!");
        return BT::NodeStatus::SUCCESS;
      }
    }
    return BT::NodeStatus::RUNNING;
  } 
  else 
  {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace robot_bt_pet_tracker

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robot_bt_pet_tracker::Patrol>("Patrol");
}
