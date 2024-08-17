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

  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS());
  yolo_sub_ = node_->create_subscription<yolov8_msgs::msg::Yolov8Inference>("/Yolov8_Inference",
    10, bind(&FindPet::yoloCallback, this, _1));
  is_aligned_ = false;

}

void FindPet::yoloCallback(const yolov8_msgs::msg::Yolov8Inference &msg)
{
  std::lock_guard<std::mutex> lock(msg_mutex_);
  last_inference_msg_ = msg;
}

void FindPet::designateControl(geometry_msgs::msg::Twist &vel_msg, cv::Rect obj, uint32_t img_width)
{
  int obj_x_center = obj.x + obj.width / 2;
  int px_to_center = img_width / 2 - obj_x_center;
  float ang_vel = ANGULAR_GAIN * px_to_center / static_cast<float>(img_width);

  if ((ang_vel >= -MAX_ANG_VEL && ang_vel <= -MIN_ANG_VEL) || 
      (ang_vel >= MIN_ANG_VEL && ang_vel <= MAX_ANG_VEL)) 
  {
    vel_msg.angular.z = ang_vel;
    if(ang_vel < 0.09)
    {
      vel_msg.angular.z = 0.0;
      is_aligned_ = true;
    }
    else
    {
      is_aligned_ = false;
    }
  }
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
      auto top = last_inference_msg_.yolov8_inference[0].top;
      auto left = last_inference_msg_.yolov8_inference[0].left;
      auto bottom = last_inference_msg_.yolov8_inference[0].bottom;
      auto right = last_inference_msg_.yolov8_inference[0].right;

      cv::Rect pet_obj(top, left, bottom-top, right-left);

      geometry_msgs::msg::Twist vel_msg;

      designateControl(vel_msg, pet_obj, 640);
      vel_pub_->publish(vel_msg);
      if(is_aligned_)
      {
        return BT::NodeStatus::SUCCESS;
      }
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
