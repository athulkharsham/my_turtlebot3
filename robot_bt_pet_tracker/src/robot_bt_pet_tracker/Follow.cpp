#include <string>
#include <iostream>

#include "robot_bt_pet_tracker/Follow.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace robot_bt_pet_tracker
{

using namespace std::chrono_literals;
using namespace std::placeholders;

Follow::Follow(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS());
  yolo_sub_ = node_->create_subscription<yolov8_msgs::msg::Yolov8Inference>("/Yolov8_Inference",
    10, bind(&Follow::yoloCallback, this, _1));
  depth_image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/depth_camera/depth/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&Follow::depthImageCallback, this, _1));
}

void Follow::yoloCallback(const yolov8_msgs::msg::Yolov8Inference &msg)
{
  geometry_msgs::msg::Twist vel_msg;  

  for(long unsigned int i = 0 ; i < msg.yolov8_inference.size(); i++)
  {
    if(msg.yolov8_inference[i].class_name == "cat"  || 
      msg.yolov8_inference[i].class_name == "dog"   ||
      msg.yolov8_inference[i].class_name == "horse" ||
      msg.yolov8_inference[i].class_name == "person")
    {
      auto top = msg.yolov8_inference[0].top;
      auto left = msg.yolov8_inference[0].left;
      auto bottom = msg.yolov8_inference[0].bottom;
      auto right = msg.yolov8_inference[0].right;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        center_x_ = top + (bottom-top)/2;
        center_y_ = left + (right-left)/2;
      }

      cv::Rect pet_obj(top, left, bottom-top, right-left);
      designateControl(vel_msg, pet_obj, image_width_);
      vel_pub_->publish(vel_msg);
    }
  }
}

void Follow::depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat depth_image = cv_bridge::toCvShare(msg, "32FC1")->image;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if(center_y_!=0 && center_x_!=0)
    {
      float depth_value = depth_image.at<float>(center_y_, center_x_);
      if(depth_value < 5.0)
      {
        center_distance_ = depth_value;
      }
      else
      {
        depth_value = 0.0;
      }
    }
  }
}

void Follow::designateControl(geometry_msgs::msg::Twist &vel_msg, cv::Rect obj, uint32_t img_width)
{

  int obj_x_center = obj.x + obj.width / 2;
  int px_to_center = img_width / 2 - obj_x_center;
  float ang_vel = ANGULAR_GAIN * px_to_center / static_cast<float>(img_width);

  const double FIXED_DISTANCE = 0.5;

  if ((ang_vel >= -MAX_ANG_VEL && ang_vel <= -MIN_ANG_VEL) || (ang_vel >= MIN_ANG_VEL && ang_vel <= MAX_ANG_VEL)) 
  {
    vel_msg.angular.z = ang_vel;
  }
  if(center_distance_ > 0.8 && center_distance_!= 0.0)
  {
    vel_msg.linear.x = (center_distance_ - FIXED_DISTANCE) * 0.9;
  }
}


void
Follow::halt()
{
  std::cout << "Follow halt" << std::endl;
}

BT::NodeStatus
Follow::tick()
{
  return BT::NodeStatus::RUNNING;
}

}  // namespace robot_bt_pet_tracker

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robot_bt_pet_tracker::Follow>("Follow");
}
