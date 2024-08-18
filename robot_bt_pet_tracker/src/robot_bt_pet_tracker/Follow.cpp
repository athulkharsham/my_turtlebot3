#include <string>
#include <iostream>
#include <chrono>

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

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

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
    "/depth_camera/depth/image_raw", 10,
    std::bind(&Follow::depthImageCallback, this, _1));

  nav2_action_client_ = rclcpp_action::create_client<NavigateToPose>(
      node_,
      "navigate_to_pose"
  );

}

void Follow::yoloCallback(const yolov8_msgs::msg::Yolov8Inference &msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_inference_msg_ = msg;
}

void Follow::depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  depth_image_msg_ = msg;
}

void Follow::designateControl(geometry_msgs::msg::Twist &vel_msg, 
  cv::Rect obj, uint32_t img_width, float distance_to_center)
{

  int obj_x_center = obj.x + obj.width / 2;
  int px_to_center = img_width / 2 - obj_x_center;
  float ang_vel = ANGULAR_GAIN * px_to_center / static_cast<float>(img_width);

  const double FIXED_DISTANCE = 0.5;

  if ((ang_vel >= -MAX_ANG_VEL && ang_vel <= -MIN_ANG_VEL) || 
      (ang_vel >= MIN_ANG_VEL && ang_vel <= MAX_ANG_VEL)) 
  {
    vel_msg.angular.z = ang_vel;
  }

  const float horizontal_fov = 1.02974;
  float ang_to_obj = (px_to_center / static_cast<float>(img_width)) * horizontal_fov;

  RCLCPP_INFO(node_->get_logger(), "Distance =%f, Angle=%f", distance_to_center, ang_to_obj);

  if(distance_to_center >= 0.8 && distance_to_center < 4.5 && distance_to_center != 0.0)
  {
    if(ang_to_obj <= 0.1)
    {
      vel_msg.linear.x = (distance_to_center - FIXED_DISTANCE) * 0.1;
    }
    else
    {
      //do nothing
    }
  }
}

void Follow::sendNavGoal(cv::Rect obj, uint32_t img_width, float distance_to_center)
{
    // Calculate the object's center in the image
    int obj_x_center = obj.x + obj.width / 2;
    int px_to_center = img_width / 2 - obj_x_center;

    // Horizontal angle to the object
    const float horizontal_fov = 1.02974;
    const double FIXED_DISTANCE = 0.5;

    float ang_to_obj = (px_to_center / static_cast<float>(img_width)) * horizontal_fov;

    // Calculate the cat's position in the camera/robot frame
    float pet_pose_x = (distance_to_center - FIXED_DISTANCE) * std::cos(ang_to_obj); 
    float pet_pose_y = (distance_to_center - FIXED_DISTANCE) * std::sin(ang_to_obj);

    // Create the target position
    geometry_msgs::msg::Point target_position;
    target_position.x = pet_pose_x;
    target_position.y = pet_pose_y;

    // Check if the goal is the same as the previous one
    bool same_goal = same_point(prev_goal_, target_position);
    prev_goal_ = target_position;

    if (same_goal) {
        return;
    }


    // Prepare the goal message
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.stamp = node_->now();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = target_position.x;
    goal_msg.pose.pose.position.y = target_position.y;

    // Align the robot towards the cat
    tf2::Quaternion q;
    q.setRPY(0, 0, std::atan2(target_position.x, target_position.y));
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    // Send the goal
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&Follow::resultCallback, this, std::placeholders::_1);
    
    // nav2_action_client_->async_send_goal(goal_msg, send_goal_options);
}


void Follow::resultCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "Object track Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "Object track Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node_->get_logger(), "Object track Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
            return;
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
  float distance_to_center = 0;
  std::lock_guard<std::mutex> lock(mutex_);
  for(long unsigned int i = 0 ; i < last_inference_msg_.yolov8_inference.size(); i++)
  {
    if(last_inference_msg_.yolov8_inference[i].class_name == "cat" || 
       last_inference_msg_.yolov8_inference[i].class_name == "dog" ||
       last_inference_msg_.yolov8_inference[i].class_name == "horse"||
       last_inference_msg_.yolov8_inference[i].class_name == "bird"
       )
    {
      auto top = last_inference_msg_.yolov8_inference[0].top;
      auto left = last_inference_msg_.yolov8_inference[0].left;
      auto bottom = last_inference_msg_.yolov8_inference[0].bottom;
      auto right = last_inference_msg_.yolov8_inference[0].right;
      int center_x = top + (bottom-top)/2;
      int center_y = left + (right-left)/2;

      cv::Rect pet_obj(top, left, bottom-top, right-left);

      /* Calculate distance to the center point*/
      if(center_y!=0 && center_x!=0)
      {
        cv::Mat depth_image = cv_bridge::toCvShare(depth_image_msg_, "32FC1")->image;
        geometry_msgs::msg::Twist vel_msg;
        distance_to_center = depth_image.at<float>(center_y, center_x);
        if(distance_to_center > 5.0)
        {
          distance_to_center = 0.0;
        }
        designateControl(vel_msg, pet_obj, image_width_, distance_to_center);
        vel_pub_->publish(vel_msg);
      }
    }
  }




  return BT::NodeStatus::RUNNING;
}

}  // namespace robot_bt_pet_tracker

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robot_bt_pet_tracker::Follow>("Follow");
}