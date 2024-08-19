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

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock(), tf2::durationFromSec(10.0));
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

void Follow::calculatePetPoseInMap(float ang_to_obj, float distance_to_center)
{
  float pet_pose_x = (distance_to_center - FIXED_DISTANCE) * std::cos(ang_to_obj); 
  float pet_pose_y = (distance_to_center - FIXED_DISTANCE) * std::sin(ang_to_obj);
  geometry_msgs::msg::Point cat_position_with_map_msg;
  calculatePetToMapTransform(cat_position_with_map_msg, pet_pose_x, pet_pose_y);
  waypoints_.push(createWaypoint(cat_position_with_map_msg));
}

// void Follow::calculatePetPoseInMap(float ang_to_obj, float distance_to_center)
// {
//   const int num_waypoints = 3;
//   float step_distance = (distance_to_center - FIXED_DISTANCE) / num_waypoints;

//   for (int i = 1; i <= num_waypoints; ++i)
//   {
//     // Calculate the intermediate waypoint positions
//     float pet_pose_x = i * step_distance * std::cos(ang_to_obj);
//     float pet_pose_y = i * step_distance * std::sin(ang_to_obj);

//     geometry_msgs::msg::Point cat_position_with_map_msg;
//     calculatePetToMapTransform(cat_position_with_map_msg, pet_pose_x, pet_pose_y);

//     waypoints_.push(createWaypoint(cat_position_with_map_msg));
//   }
// }


void Follow::sendNavGoal()
{
  if (waypoints_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "All waypoints have been visited.");
    return;
  }

  // Prepare the goal message
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = waypoints_.front();

  // Send the goal
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&Follow::resultCallback, this, std::placeholders::_1);
  nav2_action_client_->async_send_goal(goal_msg, send_goal_options);
}


void Follow::calculatePetToMapTransform(geometry_msgs::msg::Point &cat_position_msg, 
  float pet_pose_x, float pet_pose_y)
{
  try
  {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      "map",          // Target frame
      "camera_rgb_frame", // Source frame
      rclcpp::Time(0)
    );
    // Apply the transformation
    tf2::Vector3 pet_position_camera(pet_pose_x, pet_pose_y, 0.0);
    tf2::Transform transform_tf;
    tf2::fromMsg(transform.transform, transform_tf);
    tf2::Vector3 pet_position_map = transform_tf * pet_position_camera;

    cat_position_msg.x = pet_position_map.x();
    cat_position_msg.y = pet_position_map.y();
    cat_position_msg.z = 0;

    // RCLCPP_INFO(node_->get_logger(), "Cat position in map frame: x = %f, y = %f",
    //             cat_position_msg.x, cat_position_msg.y);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not transform to map frame: %s", ex.what());
  }
}


geometry_msgs::msg::PoseStamped Follow::createWaypoint(geometry_msgs::msg::Point &cat_position_msg)
{
  geometry_msgs::msg::PoseStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = node_->now();

  waypoint.pose.position.x = cat_position_msg.x;
  waypoint.pose.position.y = cat_position_msg.y;
  waypoint.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, std::atan2(cat_position_msg.y, cat_position_msg.x));
  waypoint.pose.orientation = tf2::toMsg(q);

  RCLCPP_INFO(node_->get_logger(), "Way point Created!");

  return waypoint;
}

void Follow::resultCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Object track Goal succeeded!");
      waypoints_.pop();
      sendNavGoal();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Object track Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Object track Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      break;
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