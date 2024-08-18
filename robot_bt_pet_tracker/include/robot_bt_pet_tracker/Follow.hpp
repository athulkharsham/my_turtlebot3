#ifndef ROBOT_BT_PETTRACKER_FOLLOW_HPP_
#define ROBOT_BT_PETTRACKER_FOLLOW_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/tracking.hpp>
#include "yolov8_msgs/msg/yolov8_inference.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2/convert.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace robot_bt_pet_tracker
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class Follow : public BT::ActionNodeBase
{
public:
  constexpr static float MIN_ANG_VEL = 0.15f;
  constexpr static float MAX_ANG_VEL = 0.5f;
  constexpr static float ANGULAR_GAIN = 1.2f;
  explicit Follow(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  rclcpp::Node::SharedPtr node_;
  void designateControl(geometry_msgs::msg::Twist &vel_msg, cv::Rect obj, uint32_t img_width, float distance_to_center);
  void yoloCallback(const yolov8_msgs::msg::Yolov8Inference &msg);
  void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void sendNavGoal(cv::Rect obj, uint32_t img_width, float distance_to_center);
  void resultCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<yolov8_msgs::msg::Yolov8Inference>::SharedPtr yolo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_action_client_;

  cv::Ptr<cv::Tracker> tracker_;
  int image_width_ = 640;
  std::mutex mutex_;

  yolov8_msgs::msg::Yolov8Inference last_inference_msg_;
  sensor_msgs::msg::Image::SharedPtr depth_image_msg_;
  geometry_msgs::msg::Point prev_goal_;
};

}  // namespace robot_bt_pet_tracker

#endif  // ROBOT_BT_PETTRACKER_FOLLOW_HPP_
