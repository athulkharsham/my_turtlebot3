#ifndef ROBOT_BT_PETTRACKER_FOLLOW_HPP_
#define ROBOT_BT_PETTRACKER_FOLLOW_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/twist.hpp"
#include "yolov8_msgs/msg/yolov8_inference.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/tracking.hpp>
#include "yolov8_msgs/msg/yolov8_inference.hpp"

#include "rclcpp/rclcpp.hpp"

namespace robot_bt_pet_tracker
{

class Follow : public BT::ActionNodeBase
{
public:
  constexpr static float MIN_ANG_VEL = 0.15f;
  constexpr static float MAX_ANG_VEL = 0.5f;
  constexpr static float ANGULAR_GAIN = 1.7f;
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
  void designateControl(geometry_msgs::msg::Twist &vel_msg, cv::Rect obj, uint32_t img_width);
  void yoloCallback(const yolov8_msgs::msg::Yolov8Inference &msg);
  void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<yolov8_msgs::msg::Yolov8Inference>::SharedPtr yolo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;

  cv::Ptr<cv::Tracker> tracker_;
  bool is_tracker_initialized_;
  float center_distance_;
  int center_x_, center_y_;
  int image_width_ = 640;
  std::mutex mutex_;
};

}  // namespace robot_bt_pet_tracker

#endif  // ROBOT_BT_PETTRACKER_FOLLOW_HPP_
