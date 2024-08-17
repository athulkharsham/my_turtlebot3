#ifndef ROBOT_BT_PETTRACKER_FINDPET_HPP_
#define ROBOT_BT_PETTRACKER_FINDPET_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/twist.hpp"
#include "yolov8_msgs/msg/yolov8_inference.hpp"
#include <opencv2/tracking.hpp>
#include "rclcpp/rclcpp.hpp"

namespace robot_bt_pet_tracker
{

class FindPet : public BT::ActionNodeBase
{
public:
  constexpr static float MIN_ANG_VEL = 0.15f;
  constexpr static float MAX_ANG_VEL = 0.5f;
  constexpr static float ANGULAR_GAIN = 1.7f;
  explicit FindPet(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

  void yoloCallback(const yolov8_msgs::msg::Yolov8Inference &msg);
  void designateControl(geometry_msgs::msg::Twist &vel_msg, cv::Rect obj, uint32_t img_width);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<yolov8_msgs::msg::Yolov8Inference>::SharedPtr yolo_sub_;
  yolov8_msgs::msg::Yolov8Inference last_inference_msg_;
  std::mutex msg_mutex_;
  bool is_aligned_;
};

}  // namespace robot_bt_pet_tracker

#endif  // ROBOT_BT_PETTRACKER_FINDPET_HPP_
