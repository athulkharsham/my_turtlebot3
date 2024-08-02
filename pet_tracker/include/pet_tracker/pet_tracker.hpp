#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/tracking.hpp>
#include "yolov8_msgs/msg/yolov8_inference.hpp"

class Tracker : public rclcpp::Node
{
public:
  constexpr static float MIN_ANG_VEL = 0.15f;
  constexpr static float MAX_ANG_VEL = 0.5f;
  constexpr static float ANGULAR_GAIN = 1.7f;

  Tracker();

private:
  void _imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void _initTracker(cv::Mat frame, cv::Rect obj);
  void _designateControl(geometry_msgs::msg::Twist &vel_msg, cv::Rect obj, uint32_t img_width);
  void _scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void _yoloCallback(const yolov8_msgs::msg::Yolov8Inference &msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _img_sub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _visualization_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _vel_pub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub;
  rclcpp::Subscription<yolov8_msgs::msg::Yolov8Inference>::SharedPtr _yolo_sub;
  cv::Ptr<cv::Tracker> _tracker;
  bool _is_tracker_initialized;
  double _center_distance;
  int _image_width;
};