#include "pet_tracker/pet_tracker.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::placeholders;

Tracker::Tracker() : Node("tracker"), _is_tracker_initialized(false)
{
  _img_sub = create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", rclcpp::SensorDataQoS(), bind(&Tracker::_imageCallback, this, _1));

  _visualization_pub = create_publisher<sensor_msgs::msg::Image>("/visualization", rclcpp::SensorDataQoS());
  _vel_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS());
  _scan_sub = create_subscription<sensor_msgs::msg::LaserScan>("/scan",
    10, bind(&Tracker::_scanCallback, this, _1));
  _yolo_sub = create_subscription<yolov8_msgs::msg::Yolov8Inference>("/Yolov8_Inference",
    10, bind(&Tracker::_yoloCallback, this, _1));

  _depth_image_subscriber = create_subscription<sensor_msgs::msg::Image>(
            "/depth_camera/depth/image_raw", 10,
            std::bind(&Tracker::_depthImageCallback, this, _1));

  RCLCPP_INFO(get_logger(), "Node started!");
}

void Tracker::_imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  _image_width = msg->width;
}

void Tracker::_yoloCallback(const yolov8_msgs::msg::Yolov8Inference &msg)
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

      _center_x = top + (bottom-top)/2;
      _center_y = left + (right-left)/2;

      cv::Rect cat_obj(top, left, bottom-top, right-left);
      _designateControl(vel_msg, cat_obj, _image_width);
      _vel_pub->publish(vel_msg);
    }
  }

}

void Tracker::_depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    // Convert ROS Image message to OpenCV Mat with 32-bit floating-point encoding
    cv::Mat depth_image = cv_bridge::toCvShare(msg, "32FC1")->image;

    // Access the depth value at the center pixel as a float
    if(_center_y!=0 && _center_x!=0)
    {
      float depth_value = depth_image.at<float>(_center_y, _center_x);
      if(depth_value < 5.0)
      {
        _center_distance = depth_value;
        RCLCPP_INFO(this->get_logger(), "Center X: %d, Y : %d Distance at center: %f meters", _center_x, _center_y, _center_distance);
      }
      else
      {
        depth_value = 0.0;
      }

    }

  }
  catch (const cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}


void Tracker::_scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  double distance =0;
  for(int i=0; i < 10; i++)
  {
    if(msg->ranges[i] < 5.0)
    {
      distance += msg->ranges[i];
    }
  }
  // _center_distance = distance / 5;

  // RCLCPP_INFO(get_logger(), "Center distance: %0.2f", _center_distance);
}


void Tracker::_designateControl(geometry_msgs::msg::Twist &vel_msg, cv::Rect obj, uint32_t img_width)
{

  int obj_x_center = obj.x + obj.width / 2;
  int px_to_center = img_width / 2 - obj_x_center;
  float ang_vel = ANGULAR_GAIN * px_to_center / static_cast<float>(img_width);

  const double fixed_distance = 0.5;

  if ((ang_vel >= -MAX_ANG_VEL && ang_vel <= -MIN_ANG_VEL) || (ang_vel >= MIN_ANG_VEL && ang_vel <= MAX_ANG_VEL)) 
  {
    vel_msg.angular.z = ang_vel;
  }
  if(_center_distance > 0.8 && _center_distance!= 0.0)
  {
    vel_msg.linear.x = (_center_distance - fixed_distance) * 0.9;
  }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Tracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}