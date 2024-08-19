#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/tracking.hpp>
#include "yolov8_msgs/msg/yolov8_inference.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/point.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;

const double PI = 3.141592653589793238463;

class DynamicFrameBroadcaster : public rclcpp::Node
{
public:
  DynamicFrameBroadcaster()
  : Node("dynamic_frame_tf2_broadcaster")
  {
    yolo_sub_ = this->create_subscription<yolov8_msgs::msg::Yolov8Inference>("/Yolov8_Inference",
      10, bind(&DynamicFrameBroadcaster::yoloCallback, this, _1));
    depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/depth_camera/depth/image_raw", 10,
      std::bind(&DynamicFrameBroadcaster::depthImageCallback, this, _1));
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    cat_position_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("cat_position", 10);
  }

private:

  void yoloCallback(const yolov8_msgs::msg::Yolov8Inference &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_inference_msg_ = msg;
    const float horizontal_fov = 1.02974;
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
        float distance_to_center;
        /* Calculate distance to the center point*/
        if(center_y!=0 && center_x!=0)
        {
          cv::Mat depth_image = cv_bridge::toCvShare(depth_image_msg_, "32FC1")->image;

          distance_to_center = depth_image.at<float>(center_y, center_x);
          if(distance_to_center > 5.0)
          {
            distance_to_center = 0.0;
          }

          // Calculate the object's center in the image
          int obj_x_center = pet_obj.x + pet_obj.width / 2;
          int px_to_center = image_width_ / 2 - obj_x_center;

          float ang_to_obj = (px_to_center / static_cast<float>(image_width_)) * horizontal_fov;

          // Calculate the cat's position in the camera/robot frame
          float pet_pose_x = (distance_to_center) * std::cos(ang_to_obj); 
          float pet_pose_y = (distance_to_center) * std::sin(ang_to_obj);

          // Create and publish the cat's position
          geometry_msgs::msg::Point cat_position_msg;
          try
          {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
              "map",          // Target frame
              "camera_rgb_frame", // Source frame
              rclcpp::Time(0) // Lookup the latest available transform
            );
            // Apply the transformation
            tf2::Vector3 pet_position_camera(pet_pose_x, pet_pose_y, 0.0);
            tf2::Transform transform_tf;
            tf2::fromMsg(transform.transform, transform_tf);
            tf2::Vector3 pet_position_map = transform_tf * pet_position_camera;

            cat_position_msg.x = pet_position_map.x();
            cat_position_msg.y = pet_position_map.y();
            cat_position_msg.z = 0;

            // Publish the transformed position
            cat_position_publisher_->publish(cat_position_msg);

            RCLCPP_INFO(get_logger(), "Published cat position (map frame): x = %f, y = %f, z = %f",
                        cat_position_msg.x, cat_position_msg.y, cat_position_msg.z);
          }
          catch (const tf2::TransformException &ex)
          {
            RCLCPP_ERROR(get_logger(), "Could not transform to map frame: %s", ex.what());
          }
        }
      }
    } 
  }

  void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    depth_image_msg_ = msg;
  }

  int image_width_ = 640;
  std::mutex mutex_;

  yolov8_msgs::msg::Yolov8Inference last_inference_msg_;
  sensor_msgs::msg::Image::SharedPtr depth_image_msg_;
  rclcpp::Subscription<yolov8_msgs::msg::Yolov8Inference>::SharedPtr yolo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr cat_position_publisher_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}