#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

class RobotPosePublisher : public rclcpp::Node
{
public:
    RobotPosePublisher()
        : Node("robot_pose_publisher")
    {
        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize publisher
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("robot_pose", 10);

        // Timer to periodically publish robot's position
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { publish_robot_pose(); });
        RCLCPP_INFO(get_logger(), "Robot pose publisher started");
    }

private:
    void publish_robot_pose()
    {
        geometry_msgs::msg::Point pose_msg;

        try {
            // Lookup transform from 'map' to 'base_footprint'
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map", "base_footprint", rclcpp::Time(0));

            // Extract position from the transform
            pose_msg.x = transform.transform.translation.x;
            pose_msg.y = transform.transform.translation.y;
            pose_msg.z = transform.transform.translation.z;

            // Publish the pose message
            pose_publisher_->publish(pose_msg);
        } catch (tf2::TransformException &ex) {
            // RCLCPP_WARN(this->get_logger(), "Could not transform 'base_footprint' to 'map': %s", ex.what());
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pose_publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotPosePublisher>());
    rclcpp::shutdown();
    return 0;
}
