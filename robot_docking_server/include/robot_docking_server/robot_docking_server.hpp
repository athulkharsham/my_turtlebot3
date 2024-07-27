#ifndef ROBOT_DOCKING_SERVER_HPP
#define ROBOT_DOCKING_SERVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/docking.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"


using Docking = robot_interfaces::action::Docking;
using DockingGoalHandle = rclcpp_action::ServerGoalHandle<Docking>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using namespace std::placeholders;

enum class State{
    INIT_DOCKING,
    MOVE_TO_PREPOSE,
    LINEAR_MOVEMENT,
    ANGULAR_MOVEMENT,
    GO_TO_DOCK
};


class DockingServerNode : public rclcpp::Node
{
public:
    DockingServerNode();
    constexpr static float ANGULAR_GAIN = 0.8f;
    
private:
    /*Methods*/
    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Docking::Goal> goal);
    
    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<DockingGoalHandle> goal_handle);
    
    void handle_accepted_callback(
        const std::shared_ptr<DockingGoalHandle> goal_handle);

    void execute_goal(
        const std::shared_ptr<DockingGoalHandle> goal_handle);
    
    bool send_nav2_goal();

    void result_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result);

    void publish_cmd_vel(double linear, double angular);

    void timer_callback();

    void aruco_pose_callback(const geometry_msgs::msg::PoseArray &msg);

    void angleController(geometry_msgs::msg::Twist &vel_msg, double yaw);

    /*Attributes*/
    rclcpp_action::Server<Docking>::SharedPtr robot_docking_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    bool is_docking_completed_;
    double x_offset_;
    double y_offset_;
    double theta_offset_;
    std::mutex mutex_;
    std::shared_ptr<DockingGoalHandle> goal_handle_;
    State state_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_action_client_;
    bool is_pre_pose_goal_reached_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr aruco_subscription_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif //ROBOT_DOCKING_SERVER_HPP