#ifndef ROBOT_STATEMACHINE_HPP
#define ROBOT_STATEMACHINE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/docking.hpp"
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include <fstream>

using namespace std::placeholders;


class StateMachineNode : public rclcpp::Node
{
public:
    enum class State
    {
        INIT,
        EXPLORATION,
        DOCKING,
        IDLE
    };

  StateMachineNode();

private:
    void stateMachineLoop();
    void changeState(State new_state);
    bool mapFileExists(const std::string &file_path);
  	void publishState(State state);
	void explorationStatusCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /*Callback functions for various states*/
    void initCallback();
    void explorationCallback();
    void dockingCallback();
    void idleCallback();
	void publishInitialPose();
    void sendDockingGoal();

    rclcpp::TimerBase::SharedPtr state_machine_timer_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exploration_status_subscription_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
	rclcpp_action::Client<robot_interfaces::action::Docking>::SharedPtr docking_action_client_;
    void resultCallback(const rclcpp_action::ClientGoalHandle<robot_interfaces::action::Docking>::WrappedResult &result);
    State current_state_;
	bool is_exploration_completed_  = false;
    bool is_docking_completed_ = false;
    bool is_docking_goal_sent_ = false;
};

#endif  // ROBOT_STATEMACHINE_HPP