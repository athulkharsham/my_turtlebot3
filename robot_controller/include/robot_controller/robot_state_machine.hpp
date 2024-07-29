#ifndef ROBOT_STATEMACHINE_HPP
#define ROBOT_STATEMACHINE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include <fstream>

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
    // Callback functions for various states
    void initCallback();
    void explorationCallback();
    void dockingCallback();
    void idleCallback();

    rclcpp::TimerBase::SharedPtr state_machine_timer_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exploration_status_subscription_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
    State current_state_;
	bool is_exploration_completed_  = false;
};

#endif  // ROBOT_STATEMACHINE_HPP