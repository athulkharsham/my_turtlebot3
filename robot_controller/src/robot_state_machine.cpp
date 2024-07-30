#include "robot_controller/robot_state_machine.hpp"
#include <chrono>
#include <string>
#include <cstdlib>



StateMachineNode::StateMachineNode()
	: Node("state_machine_node"), current_state_(State::INIT)
{
	// Initialize timer for state machine loop
	state_machine_timer_ = this->create_wall_timer(
		std::chrono::milliseconds(100),
		std::bind(&StateMachineNode::stateMachineLoop, this));

	// Create the exploration status subscription
	exploration_status_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
		"/explore/status", 10,
		std::bind(&StateMachineNode::explorationStatusCallback, this, std::placeholders::_1));

	state_publisher_ = this->create_publisher<std_msgs::msg::String>("robot_current_state", 10);

	// Create a publisher for the /initialpose topic
	initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
		"/initialpose", 10);

	// Create the action client
	docking_action_client_ = rclcpp_action::create_client<robot_interfaces::action::Docking>(this, "dock_robot");

	RCLCPP_INFO(this->get_logger(), "State Machine Initialized.");
}

void StateMachineNode::stateMachineLoop()
{
	switch (current_state_)
	{
		case State::INIT:
			initCallback();
			break;
		case State::EXPLORATION:
			explorationCallback();
			break;
		case State::DOCKING:
			dockingCallback();
			break;
		case State::IDLE:
			idleCallback();
			break;
		default:
			RCLCPP_ERROR(this->get_logger(), "Unknown state.");
			break;
	}
}

void StateMachineNode::changeState(State new_state)
{
	current_state_ = new_state;
	publishState(new_state);
}

bool StateMachineNode::mapFileExists(const std::string &file_path)
{
  std::ifstream file(file_path);
  return file.good();
}

void StateMachineNode::publishState(State state)
{
	auto message = std::make_shared<std_msgs::msg::String>();
	
	switch (state)
	{
		case State::INIT:
			message->data = "INIT";
			break;
		case State::EXPLORATION:
			message->data = "EXPLORATION";
			break;
		case State::DOCKING:
			message->data = "DOCKING";
			break;
		case State::IDLE:
			message->data = "IDLE";
			break;
		default:
			message->data = "UNKNOWN";
			break;
	}
	state_publisher_->publish(*message);
}

void StateMachineNode::publishInitialPose()
{
	geometry_msgs::msg::PoseWithCovarianceStamped msg;

	// Set the header
	msg.header.stamp = this->get_clock()->now();
	msg.header.frame_id = "map";

	// Set the pose
	msg.pose.pose.position.x = 0.0;
	msg.pose.pose.position.y = 0.0;
	msg.pose.pose.position.z = 0.0;
	msg.pose.pose.orientation.x = 0.0;
	msg.pose.pose.orientation.y = 0.0;
	msg.pose.pose.orientation.z = 0.0;
	msg.pose.pose.orientation.w = 1.0;  // No rotation

	RCLCPP_INFO(this->get_logger(), "Publishing initial pose: (0, 0, 0)");

	initial_pose_publisher_->publish(msg);
}

void StateMachineNode::explorationStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
	is_exploration_completed_ = msg->data;
}


void StateMachineNode::sendDockingGoal()
{
    if (!docking_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
	    RCLCPP_ERROR(this->get_logger(), "DockRobot action server not available.");
    	return;
    }
	// Create the goal message
	auto goal_msg = robot_interfaces::action::Docking::Goal();
	goal_msg.start_docking = true; 

    auto send_goal_options = rclcpp_action::Client<robot_interfaces::action::Docking>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&StateMachineNode::resultCallback, this, _1);

    docking_action_client_->async_send_goal(goal_msg, send_goal_options);
}


void StateMachineNode::resultCallback(const rclcpp_action::ClientGoalHandle<robot_interfaces::action::Docking>::WrappedResult &result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Docking succeeded!");
            is_docking_completed_ = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Docking was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Docking Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }
}

void StateMachineNode::initCallback()
{
	// RCLCPP_INFO(this->get_logger(), "Executing Init State");
	// Check if the map file exists
	std::string map_file_path = "/home/ubuntu/turtlebot3_ws/src/turtlebot3_gazebo/maps/robot_house.yaml";
	if (mapFileExists(map_file_path))
	{
		RCLCPP_INFO(this->get_logger(), "Map file exists. Transitioning to Idle state.");
		publishInitialPose();
		changeState(State::IDLE);
	}
	else
	{
		RCLCPP_INFO(this->get_logger(), "Map file does not exists. Transitioning to Exploration state.");
		changeState(State::EXPLORATION);
	}
}

void StateMachineNode::explorationCallback()
{
	// RCLCPP_INFO(this->get_logger(), "Executing Exploration State");
	// RCLCPP_INFO(this->get_logger(), "Exploration status %d", (int)is_exploration_completed_);
	if(is_exploration_completed_)
	{
		changeState(State::DOCKING);
	}
}

void StateMachineNode::dockingCallback()
{
	if(is_docking_goal_sent_ == false)
	{
		is_docking_goal_sent_ = true;
		sendDockingGoal();
	}
	if(is_docking_completed_)
	{
		changeState(State::IDLE);
	}
}

void StateMachineNode::idleCallback()
{
	// RCLCPP_INFO(this->get_logger(), "Executing Idle State");
	// changeState(State::EXPLORATION);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StateMachineNode>());
	rclcpp::shutdown();
	return 0;
}