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
    	"explore/status", 10,
      	std::bind(&StateMachineNode::explorationStatusCallback, this, std::placeholders::_1));

	state_publisher_ = this->create_publisher<std_msgs::msg::String>("robot_current_state", 10);

    // Create a publisher for the /initialpose topic
    initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);

	// Create the action client
    docking_action_client_ = rclcpp_action::create_client<robot_interfaces::action::Docking>(this, "dock_robot");

    if (!docking_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
	    RCLCPP_ERROR(this->get_logger(), "DockRobot action server not available.");
    	return;
    }

	RCLCPP_INFO(this->get_logger(), "State Machine Initialized.");
}

void StateMachineNode::stateMachineLoop()
{
	while(rclcpp::ok())
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
	return;
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


void StateMachineNode::initCallback()
{
	RCLCPP_INFO(this->get_logger(), "Executing Init State");

	// Check if the map file exists
	std::string map_file_path = "/home/ubuntu/turtlebot3_ws/src/turtlebot3_gazebo/maps/map_my_house.yaml";
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
	RCLCPP_INFO(this->get_logger(), "Executing Exploration State");
	if(is_exploration_completed_)
	{
		changeState(State::DOCKING);
	}
}

void StateMachineNode::dockingCallback()
{
	RCLCPP_INFO(this->get_logger(), "Executing Docking State");
	changeState(State::IDLE);
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