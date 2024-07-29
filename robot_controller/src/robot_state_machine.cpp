#include "robot_controller/robot_state_machine.hpp"

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


void StateMachineNode::explorationStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
	is_exploration_completed_ = msg->data;
}

void StateMachineNode::initCallback()
{
	RCLCPP_INFO(this->get_logger(), "Executing Init State");

	// Check if the map file exists
	std::string map_file_path = "/home/ubuntu/turtlebot3_ws/src/turtlebot3_gazebo/maps/robot_house.yaml";
	if (mapFileExists(map_file_path))
	{
		RCLCPP_INFO(this->get_logger(), "Map file exists. Transitioning to Idle state.");
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
	RCLCPP_INFO(this->get_logger(), "Executing Idle State");
	changeState(State::EXPLORATION);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StateMachineNode>());
	rclcpp::shutdown();
	return 0;
}