#include "robot_docking_server/robot_docking_server.hpp"


DockingServerNode::DockingServerNode() :
    Node("robot_docking_server"), 
    is_docking_completed_(false),
    x_offset_(5.0),
    y_offset_(5.0),
    theta_offset_(0.0),
    state_(State::INIT_DOCKING),
    is_pre_pose_goal_reached_(false)
{
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    robot_docking_server_ = rclcpp_action::create_server<Docking>(
        this,
        "dock_robot",
        std::bind(&DockingServerNode::goal_callback, this, _1, _2),
        std::bind(&DockingServerNode::cancel_callback, this, _1),
        std::bind(&DockingServerNode::handle_accepted_callback, this, _1),
        rcl_action_server_get_default_options(),
        cb_group_
    );

    goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    nav2_action_client_ = rclcpp_action::create_client<NavigateToPose>(
        this,
        "navigate_to_pose"
    );

    RCLCPP_INFO(this->get_logger(), "Docking Action server has been started");

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DockingServerNode::timer_callback, this)
    );
}


rclcpp_action::GoalResponse DockingServerNode::goal_callback(
    const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Docking::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received a new goal");

    // Policy: refuse new goal if one goal is being active
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (goal_handle_) {
            if (goal_handle_->is_active()) {
                RCLCPP_INFO(this->get_logger(), "A goal is still active, reject new goal");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
    }

    // Validate new goal
    if (!goal->start_docking) 
    {
        RCLCPP_INFO(this->get_logger(), "Invalid request, reject goal");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // Accept goal
    RCLCPP_INFO(this->get_logger(), "Accept goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse DockingServerNode::cancel_callback(
    const std::shared_ptr<DockingGoalHandle> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
}


void DockingServerNode::handle_accepted_callback(
    const std::shared_ptr<DockingGoalHandle> goal_handle)
{
    execute_goal(goal_handle);
}


void DockingServerNode::execute_goal(
    const std::shared_ptr<DockingGoalHandle> goal_handle)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        this->goal_handle_ = goal_handle;
    }

    bool start_docking = goal_handle->get_goal()->start_docking;

    auto result = std::make_shared<Docking::Result>();
    auto feedback = std::make_shared<Docking::Feedback>();

    RCLCPP_INFO(this->get_logger(), "Execute goal");

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) 
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle->is_canceling()) 
            {
                result->docking_completed = is_docking_completed_;
                if(y_offset_ == 0.01)
                {
                    result->message = "success";
                    goal_handle->succeed(result);
                }
                else
                {
                    result->message = "Goal is cancelled";
                    goal_handle->canceled(result);
                }
                start_docking = false;
                return;
            }
        }
        // Check if cancel request
        if (goal_handle->is_canceling()) 
        {
            result->docking_completed = is_docking_completed_;
            x_offset_ = 5.0;
            y_offset_ = 5.0;
            if (is_docking_completed_) 
            {
                result->message = "Success";
                goal_handle->succeed(result); 
            }
            else 
            {
                result->message = "Canceled";
                goal_handle->canceled(result);
            }
            return;
        }

        switch (state_)
        {
        case State::INIT_DOCKING:
            RCLCPP_INFO(get_logger(), "Init Docking");
            x_offset_ = 0.0;
            y_offset_ = 0.0;
            theta_offset_ = 0.0;
            if(send_nav2_goal())
            {
                state_ = State::MOVE_TO_PREPOSE;
            }
            break;

        case State::MOVE_TO_PREPOSE:
            // RCLCPP_INFO(get_logger(), "Move to Prepose");
            if(is_pre_pose_goal_reached_)
            {
                is_pre_pose_goal_reached_= false;
                state_ = State::LINEAR_MOVEMENT;
            }
            break;

        case State::LINEAR_MOVEMENT:
            // RCLCPP_INFO(get_logger(), "Linear Movement");
            publish_cmd_vel(0.02, 0.0);
            if(y_offset_ <= 0.03)
            {   
                publish_cmd_vel(0.0, 0.0);
                state_ = State::ANGULAR_MOVEMENT;
            }
            break;

        case State::ANGULAR_MOVEMENT:
            RCLCPP_INFO(get_logger(), "Angular Movement");
            state_ = State::GO_TO_DOCK;
            break;

        case State::GO_TO_DOCK: 
            RCLCPP_INFO(get_logger(), "Go to Dock");
            state_ = State::INIT_DOCKING;
            result->message =  "docking done";
            result->docking_completed = true;
            goal_handle->succeed(result);
            return;

        default:
            break;
        }

        // Send the feedback
        feedback->offset_x = x_offset_;
        feedback->offset_y = y_offset_;
        feedback->offset_theta = theta_offset_;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
    }
}


bool DockingServerNode::send_nav2_goal()
{
    if (!nav2_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return false;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = -1.227;
    goal_msg.pose.pose.position.y = -0.1979;

    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 0.707;
    goal_msg.pose.pose.orientation.w = 0.707;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&DockingServerNode::result_callback, this, _1);

    nav2_action_client_->async_send_goal(goal_msg, send_goal_options);
    return true;
}


void DockingServerNode::result_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Pre pose Goal succeeded!");
            is_pre_pose_goal_reached_ = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Pre pose Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Pre pose Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }
}

void DockingServerNode::publish_cmd_vel(double linear, double angular)
{
    auto cmd_vel_msg = geometry_msgs::msg::Twist();
    cmd_vel_msg.linear.x = linear;
    cmd_vel_msg.angular.z = angular;
    cmd_vel_publisher_->publish(cmd_vel_msg);
    RCLCPP_INFO(this->get_logger(), "Published cmd_vel: linear.x = %f, angular.z = %f", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
}

void DockingServerNode::timer_callback()
{
    try
    {
        // Look up the transform from /map to base_footprint
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            "map",  // Target frame
            "base_footprint", // Source frame
            tf2::TimePointZero // Time (latest available)
        );


        // Extract translation and rotation
        x_offset_ = abs(transform.transform.translation.x);
        y_offset_ = abs(transform.transform.translation.y);

        double roll, pitch, yaw;

        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        //Log the transform
        RCLCPP_INFO(this->get_logger(), "x: %f , y: %f", x_offset_, y_offset_);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DockingServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
