/**
 * @file camera_lift_node.cpp
 * @brief ROS 2 node for controlling a camera lift mechanism via CAN bus with timeout support.
 *
 * This node provides both action and topic interfaces to command the camera lift
 * to a specific height. It communicates with the hardware using CAN messages,
 * supports timeouts, and ensures mutual exclusion of goals.
 */
#include "rover_base/camera_lift_node.hpp"
#include <cmath>

CameraLiftNode::CameraLiftNode() : Node("rover_camera_lift"),
                                  goal_timeout_(rclcpp::Duration::from_seconds(0.0)),
                                  goal_start_time_(this->now())
{
    // Declare, load, and validate parameters
    declareParameters();
    loadParameters();
    validateParameters();

    RCLCPP_INFO(this->get_logger(), "Parameter 'height_tolerance' FOR CAMERA : %f", tolerance_);
    RCLCPP_INFO(this->get_logger(), "CAN TX ID: 0x%03X, CAN RX ID: 0x%03X",
                can_tx_id_, can_rx_id_);
    RCLCPP_INFO(this->get_logger(), "Goal timeout: %.1f seconds", goal_timeout_.seconds());

    // Subscribe to CAN RX messages (from hardware)
    can_rx_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "/canbus/rx", 10,
        std::bind(&CameraLiftNode::canRxCallback, this, std::placeholders::_1));

    // Publisher for CAN TX messages (to hardware)
    can_tx_pub_ = this->create_publisher<can_msgs::msg::Frame>("/canbus/tx", 10);

    // Publisher for camera lift status feedback
    feedback_pub_ = this->create_publisher<rover_msgs::msg::CameraLiftStatus>(
        "camera_lift/status", 10);

    // Action server for camera lift goals
    action_server_ = rclcpp_action::create_server<CameraLiftAction>(
        this,
        "camera_lift",
        std::bind(&CameraLiftNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CameraLiftNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&CameraLiftNode::handle_accepted, this, std::placeholders::_1)
    );

    // Topic subscriber for UI-based height commands
    height_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "camera_lift/target_height", 10,
        std::bind(&CameraLiftNode::topic_height_callback, this, std::placeholders::_1)
    );

    // Timer for the control loop (periodic feedback and command sending)
    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / control_rate_),
        std::bind(&CameraLiftNode::control_loop, this)
    );
}

void CameraLiftNode::declareParameters()
{
    this->declare_parameter("can_tx_id", 0x120);
    this->declare_parameter("can_rx_id", 0x220);
    this->declare_parameter("max_height", 0.21);
    this->declare_parameter("tolerance", 0.005);
    this->declare_parameter("control_rate", 10.0);
    this->declare_parameter("goal_timeout_sec", 30.0); // Add timeout param
}

void CameraLiftNode::loadParameters()
{
    int tx_id, rx_id;
    double goal_timeout_sec;

    this->get_parameter("can_tx_id", tx_id);
    this->get_parameter("can_rx_id", rx_id);
    this->get_parameter("max_height", max_height_);
    this->get_parameter("tolerance", tolerance_);
    this->get_parameter("control_rate", control_rate_);
    this->get_parameter("goal_timeout_sec", goal_timeout_sec);

    can_tx_id_ = static_cast<uint32_t>(tx_id);
    can_rx_id_ = static_cast<uint32_t>(rx_id);
    goal_timeout_ = rclcpp::Duration::from_seconds(goal_timeout_sec);
}

void CameraLiftNode::validateParameters()
{
    if (can_tx_id_ == 0 || can_tx_id_ > 0x7FF) {
        RCLCPP_WARN(this->get_logger(), "Invalid CAN TX ID: 0x%X. Reset to 0x120", can_tx_id_);
        can_tx_id_ = 0x120;
    }
    if (can_rx_id_ == 0 || can_rx_id_ > 0x7FF) {
        RCLCPP_WARN(this->get_logger(), "Invalid CAN RX ID: 0x%X. Reset to 0x220", can_rx_id_);
        can_rx_id_ = 0x220;
    }
    if (max_height_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid max_height: %.3f. Reset to 0.2", max_height_);
        max_height_ = 0.2;
    }
    if (tolerance_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid tolerance: %.3f. Reset to 0.005", tolerance_);
        tolerance_ = 0.005;
    }
    if (control_rate_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid control_rate: %.3f. Reset to 10.0", control_rate_);
        control_rate_ = 10.0;
    }
    if (goal_timeout_.seconds() <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid goal_timeout_sec: %.1f. Reset to 30.0", goal_timeout_.seconds());
        goal_timeout_ = rclcpp::Duration::from_seconds(30.0);
    }
}

rclcpp_action::GoalResponse CameraLiftNode::handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const CameraLiftAction::Goal> /*goal*/)
{
    goal_start_time_ = this->now(); // Set start time
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CameraLiftNode::handle_cancel(
    const std::shared_ptr<GoalHandleCameraLift>)
{
    RCLCPP_INFO(this->get_logger(), "Goal cancellation requested");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CameraLiftNode::handle_accepted(const std::shared_ptr<GoalHandleCameraLift> goal_handle)
{
    std::lock_guard<std::mutex> lock(goal_mutex_);
    double height = goal_handle->get_goal()->height;

    if (height < 0.0 || height > max_height_) {
        RCLCPP_ERROR(this->get_logger(), "Rejecting goal: height %.3fm invalid (0-%.3fm)", height, max_height_);
        auto result = std::make_shared<CameraLiftAction::Result>();
        result->success = false;
        result->message = "Requested height out of bounds";
        goal_handle->abort(result);
        return;
    }

    if (active_goal_ || topic_goal_active_) {
        RCLCPP_ERROR(this->get_logger(), "Active goal already exists! Rejecting new action goal");
        auto result = std::make_shared<CameraLiftAction::Result>();
        result->success = false;
        result->message = "Another goal is active";
        goal_handle->abort(result);
        return;
    }

    active_goal_ = goal_handle;
    send_height_command(height);
}

void CameraLiftNode::send_height_command(double height_m)
{
    uint16_t height_mm = static_cast<uint16_t>(std::round(height_m * 1000.0));
    can_msgs::msg::Frame msg;
    msg.id = can_tx_id_;
    msg.dlc = 8;
    for (int i = 0; i < 8; ++i) msg.data[i] = 0;
    msg.data[0] = static_cast<uint8_t>(height_mm & 0xFF);
    msg.data[1] = static_cast<uint8_t>((height_mm >> 8) & 0xFF);
    can_tx_pub_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "Sent height command: %u mm (0x%02X 0x%02X)",
                 height_mm, msg.data[0], msg.data[1]);
}

void CameraLiftNode::canRxCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
    if (msg->id != can_rx_id_) return;
    if (msg->dlc < 2) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Invalid CAN length: %d (expected >=2)", msg->dlc);
        return;
    }
    uint16_t height_mm = static_cast<uint16_t>(msg->data[0]) |
                         (static_cast<uint16_t>(msg->data[1]) << 8);
    current_height_.store(static_cast<double>(height_mm) / 1000.0);

    rover_msgs::msg::CameraLiftStatus status_msg;
    status_msg.height = static_cast<int64_t>(height_mm);
    feedback_pub_->publish(status_msg);
}

void CameraLiftNode::topic_height_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    double height = static_cast<double>(msg->data);
    if (height < 0.0 || height > max_height_) {
        RCLCPP_WARN(this->get_logger(), "Topic command rejected: height %.3fm invalid (0-%.3fm)", height, max_height_);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (active_goal_ || topic_goal_active_) {
            //RCLCPP_INFO(this->get_logger(), "Topic command ignored: another goal is active");
            return;
        }
        topic_goal_active_ = true;
        topic_goal_height_ = height;
        goal_start_time_ = this->now(); // Reuse same timestamp
    }

    RCLCPP_INFO(this->get_logger(), "Topic command: move to %.3fm", height);
    send_height_command(height);
}

void CameraLiftNode::control_loop()
{
    std::shared_ptr<GoalHandleCameraLift> current_goal;
    bool topic_active = false;
    double topic_target = 0.0;

    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        current_goal = active_goal_;
        topic_active = topic_goal_active_;
        topic_target = topic_goal_height_;
    }

    if (current_goal) {
        const double target = current_goal->get_goal()->height;
        const double current = current_height_.load();
        const double error = std::abs(current - target);

        // Check for timeout
        if ((this->now() - goal_start_time_) > goal_timeout_) {
            RCLCPP_WARN(this->get_logger(), "Goal timed out after %.1f seconds", goal_timeout_.seconds());
            auto result = std::make_shared<CameraLiftAction::Result>();
            result->success = false;
            result->message = "Goal timed out";
            current_goal->abort(result);
            std::lock_guard<std::mutex> lock(goal_mutex_);
            active_goal_.reset();
            return;
        }

        if (current_goal->is_canceling()) {
            RCLCPP_INFO(this->get_logger(), "Goal canceled at %.3fm", current);
            auto result = std::make_shared<CameraLiftAction::Result>();
            result->success = false;
            result->message = "Goal canceled by client";
            current_goal->canceled(result);
            std::lock_guard<std::mutex> lock(goal_mutex_);
            active_goal_.reset();
            return;
        }

        auto feedback = std::make_shared<CameraLiftAction::Feedback>();
        feedback->current_height = current;
        current_goal->publish_feedback(feedback);

        if (error < tolerance_) {
            RCLCPP_INFO(this->get_logger(), "Goal succeeded: Reached %.3fm (error=%.4fm)", target, error);
            auto result = std::make_shared<CameraLiftAction::Result>();
            result->success = true;
            result->message = "Target height reached";
            current_goal->succeed(result);
            std::lock_guard<std::mutex> lock(goal_mutex_);
            active_goal_.reset();
        } else {
            send_height_command(target);
        }
        return;
    }

    if (topic_active) {
        double current = current_height_.load();
        double error = std::abs(current - topic_target);

        if ((this->now() - goal_start_time_) > goal_timeout_) {
            RCLCPP_WARN(this->get_logger(), "Topic goal timed out after %.1f seconds", goal_timeout_.seconds());
            std::lock_guard<std::mutex> lock(goal_mutex_);
            topic_goal_active_ = false;
            return;
        }

        if (error < tolerance_) {
            {
                std::lock_guard<std::mutex> lock(goal_mutex_);
                topic_goal_active_ = false;
            }
            RCLCPP_INFO(this->get_logger(), "Topic goal reached: %.3fm", topic_target);
        } else {
            send_height_command(topic_target);
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraLiftNode>());
    rclcpp::shutdown();
    return 0;
}