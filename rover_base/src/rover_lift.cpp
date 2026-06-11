/**
 * @file rover_lift_node.cpp
 * @brief ROS2 node for rover lift control via CAN bus.
 */
#include "rover_base/rover_lift_node.hpp"
#include <cmath>
#include <sstream>

using namespace std::placeholders;

GoalActiveGuard::GoalActiveGuard(std::atomic<bool>& flag) : flag_(flag) { flag_ = true; }
GoalActiveGuard::~GoalActiveGuard() { flag_ = false; }

RoverLiftNode::RoverLiftNode() : Node("rover_lift_action_server") {
    declareParameters();
    loadParameters();
    validateParameters();

    RCLCPP_INFO(this->get_logger(), "Parameter 'height_tolerance' loaded with value: %f", height_tolerance_);
    RCLCPP_INFO(this->get_logger(), "Node '%s' has been initialized.", this->get_name());

    auto now = this->now();
    for (size_t i = 0; i < NUM_LIFTS; ++i) {
        lift_heights_[i] = 0.0;
        lift_last_update_times_[i] = now;
    }

    can_tx_pub_ = this->create_publisher<can_msgs::msg::Frame>("/canbus/tx", 10);
    lift_status_pub_ = this->create_publisher<rover_msgs::msg::RoverLiftStatus>("rover_lift/status", 10);

    can_rx_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "/canbus/rx", 10,
        [this](const can_msgs::msg::Frame::SharedPtr msg) {
            this->canRxCallback(msg);
        });

    lift_command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "rover_lift/target_height", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
            this->handleLiftCommand(msg->data);
        });

    action_server_ = rclcpp_action::create_server<RoverLiftAction>(
        this,
        "rover_lift",
        std::bind(&RoverLiftNode::handle_goal, this, _1, _2),
        std::bind(&RoverLiftNode::handle_cancel, this, _1),
        std::bind(&RoverLiftNode::handle_accepted, this, _1));
}

void RoverLiftNode::declareParameters() {
    this->declare_parameter("can_tx_id", 0x110);
    this->declare_parameter("can_rx_base_id", 0x210);
    this->declare_parameter("max_height", 0.16);
    this->declare_parameter("min_height", 0.0);
    this->declare_parameter("goal_timeout", 30.0);         // Renamed from movement_timeout
    this->declare_parameter("height_tolerance", 0.0001);
    this->declare_parameter("default_speed", 10.0);
}

void RoverLiftNode::loadParameters() {
    this->get_parameter("can_tx_id", can_tx_id_);
    this->get_parameter("can_rx_base_id", can_rx_base_id_);
    this->get_parameter("max_height", max_height_);
    this->get_parameter("min_height", min_height_);
    this->get_parameter("goal_timeout", goal_timeout_);     // Renamed from movement_timeout
    this->get_parameter("height_tolerance", height_tolerance_);
    this->get_parameter("default_speed", default_speed_);
}

void RoverLiftNode::validateParameters() {
    if (max_height_ <= min_height_) throw std::runtime_error("max_height must be > min_height");
    if (height_tolerance_ <= 0.0) throw std::runtime_error("height_tolerance must be positive");
    if (default_speed_ <= 0.0) throw std::runtime_error("default_speed must be positive");
    if (goal_timeout_ <= 0.0) throw std::runtime_error("goal_timeout must be positive");  // Updated message
}

bool RoverLiftNode::canAcceptNewCommand() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    double current_avg = getAverageHeight();
    return !command_in_progress_ || std::abs(current_avg - last_commanded_height_) <= height_tolerance_;
}

void RoverLiftNode::finishCommandIfArrived() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    double current_avg = getAverageHeight();
    if (command_in_progress_ && std::abs(current_avg - last_commanded_height_) <= height_tolerance_) {
        command_in_progress_ = false;
    }
}

rclcpp_action::GoalResponse RoverLiftNode::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const RoverLiftAction::Goal> goal) {
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RoverLiftNode::handle_cancel(const std::shared_ptr<GoalHandleRoverLift>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RoverLiftNode::handle_accepted(const std::shared_ptr<GoalHandleRoverLift> goal_handle) {
    std::thread{&RoverLiftNode::execute, this, goal_handle}.detach();
}

void RoverLiftNode::execute(const std::shared_ptr<GoalHandleRoverLift> goal_handle) {
    auto result = std::make_shared<RoverLiftAction::Result>();
    auto feedback = std::make_shared<RoverLiftAction::Feedback>();
    double target_height = goal_handle->get_goal()->height;

    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        double current_avg = getAverageHeight();
        if (command_in_progress_ && std::abs(current_avg - last_commanded_height_) > height_tolerance_) {
            result->success = false;
            result->message = "Lift is still moving to previous commanded height. Try again later.";
            goal_handle->abort(result);
            return;
        }

        if (target_height < min_height_ || target_height > max_height_) {
            result->success = false;
            std::ostringstream oss;
            oss << "Requested height " << target_height << " out of range ("
                << min_height_ << " < height < " << max_height_ << ")";
            result->message = oss.str();
            goal_handle->abort(result);
            return;
        }

        command_in_progress_ = true;
        last_commanded_height_ = target_height;
    }

    GoalActiveGuard guard(command_in_progress_);

    sendLiftCommand(target_height);

    rclcpp::Rate loop_rate(10);
    auto start_time = this->now();

    while (rclcpp::ok() && !goal_handle->is_canceling()) {
        double current_avg = getAverageHeight();
        double error = std::abs(current_avg - target_height);
        feedback->current_height = current_avg;
        goal_handle->publish_feedback(feedback);

        if (error <= height_tolerance_) {
            result->success = true;
            result->message = "Target height reached.";
            goal_handle->succeed(result);
            finishCommandIfArrived();
            return;
        }

        if ((this->now() - start_time).seconds() > goal_timeout_) {  // Uses goal_timeout_
            result->success = false;
            result->message = "Goal timeout reached.";
            goal_handle->abort(result);
            finishCommandIfArrived();
            return;
        }

        loop_rate.sleep();
    }

    if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Goal canceled by client.";
        goal_handle->canceled(result);
        finishCommandIfArrived();
    }
}

void RoverLiftNode::handleLiftCommand(double height) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    double current_avg = getAverageHeight();

    if (command_in_progress_ && std::abs(current_avg - last_commanded_height_) > height_tolerance_) {
        //RCLCPP_WARN(this->get_logger(), "Lift is still moving to previous commanded height. Ignoring new topic command.");
        return;
    }

    if (height < min_height_ || height > max_height_) {
        RCLCPP_WARN(this->get_logger(), "Topic command out of range: %.3f m (%.3f-%.3f)", height, min_height_, max_height_);
        return;
    }

    command_in_progress_ = true;
    last_commanded_height_ = height;

    sendLiftCommand(height);
    RCLCPP_INFO(this->get_logger(), "Lift topic command: height=%.3f m", height);
}

void RoverLiftNode::sendLiftCommand(double target_height) {
    uint16_t height_mm = static_cast<uint16_t>(std::round(target_height * 1000.0));
    uint16_t speed_mmps = static_cast<uint16_t>(default_speed_);

    can_msgs::msg::Frame can_msg;
    can_msg.id = can_tx_id_;
    can_msg.is_extended = false;
    can_msg.is_rtr = false;
    can_msg.is_error = false;
    can_msg.dlc = 8;
    can_msg.data[0] = static_cast<uint8_t>(height_mm & 0xFF);
    can_msg.data[1] = static_cast<uint8_t>((height_mm >> 8) & 0xFF);
    can_msg.data[2] = static_cast<uint8_t>(speed_mmps & 0xFF);
    can_msg.data[3] = static_cast<uint8_t>((speed_mmps >> 8) & 0xFF);
    for (int i = 4; i < 8; ++i) can_msg.data[i] = 0x00;

    can_tx_pub_->publish(can_msg);

    RCLCPP_INFO(this->get_logger(), "Starting movement to height: %.3f m", target_height);
}

double RoverLiftNode::getAverageHeight() {
    std::lock_guard<std::mutex> lock(height_mutex_);
    double sum = 0.0;
    for (size_t i = 0; i < NUM_LIFTS; ++i) {
        double h = lift_heights_[i];
        if (h < 0.0) h = 0.0;
        sum += h;
    }
    return sum / static_cast<double>(NUM_LIFTS);
}

void RoverLiftNode::canRxCallback(const can_msgs::msg::Frame::SharedPtr msg) {
    if (msg->id < can_rx_base_id_ || msg->id >= can_rx_base_id_ + NUM_LIFTS) return;
    if (msg->dlc != 8) return;

    size_t index = msg->id - can_rx_base_id_;
    uint16_t height_hundredths_mm = (static_cast<uint16_t>(msg->data[1]) << 8) | msg->data[0];
    double height_m = static_cast<double>(height_hundredths_mm) * 0.00001;

    {
        std::lock_guard<std::mutex> lock(height_mutex_);
        lift_heights_[index] = height_m;
    }

    {
        std::lock_guard<std::mutex> lock(last_update_mutex_);
        lift_last_update_times_[index] = this->now();
    }

    publishLiftStatus();
    finishCommandIfArrived();
}

void RoverLiftNode::publishLiftStatus() {
    auto status_msg = std::make_unique<rover_msgs::msg::RoverLiftStatus>();
    auto now = this->now();

    for (size_t i = 0; i < NUM_LIFTS; ++i) {
        rover_msgs::msg::LiftData data;
        {
            std::lock_guard<std::mutex> lock(height_mutex_);
            double h = lift_heights_[i];
            data.height = h < 0.0 ? 0.0 : h;
        }
        {
            std::lock_guard<std::mutex> lock(last_update_mutex_);
            data.read_time_lapse = (now - lift_last_update_times_[i]).seconds();
        }
        data.id = static_cast<uint8_t>(i);
        status_msg->data.push_back(data);
    }

    lift_status_pub_->publish(std::move(status_msg));
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverLiftNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}