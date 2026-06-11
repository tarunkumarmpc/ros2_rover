/**
 * @file camera_lift_node.hpp
 * @brief Declaration of the CameraLiftNode class for ROS 2 camera lift control.
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32.hpp"
#include "can_msgs/msg/frame.hpp"
#include "rover_msgs/action/camera_lift.hpp"
#include "rover_msgs/msg/camera_lift_status.hpp"
#include <atomic>
#include <mutex>
#include <memory>

using CameraLiftAction = rover_msgs::action::CameraLift;
using GoalHandleCameraLift = rclcpp_action::ServerGoalHandle<CameraLiftAction>;

/**
 * @class CameraLiftNode
 * @brief ROS 2 node for controlling and monitoring a camera lift mechanism.
 *
 * Provides both action and topic interfaces for control, and publishes lift status.
 */
class CameraLiftNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor. Initializes parameters, publishers, subscribers, and action server.
     */
    CameraLiftNode();

private:
    // === Parameters ===

    uint32_t can_tx_id_;    ///< CAN TX ID for sending height commands
    uint32_t can_rx_id_;    ///< CAN RX ID for receiving feedback
    double max_height_;     ///< Maximum allowed height (meters)
    double tolerance_;      ///< Tolerance for goal achievement (meters)
    double control_rate_;   ///< Control loop rate (Hz)
    private:
    // Timeout-related members
    rclcpp::Duration goal_timeout_;
    rclcpp::Time goal_start_time_;

    // === ROS Interfaces ===

    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_sub_; ///< CAN RX subscriber
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_pub_;    ///< CAN TX publisher
    rclcpp::Publisher<rover_msgs::msg::CameraLiftStatus>::SharedPtr feedback_pub_; ///< Status publisher
    rclcpp_action::Server<CameraLiftAction>::SharedPtr action_server_; ///< Action server
    rclcpp::TimerBase::SharedPtr control_timer_;                       ///< Control loop timer
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr height_cmd_sub_; ///< Topic command subscriber

    // === State ===

    std::atomic<double> current_height_{0.0}; ///< Current lift height (meters)
    std::shared_ptr<GoalHandleCameraLift> active_goal_; ///< Currently active action goal
    std::mutex goal_mutex_;                             ///< Mutex for goal state

    // Topic command state
    std::atomic<bool> topic_goal_active_{false}; ///< Is a topic-based goal active?
    std::atomic<double> topic_goal_height_{0.0}; ///< Target height from topic (meters)

    // === Internal Helpers ===

    /**
     * @brief Declare all configurable parameters with default values.
     */
    void declareParameters();

    /**
     * @brief Load parameters from the parameter server.
     */
    void loadParameters();

    /**
     * @brief Validate and correct parameters if necessary.
     */
    void validateParameters();

    /**
     * @brief Action server goal callback.
     * @param uuid Goal UUID.
     * @param goal Goal message.
     * @return GoalResponse indicating acceptance or rejection.
     */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const CameraLiftAction::Goal>);

    /**
     * @brief Action server cancel callback.
     * @param goal_handle Handle to the goal being canceled.
     * @return CancelResponse indicating acceptance.
     */
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCameraLift>);

    /**
     * @brief Action server goal acceptance callback.
     * @param goal_handle Handle to the accepted goal.
     */
    void handle_accepted(const std::shared_ptr<GoalHandleCameraLift>);

    /**
     * @brief Send a CAN command to move the camera lift to the specified height.
     * @param height_m Target height in meters.
     */
    void send_height_command(double height_m);

    /**
     * @brief CAN RX callback. Updates current height and publishes status.
     * @param msg CAN frame message.
     */
    void canRxCallback(const can_msgs::msg::Frame::SharedPtr msg);

    /**
     * @brief Topic subscriber callback for manual/slider control.
     * @param msg Float32 message with target height (meters).
     */
    void topic_height_callback(const std_msgs::msg::Float32::SharedPtr msg);

    /**
     * @brief Main control loop. Handles action and topic goals, sends commands, and publishes feedback.
     */
    void control_loop();
};
