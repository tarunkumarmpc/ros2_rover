#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "can_msgs/msg/frame.hpp"
#include "rover_msgs/action/rover_lift.hpp"
#include "rover_msgs/msg/rover_lift_status.hpp"
#include "rover_msgs/msg/lift_data.hpp"
#include "std_msgs/msg/float64.hpp"
#include <mutex>
#include <array>
#include <atomic>
#include <thread>

using RoverLiftAction = rover_msgs::action::RoverLift;
using GoalHandleRoverLift = rclcpp_action::ServerGoalHandle<RoverLiftAction>;

/**
 * @brief RAII helper to automatically set and clear a boolean flag.
 *
 * Used to manage the state of command_in_progress_ in a thread-safe way.
 */
class GoalActiveGuard {
public:
    /**
     * @brief Constructor. Sets the referenced flag to true.
     * @param flag Reference to the atomic flag to manage.
     */
    explicit GoalActiveGuard(std::atomic<bool>& flag);

    /**
     * @brief Destructor. Sets the referenced flag to false.
     */
    ~GoalActiveGuard();

private:
    std::atomic<bool>& flag_; ///< Reference to the atomic flag being managed.
};

/**
 * @brief ROS2 node for controlling and monitoring a rover's lift via CAN bus.
 *
 * This node provides an action server interface for lift commands, subscribes to CAN bus messages,
 * and publishes status updates. It manages concurrency, parameter validation, and feedback.
 */
class RoverLiftNode : public rclcpp::Node {
public:
    /**
     * @brief Construct the RoverLiftNode and initialize all parameters, publishers, and subscriptions.
     */
    RoverLiftNode();

private:
    // --- Constants ---
    static constexpr size_t NUM_LIFTS = 4; ///< Number of independent lift mechanisms supported.

    // --- Parameters ---
    uint32_t can_tx_id_;         ///< CAN bus transmit ID for sending commands to the lift controller.
    uint32_t can_rx_base_id_;    ///< Base CAN bus receive ID for receiving lift state feedback.
    double max_height_;          ///< Maximum allowable lift height (meters).
    double min_height_;          ///< Minimum allowable lift height (meters).
    double goal_timeout_;    ///< Timeout for reaching a target height (seconds).
    double height_tolerance_;    ///< Acceptable error for reaching the target height (meters).
    double default_speed_;       ///< Default lift movement speed (mm/s).

    // --- ROS interfaces ---
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_pub_; ///< Publishes CAN frames to the bus.
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_sub_; ///< Subscribes to CAN frames from the bus.
    rclcpp::Publisher<rover_msgs::msg::RoverLiftStatus>::SharedPtr lift_status_pub_; ///< Publishes lift status messages.
    rclcpp_action::Server<RoverLiftAction>::SharedPtr action_server_; ///< Action server for lift commands.
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lift_command_sub_; ///< Subscribes to direct height commands (e.g., from a UI slider).

    // --- State ---
    std::array<double, NUM_LIFTS> lift_heights_; ///< Most recent measured heights for each lift (meters).
    std::array<rclcpp::Time, NUM_LIFTS> lift_last_update_times_; ///< Time of last update for each lift.
    std::mutex height_mutex_; ///< Mutex to protect access to lift_heights_.
    std::mutex last_update_mutex_; ///< Mutex to protect access to lift_last_update_times_.

    // --- Command management ---
    std::atomic<bool> command_in_progress_{false}; ///< Indicates if a lift command is currently being executed.
    double last_commanded_height_{0.0}; ///< The most recent target height commanded.
    std::mutex command_mutex_; ///< Protects command state variables.

    // --- Parameter helpers ---

    /**
     * @brief Declare all configurable ROS2 parameters with default values.
     */
    void declareParameters();

    /**
     * @brief Load parameters from the ROS2 parameter server.
     */
    void loadParameters();

    /**
     * @brief Validate loaded parameters for correctness.
     * @throws std::runtime_error if any parameter is invalid.
     */
    void validateParameters();

    // --- Action server callbacks ---

    /**
     * @brief Callback to handle incoming action goals.
     * @param uuid The unique identifier for the goal.
     * @param goal The goal message.
     * @return Goal response (accept or reject).
     */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const RoverLiftAction::Goal> goal);

    /**
     * @brief Callback to handle action goal cancellation requests.
     * @param goal_handle The handle to the goal.
     * @return Cancel response (accept or reject).
     */
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleRoverLift>);

    /**
     * @brief Callback when a new goal is accepted. Launches execution in a new thread.
     * @param goal_handle The handle to the accepted goal.
     */
    void handle_accepted(const std::shared_ptr<GoalHandleRoverLift> goal_handle);

    /**
     * @brief The main execution loop for a lift action goal.
     * @param goal_handle The handle to the action goal.
     */
    void execute(const std::shared_ptr<GoalHandleRoverLift> goal_handle);

    // --- CAN and status ---

    /**
     * @brief Compute the average height across all lifts.
     * @return The average height in meters.
     */
    double getAverageHeight();

    /**
     * @brief Callback for incoming CAN bus messages.
     * @param msg The received CAN frame.
     */
    void canRxCallback(const can_msgs::msg::Frame::SharedPtr msg);

    /**
     * @brief Publish the current lift status to the appropriate topic.
     */
    void publishLiftStatus();

    // --- Topic-based UI/slider control ---

    /**
     * @brief Handle direct height commands from a topic (e.g., UI slider).
     * @param height The target lift height in meters.
     */
    void handleLiftCommand(double height);

    // --- Utility ---

    /**
     * @brief Check if a new command can be accepted (i.e., no command in progress or the last command is finished).
     * @return True if a new command can be accepted.
     */
    bool canAcceptNewCommand();

    /**
     * @brief Mark the command as finished if the target height has been reached.
     */
    void finishCommandIfArrived();

    void sendLiftCommand(double height);  
};
