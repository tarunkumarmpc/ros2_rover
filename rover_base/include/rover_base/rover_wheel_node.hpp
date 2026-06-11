/**
 * @file rover_wheel_node.hpp
 * @brief ROS 2 node for mecanum rover base control with CAN bus, velocity smoothing, odometry, and safety services.
 *
 * This node provides:
 * - /cmd_vel subscriber with velocity smoothing
 * - CAN bus communication for wheel velocities and enable/disable
 * - Odometry publishing and TF
 * - /enable_cmd_vel, /hard_stop, /estop, /reset_odometry services
 * - Professional safety logic (E-stop lockout, smooth stopping, thread safety)
 *   - E-stop logic to immediately stop all motion
 *   - Smooth stopping to prevent sudden jerks
 *   
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <array>
#include <mutex>
#include <memory>

/**
 * @class MecanumKinematics
 * @brief Provides forward and inverse kinematics for a mecanum drive robot.
 */
class MecanumKinematics {
public:
    MecanumKinematics(double wheel_radius, double lx, double ly);
    std::array<double, 4> inverse(const std::array<double, 3>& cmd_vel) const;
    std::array<double, 3> forward(const std::array<double, 4>& wheel_vel) const;
private:
    double wheel_radius_, lx_, ly_, lx_plus_ly_;
};

/**
 * @class VelocitySmoother
 * @brief Provides cubic smoothing for velocity transitions (accel/decel).
 */
class VelocitySmoother {
public:
    VelocitySmoother(double accel_duration, double decel_duration);
    static double smoothStep(double start, double end, double t, double duration);
    std::array<double, 3> smooth(const std::array<double, 3>& current,
                                 const std::array<double, 3>& target,
                                 double elapsed,
                                 bool is_stopping) const;
private:
    double accel_duration_;
    double decel_duration_;
};

/**
 * @class RoverWheelNode
 * @brief Main ROS 2 node for mecanum rover base control.
 */
class RoverWheelNode : public rclcpp::Node {
public:
    RoverWheelNode();
    ~RoverWheelNode();

private:
    // Parameters
    double wheel_radius_;
    double lx_;
    double ly_;
    double wheel_vel_scale_;
    bool use_vel_smoothing_;
    double accel_smooth_duration_;
    double decel_smooth_duration_;
    double cmd_vel_timeout_;
    double vel_threshold_;
    uint32_t can_id_wheel_vel_;
    uint32_t can_id_enable_;
    uint32_t can_id_feedback_;
    double control_rate_;   ///< Control loop rate (Hz)


    // Kinematics and smoothing
    std::unique_ptr<MecanumKinematics> kinematics_;
    std::unique_ptr<VelocitySmoother> smoother_;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_sub_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_odom_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_cmd_vel_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr estop_srv_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr enable_timer_;

    // State
    std::array<double, 3> current_cmd_vel_;
    std::array<double, 3> target_cmd_vel_;
    double x_, y_, yaw_;
    rclcpp::Time last_time_;
    rclcpp::Time last_smooth_time_;
    rclcpp::Time last_cmd_vel_time_;
    bool wheels_enabled_;
    bool is_stopping_;
    bool cmd_vel_enabled_;
    bool estop_active_;
    std::mutex state_mutex_;

    // Parameter handling
    void declareParameters();
    void loadParameters();
    void validateParameters();

    // CAN and wheel control
    void sendWheelVelocities(const std::array<double, 4>& wheel_ang_vel);
    void sendWheelEnable(bool enable);

    // Odometry
    void publishOdometry(const rclcpp::Time& stamp, double vx, double vy, double wz);

    // Callbacks
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void enableCmdVelCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> res);
    void hardStopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void resetOdometryCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void estopCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                       std::shared_ptr<std_srvs::srv::SetBool::Response> res);
    void canRxCallback(const can_msgs::msg::Frame::SharedPtr msg);
    void update();
};

