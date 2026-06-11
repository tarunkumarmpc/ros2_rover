/**
 * @file rover_wheel_node.cpp
 * @brief Implementation of RoverWheelNode for mecanum rover base control.
 *
 */

#include "rover_base/rover_wheel_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

// ==================== MecanumKinematics Implementation ====================

// Constructor: Initializes kinematics with wheel radius and robot dimensions
MecanumKinematics::MecanumKinematics(double wheel_radius, double lx, double ly)
    : wheel_radius_(wheel_radius), lx_(lx), ly_(ly), lx_plus_ly_(lx + ly)
{
    // Validate input parameters
    if (wheel_radius <= 0.0 || lx <= 0.0 || ly <= 0.0)
    {
        throw std::invalid_argument("MecanumKinematics: wheel_radius, lx, ly must be positive");
    }
}

// Converts robot velocity commands (vx, vy, wz) to individual wheel angular velocities
std::array<double, 4> MecanumKinematics::inverse(const std::array<double, 3> &cmd_vel) const
{
    // std::cout << "[Inverse] Input cmd_vel: vx=" << cmd_vel[0]
    //           << ", vy=" << cmd_vel[1]
    //           << ", wz=" << cmd_vel[2] << std::endl;

    const double denom = wheel_radius_;
    const double term = lx_plus_ly_ * cmd_vel[2];
    std::array<double, 4> wheel_speeds = {
        (cmd_vel[0] - cmd_vel[1] - term) / denom,
        (cmd_vel[0] + cmd_vel[1] + term) / denom,
        (cmd_vel[0] + cmd_vel[1] - term) / denom,
        (cmd_vel[0] - cmd_vel[1] + term) / denom};

    // std::cout << "[Inverse] Output wheel_speeds: "
    //           << "w0=" << wheel_speeds[0] << ", "
    //           << "w1=" << wheel_speeds[1] << ", "
    //           << "w2=" << wheel_speeds[2] << ", "
    //           << "w3=" << wheel_speeds[3] << std::endl;

    return wheel_speeds;
}

// Converts wheel angular velocities to robot velocity (vx, vy, wz)
std::array<double, 3> MecanumKinematics::forward(const std::array<double, 4> &wheel_vel) const
{
    double vx = (wheel_vel[0] + wheel_vel[1] + wheel_vel[2] + wheel_vel[3]) * wheel_radius_ / 4.0;
    double vy = (-wheel_vel[0] + wheel_vel[1] + wheel_vel[2] - wheel_vel[3]) * wheel_radius_ / 4.0;
    double wz = (-wheel_vel[0] + wheel_vel[1] - wheel_vel[2] + wheel_vel[3]) *
                (wheel_radius_ / (4.0 * lx_plus_ly_));
    return {vx, vy, wz};
}

// ==================== VelocitySmoother Implementation ====================

// Constructor: Sets acceleration and deceleration durations for smoothing
VelocitySmoother::VelocitySmoother(double accel_duration, double decel_duration)
    : accel_duration_(accel_duration), decel_duration_(decel_duration)
{
    if (accel_duration < 0.0 || decel_duration < 0.0)
    {
        throw std::invalid_argument("VelocitySmoother: durations must be non-negative");
    }
}

// Smoothly interpolates between start and end values over a duration using a cubic curve
double VelocitySmoother::smoothStep(double start, double end, double t, double duration)
{
    if (duration <= 0.0)
        return end;
    t = std::clamp(t / duration, 0.0, 1.0);
    return start + (end - start) * (3.0 * t * t - 2.0 * t * t * t);
}

// Smooths each velocity component (vx, vy, wz) based on acceleration/deceleration
std::array<double, 3> VelocitySmoother::smooth(const std::array<double, 3> &current,
                                               const std::array<double, 3> &target,
                                               double elapsed,
                                               bool is_stopping) const
{
    std::array<double, 3> result;
    for (size_t i = 0; i < 3; ++i)
    {
        double duration = is_stopping || std::abs(target[i]) <= std::abs(current[i])
                              ? decel_duration_
                              : accel_duration_;
        result[i] = smoothStep(current[i], target[i], elapsed, duration);
    }
    return result;
}

// ==================== RoverWheelNode Implementation ====================

// Constructor: Initializes node, parameters, publishers, subscribers, and timers
RoverWheelNode::RoverWheelNode() : Node("rover_wheel_node")
{
    declareParameters();
    loadParameters();
    validateParameters();
    RCLCPP_INFO(this->get_logger(), "accel_smooth_duration_ = %.3f", accel_smooth_duration_);
    RCLCPP_INFO(this->get_logger(), "use_vel_smoothing_ = %s", use_vel_smoothing_ ? "true" : "false");

    // Initialize kinematics and velocity smoother
    kinematics_ = std::make_unique<MecanumKinematics>(wheel_radius_, lx_, ly_);
    smoother_ = std::make_unique<VelocitySmoother>(accel_smooth_duration_, decel_smooth_duration_);

    // ROS2 topic subscriptions
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&RoverWheelNode::cmdVelCallback, this, std::placeholders::_1));

    can_rx_sub_ = create_subscription<can_msgs::msg::Frame>(
        "/canbus/rx", 10,
        std::bind(&RoverWheelNode::canRxCallback, this, std::placeholders::_1));

    // ROS2 topic publishers
    can_tx_pub_ = create_publisher<can_msgs::msg::Frame>("/canbus/tx", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // ROS2 services for odometry reset, cmd_vel enable, and emergency stop
    reset_odom_srv_ = create_service<std_srvs::srv::Trigger>(
        "/reset_odometry",
        std::bind(&RoverWheelNode::resetOdometryCallback, this, std::placeholders::_1, std::placeholders::_2));

    enable_cmd_vel_srv_ = create_service<std_srvs::srv::SetBool>(
        "/enable_cmd_vel",
        std::bind(&RoverWheelNode::enableCmdVelCallback, this, std::placeholders::_1, std::placeholders::_2));

    estop_srv_ = create_service<std_srvs::srv::SetBool>(
        "/estop",
        std::bind(&RoverWheelNode::estopCallback, this, std::placeholders::_1, std::placeholders::_2));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize state variables
    current_cmd_vel_ = {0.0, 0.0, 0.0};
    target_cmd_vel_ = {0.0, 0.0, 0.0};
    x_ = y_ = yaw_ = 0.0;
    last_time_ = now();
    last_smooth_time_ = last_time_;
    last_cmd_vel_time_ = last_time_;
    wheels_enabled_ = true;
    is_stopping_ = false;
    cmd_vel_enabled_ = true;
    estop_active_ = false;

    // Main control loop timer (10 ms)
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / control_rate_),
        std::bind(&RoverWheelNode::update, this));

    // Timer to enable motors at startup (runs once)
    enable_timer_ = create_wall_timer(
        std::chrono::milliseconds(300),
        [this]()
        {
            sendWheelEnable(true);
            RCLCPP_INFO(get_logger(), "Motors ENABLED (startup timer)");
            enable_timer_->cancel();
        });
}

// Destructor: Disables motors on shutdown
RoverWheelNode::~RoverWheelNode()
{
    sendWheelEnable(false);
}

// Declare all configurable ROS2 parameters
void RoverWheelNode::declareParameters()
{
    this->declare_parameter<double>("wheel_radius", 0.05); // 50 mm
    // lx: half the rover width (607 mm / 2 = 303.5 mm)
    // ly: half the rover length (322 mm / 2 = 161 mm)
    this->declare_parameter<double>("lx", 0.3035);        // 303.5 mm
    this->declare_parameter<double>("ly", 0.161);         // 161 mm
    this->declare_parameter<double>("wheel_vel_scale", 1000.0);
    this->declare_parameter<bool>("use_vel_smoothing", true);
    this->declare_parameter<double>("accel_smooth_duration", 0.3);
    this->declare_parameter<double>("decel_smooth_duration", 0.3);
    this->declare_parameter<double>("cmd_vel_timeout", 0.5);
    this->declare_parameter<double>("vel_threshold", 0.0005);
    this->declare_parameter<int64_t>("can_id_wheel_vel", 0x100);
    this->declare_parameter<int64_t>("can_id_enable", 0x101);
    this->declare_parameter<int64_t>("can_id_feedback", 0x200);
    this->declare_parameter<double>("control_rate", 10.0);
}

// Load parameters from the ROS2 parameter server
void RoverWheelNode::loadParameters()
{
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    lx_ = this->get_parameter("lx").as_double();
    ly_ = this->get_parameter("ly").as_double();
    wheel_vel_scale_ = this->get_parameter("wheel_vel_scale").as_double();
    use_vel_smoothing_ = this->get_parameter("use_vel_smoothing").as_bool();
    accel_smooth_duration_ = this->get_parameter("accel_smooth_duration").as_double();
    decel_smooth_duration_ = this->get_parameter("decel_smooth_duration").as_double();
    cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
    vel_threshold_ = this->get_parameter("vel_threshold").as_double();
    can_id_wheel_vel_ = static_cast<uint32_t>(this->get_parameter("can_id_wheel_vel").as_int());
    can_id_enable_ = static_cast<uint32_t>(this->get_parameter("can_id_enable").as_int());
    can_id_feedback_ = static_cast<uint32_t>(this->get_parameter("can_id_feedback").as_int());
    control_rate_ = this->get_parameter("control_rate").as_double(); // Hz
}

// Validate loaded parameters for correctness
void RoverWheelNode::validateParameters()
{
    if (wheel_radius_ <= 0.0 || lx_ <= 0.0 || ly_ <= 0.0 || wheel_vel_scale_ <= 0.0)
    {
        throw std::runtime_error("Invalid physical parameters");
    }

    if (accel_smooth_duration_ < 0.0 || decel_smooth_duration_ < 0.0 ||
        cmd_vel_timeout_ < 0.0 || vel_threshold_ < 0.0)
    {
        throw std::runtime_error("Invalid timing parameters");
    }

    if (can_id_wheel_vel_ == can_id_enable_ || can_id_wheel_vel_ == can_id_feedback_ ||
        can_id_enable_ == can_id_feedback_)
    {
        throw std::runtime_error("Duplicate CAN IDs");
    }
}

// Publishes wheel angular velocities to the CAN bus
void RoverWheelNode::sendWheelVelocities(const std::array<double, 4> &wheel_ang_vel)
{
    if (!can_tx_pub_)
        return;

    can_msgs::msg::Frame msg;
    msg.id = can_id_wheel_vel_;
    msg.is_extended = false;
    msg.is_rtr = false;
    msg.is_error = false;
    msg.dlc = 8;

    // Pack each wheel velocity as int16_t into CAN frame
    for (size_t i = 0; i < 4; ++i)
    {
        int16_t val = static_cast<int16_t>(std::round(wheel_ang_vel[i] * wheel_vel_scale_));
        msg.data[2 * i] = static_cast<uint8_t>(val & 0xFF);
        msg.data[2 * i + 1] = static_cast<uint8_t>((val >> 8) & 0xFF);
    }

    can_tx_pub_->publish(msg);
}

// Sends enable/disable command to the motors via CAN
void RoverWheelNode::sendWheelEnable(bool enable)
{
    if (!can_tx_pub_)
        return;

    can_msgs::msg::Frame msg;
    msg.id = can_id_enable_;
    msg.is_extended = false;
    msg.is_rtr = false;
    msg.is_error = false;
    msg.dlc = 1;
    msg.data[0] = enable ? 0x00 : 0x01;
    can_tx_pub_->publish(msg);
    wheels_enabled_ = enable;
}

// Publishes odometry and TF transform based on current pose and velocities
void RoverWheelNode::publishOdometry(const rclcpp::Time &stamp, double vx, double vy, double wz)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = wz;

    odom_pub_->publish(odom_msg);

    // Broadcast TF transform
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";

    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(transform);
}

// Callback for incoming cmd_vel messages
void RoverWheelNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (estop_active_ || !cmd_vel_enabled_)
    {
        return; // Ignore new cmd_vel when E-stop active or disabled
    }

    // Update target velocity and reset timers
    target_cmd_vel_ = {msg->linear.x, msg->linear.y, msg->angular.z};
    last_smooth_time_ = now();
    last_cmd_vel_time_ = now();
    is_stopping_ = false;

    // Enable wheels if not already enabled
    if (!wheels_enabled_)
    {
        sendWheelEnable(true);
    }

    RCLCPP_DEBUG(get_logger(), "Received cmd_vel: vx=%.3f, vy=%.3f, wz=%.3f",
                 msg->linear.x, msg->linear.y, msg->angular.z);
}

// Service callback to enable or disable cmd_vel control
void RoverWheelNode::enableCmdVelCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (req->data)
    {
        // Enable cmd_vel if possible
        if (estop_active_ || !wheels_enabled_)
        {
            res->success = false;
            res->message = "Cannot enable cmd_vel: E-stop active or motors are disabled.";
            RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
            return;
        }
        if (cmd_vel_enabled_)
        {
            res->success = true;
            res->message = "cmd_vel already enabled";
            return;
        }
        cmd_vel_enabled_ = true;
        last_cmd_vel_time_ = now();
        res->success = true;
        res->message = "cmd_vel enabled";
        RCLCPP_INFO(get_logger(), "cmd_vel enabled");
    }
    else
    {
        // Disable cmd_vel and initiate smooth stop
        if (!cmd_vel_enabled_)
        {
            res->success = true;
            res->message = "cmd_vel already disabled";
            return;
        }
        cmd_vel_enabled_ = false;
        target_cmd_vel_ = {0.0, 0.0, 0.0};
        is_stopping_ = true;
        res->success = true;
        res->message = "cmd_vel disabled: smooth stop initiated";
        RCLCPP_INFO(get_logger(), "cmd_vel disabled: smooth stop initiated");
    }
}

// Service callback to reset odometry to zero
void RoverWheelNode::resetOdometryCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    x_ = y_ = yaw_ = 0.0;
    res->success = true;
    res->message = "Odometry reset successfully";
}

// Service callback for emergency stop (E-STOP)
void RoverWheelNode::estopCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    sendWheelEnable(req->data);
    estop_active_ = !req->data;
    current_cmd_vel_ = {0.0, 0.0, 0.0};
    target_cmd_vel_ = {0.0, 0.0, 0.0};
    auto wheel_vel = kinematics_->inverse(current_cmd_vel_);
    sendWheelVelocities(wheel_vel);
    res->success = true;
    res->message = req->data ? "Motors ENABLED" : "Motors DISABLED (E-STOP)";
}

// Callback for incoming CAN feedback messages (wheel velocities)
void RoverWheelNode::canRxCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
    if (msg->id != can_id_feedback_ || msg->dlc != 8)
        return;

    // Unpack wheel velocities from CAN frame
    int16_t w0 = static_cast<int16_t>(msg->data[0] | (msg->data[1] << 8));
    int16_t w1 = static_cast<int16_t>(msg->data[2] | (msg->data[3] << 8));
    int16_t w2 = static_cast<int16_t>(msg->data[4] | (msg->data[5] << 8));
    int16_t w3 = static_cast<int16_t>(msg->data[6] | (msg->data[7] << 8));

    std::array<double, 4> wheel_vel = {
        static_cast<double>(w0) / wheel_vel_scale_,
        static_cast<double>(w1) / wheel_vel_scale_,
        static_cast<double>(w2) / wheel_vel_scale_,
        static_cast<double>(w3) / wheel_vel_scale_};

    // Convert wheel velocities to robot velocities
    auto vels = kinematics_->forward(wheel_vel);
    double dt = (now() - last_time_).seconds();
    last_time_ = now();

    // Update robot pose estimate
    double dx = vels[0] * std::cos(yaw_) - vels[1] * std::sin(yaw_);
    double dy = vels[0] * std::sin(yaw_) + vels[1] * std::cos(yaw_);
    x_ += dx * dt;
    y_ += dy * dt;
    yaw_ += vels[2] * dt;

    publishOdometry(now(), vels[0], vels[1], vels[2]);
}

// Main control loop: handles velocity smoothing, timeouts, and sends wheel commands
void RoverWheelNode::update()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    // If E-STOP or motors disabled, stop the robot
    if (estop_active_ || !wheels_enabled_)
    {
        current_cmd_vel_ = {0.0, 0.0, 0.0};
        target_cmd_vel_ = {0.0, 0.0, 0.0};
        auto wheel_vel = kinematics_->inverse(current_cmd_vel_);
        sendWheelVelocities(wheel_vel);
        return;
    }

    // If cmd_vel timeout, initiate smooth stop
    if ((now() - last_cmd_vel_time_).seconds() > cmd_vel_timeout_)
    {
        if (!is_stopping_)
        {
            target_cmd_vel_ = {0.0, 0.0, 0.0};
            is_stopping_ = true;
        }
    }

    if (!cmd_vel_enabled_)
    {
    }

    // Compute time since last smoothing step
    double dt = (now() - last_smooth_time_).seconds();
    last_smooth_time_ = now();

    std::array<double, 3> next_cmd_vel = target_cmd_vel_;
    if (use_vel_smoothing_)
    {
        // Smooth velocity towards target
        next_cmd_vel = smoother_->smooth(current_cmd_vel_, target_cmd_vel_, dt, is_stopping_);
    }
    else
    {
        // No smoothing, just use target directly
        next_cmd_vel = target_cmd_vel_;
    }

    // If velocity is below threshold, stop completely
    if (std::abs(next_cmd_vel[0]) < vel_threshold_ &&
        std::abs(next_cmd_vel[1]) < vel_threshold_ &&
        std::abs(next_cmd_vel[2]) < vel_threshold_)
    {
        next_cmd_vel = {0.0, 0.0, 0.0};
        is_stopping_ = false;
    }

    current_cmd_vel_ = next_cmd_vel;
    auto wheel_vel = kinematics_->inverse(current_cmd_vel_);
    sendWheelVelocities(wheel_vel);
}

// ==================== Main ====================

// Main entry point: initializes ROS2 and spins the node
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverWheelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}