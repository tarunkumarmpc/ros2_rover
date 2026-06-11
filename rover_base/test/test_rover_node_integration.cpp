#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "rover_base/rover_wheel_node.hpp"

class RoverWheelNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node = std::make_shared<RoverWheelNode>();
    }

    void TearDown() override {
        node.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<RoverWheelNode> node;
};

TEST_F(RoverWheelNodeTest, CmdVelSubscriber_ReceivesCommand) {
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    while (publisher->get_subscription_count() == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    geometry_msgs::msg::Twist msg;
    msg.linear.x = 1.0;
    msg.angular.z = 0.5;

    publisher->publish(msg);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto current_vel = node->get_current_cmd_vel();
    EXPECT_NEAR(current_vel[0], 1.0, 1e-6);
    EXPECT_NEAR(current_vel[2], 0.5, 1e-6);
}

TEST_F(RoverWheelNodeTest, OdometryPublished) {
    nav_msgs::msg::Odometry::SharedPtr last_odom;
    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
            last_odom = msg;
        });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin_once(std::chrono::seconds(1));

    ASSERT_TRUE(last_odom != nullptr);
    EXPECT_NEAR(last_odom->pose.pose.position.x, 0.0, 1e-3);
    EXPECT_NEAR(last_odom->pose.pose.position.y, 0.0, 1e-3);
}

TEST_F(RoverWheelNodeTest, ResetOdometryService_ResetPosition) {
    auto client = node->create_client<std_srvs::srv::Trigger>("/reset_odometry");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node->get_logger(), "Waiting for reset_odometry service...");
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
    EXPECT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    auto response = future.get();

    EXPECT_TRUE(response->success);
    EXPECT_EQ(response->message, "Odometry reset successfully");

    EXPECT_NEAR(node->get_x(), 0.0, 1e-6);
    EXPECT_NEAR(node->get_y(), 0.0, 1e-6);
    EXPECT_NEAR(node->get_yaw(), 0.0, 1e-6);
}

TEST_F(RoverWheelNodeTest, EStopService_DisablesMotors) {
    auto client = node->create_client<std_srvs::srv::SetBool>("/estop");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node->get_logger(), "Waiting for estop service...");
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true; // Disable motors

    auto future = client->async_send_request(request);
    EXPECT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    auto response = future.get();

    EXPECT_TRUE(response->success);
    EXPECT_EQ(response->message, "Motors DISABLED (E-STOP)");

    EXPECT_TRUE(node->is_estop_active());
    EXPECT_FALSE(node->are_wheels_enabled());
}

// Add more integration tests as needed...
