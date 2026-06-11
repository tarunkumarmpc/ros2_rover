#include <gtest/gtest.h>
#include "rover_base/rover_wheel_node.hpp"

// =============== MecanumKinematics Tests ===============
TEST(MecanumKinematicsTest, Inverse_ForwardOnly) {
    MecanumKinematics kin(0.05, 0.15, 0.15);
    std::array<double, 3> cmd_vel = {1.0, 0.0, 0.0}; // Forward
    auto wheel_vel = kin.inverse(cmd_vel);

    EXPECT_NEAR(wheel_vel[0], 20.0, 1e-6); // FL
    EXPECT_NEAR(wheel_vel[1], 20.0, 1e-6); // FR
    EXPECT_NEAR(wheel_vel[2], 20.0, 1e-6); // BL
    EXPECT_NEAR(wheel_vel[3], 20.0, 1e-6); // BR
}

TEST(MecanumKinematicsTest, Inverse_StrafeRight) {
    MecanumKinematics kin(0.05, 0.15, 0.15);
    std::array<double, 3> cmd_vel = {0.0, -1.0, 0.0}; // Strafe right
    auto wheel_vel = kin.inverse(cmd_vel);

    EXPECT_NEAR(wheel_vel[0], 20.0, 1e-6);  // FL
    EXPECT_NEAR(wheel_vel[1], -20.0, 1e-6); // FR
    EXPECT_NEAR(wheel_vel[2], 20.0, 1e-6);  // BL
    EXPECT_NEAR(wheel_vel[3], -20.0, 1e-6); // BR
}

TEST(MecanumKinematicsTest, Inverse_RotateCCW) {
    MecanumKinematics kin(0.05, 0.15, 0.15);
    std::array<double, 3> cmd_vel = {0.0, 0.0, 1.0}; // Rotate CCW
    auto wheel_vel = kin.inverse(cmd_vel);

    EXPECT_NEAR(wheel_vel[0], -30.0, 1e-6);
    EXPECT_NEAR(wheel_vel[1], 30.0, 1e-6);
    EXPECT_NEAR(wheel_vel[2], -30.0, 1e-6);
    EXPECT_NEAR(wheel_vel[3], 30.0, 1e-6);
}

TEST(MecanumKinematicsTest, Forward_CheckConsistency) {
    MecanumKinematics kin(0.05, 0.15, 0.15);
    std::array<double, 4> wheel_vel = {20.0, 20.0, 20.0, 20.0};
    auto base_vel = kin.forward(wheel_vel);

    EXPECT_NEAR(base_vel[0], 1.0, 1e-6); // vx
    EXPECT_NEAR(base_vel[1], 0.0, 1e-6); // vy
    EXPECT_NEAR(base_vel[2], 0.0, 1e-6); // wz
}

// =============== VelocitySmoother Tests ===============
TEST(VelocitySmootherTest, SmoothStep_LinearInterpolation) {
    VelocitySmoother smoother(1.0, 1.0);
    EXPECT_NEAR(smoother.smoothStep(0.0, 1.0, 0.5, 1.0), 0.5, 1e-6);
    EXPECT_NEAR(smoother.smoothStep(0.0, 1.0, 1.0, 1.0), 1.0, 1e-6);
}

TEST(VelocitySmootherTest, SmoothStep_CubicBehavior) {
    VelocitySmoother smoother(1.0, 1.0);
    double result = smoother.smoothStep(0.0, 1.0, 0.5, 1.0);
    EXPECT_GT(result, 0.5); // Cubic should be > linear at midpoint
    EXPECT_LT(result, 1.0);
}

TEST(VelocitySmootherTest, Smooth_NoSmoothing_WhenDurationZero) {
    VelocitySmoother smoother(0.0, 0.0);
    std::array<double, 3> current = {0.0, 0.0, 0.0};
    std::array<double, 3> target = {1.0, 2.0, 3.0};
    auto result = smoother.smooth(current, target, 1.0, false);
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(result[i], target[i]);
    }
}

TEST(VelocitySmootherTest, Smooth_DecelerationDirection) {
    VelocitySmoother smoother(0.5, 0.5);
    std::array<double, 3> current = {2.0, 2.0, 2.0};
    std::array<double, 3> target = {0.0, 0.0, 0.0};
    auto result = smoother.smooth(current, target, 0.25, true);
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_LT(result[i], current[i]);
        EXPECT_GT(result[i], target[i]);
    }
}

// =============== Main ===============
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
