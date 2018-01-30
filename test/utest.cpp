//
// Created by haavard on 15.10.17.
//

//Include all unit testing definitions
#ifndef CONTROL_FSM_UNIT_TEST
#define CONTROL_FSM_UNIT_TEST
#endif

#include <control/tools/target_tools.hpp>
#include <control/fsm/control_fsm.hpp>
#include "gtest/gtest.h"

constexpr double PI_HALF = 1.57079632679;
constexpr double PI = 3.14159265359;

TEST(ControlTest, goToStateHelpers) {
    geometry_msgs::TwistStamped test_vel;
    test_vel.twist.linear.x = 0.0;
    test_vel.twist.linear.y = 0.0;
    test_vel.twist.linear.z = 0.0;
    EXPECT_EQ(droneNotMoving(test_vel), true);
    test_vel.twist.linear.x = 3.0;
    EXPECT_EQ(droneNotMoving(test_vel), false);

    EXPECT_NEAR(calculatePathYaw(2.0, 1.0), 0.0, 0.0001);
    EXPECT_NEAR(calculatePathYaw(1.0, 2.0), PI_HALF, 0.0001);
    EXPECT_NEAR(calculatePathYaw(-1.0, 2.0), PI_HALF, 0.0001);
    EXPECT_NEAR(calculatePathYaw(-2.0, 1.0), PI, 0.0001);
    EXPECT_NEAR(calculatePathYaw(-1.0, -2.0), -PI_HALF, 0.0001);
    EXPECT_NEAR(calculatePathYaw(2.0, -1.0), 0.0, 0.0001);
}

TEST(ControlTest, stateHandlerTest) {
    ///There should be multiple states in the state vector
    EXPECT_NE(0, StateInterface::getNumStates());
}

TEST(ControlTest, yawTargetTest) {
    EXPECT_NEAR(PI_HALF - PI_HALF,control::getMavrosCorrectedTargetYaw(PI_HALF + 0.5), 0.001);
    EXPECT_NEAR(0.0 - PI_HALF, control::getMavrosCorrectedTargetYaw(0.5), 0.001);
    EXPECT_NEAR(-PI_HALF - PI_HALF, control::getMavrosCorrectedTargetYaw(-PI_HALF + 0.5), 0.001);
    EXPECT_NEAR(PI_HALF*2.0 - PI_HALF, control::getMavrosCorrectedTargetYaw(PI_HALF + 0.8), 0.001);
}

TEST(ControlTest, quatConversionTest) {
    struct Quaternion {
        double x;
        double y;
        double z;
        double w;
    };
    Quaternion zero { 0.0, 0.0, 0.0, 1.0 };
    Quaternion pi6 { 0.0, 0.0, 0.259, 0.966 };
    Quaternion pi { 0.0, 0.0, 1.0, 0.0 };
    Quaternion pi3 { 0.0, 0.0, 0.50, 0.866 };
    using control::pose::quat2yaw;
    using control::pose::quat2mavrosyaw;
    EXPECT_NEAR(0.0, quat2yaw(zero), 0.001);
    EXPECT_NEAR(PI / 6.0, quat2yaw(pi6), 0.001);
    EXPECT_NEAR(PI, quat2yaw(pi), 0.001);
    EXPECT_NEAR(PI / 3.0, quat2yaw(pi3), 0.001);

    EXPECT_NEAR(0.0 - PI_HALF, quat2mavrosyaw(zero), 0.001);

}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

