//
// Created by haavard on 15.10.17.
//

#include <control/tools/target_tools.hpp>
#include <control/fsm/control_fsm.hpp>
#include "gtest/gtest.h"

constexpr double PI_HALF = 1.57079632679;

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

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

