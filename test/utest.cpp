//
// Created by haavard on 15.10.17.
//

#include <control_fsm/control_fsm.hpp>
#include "gtest/gtest.h"

TEST(TestSuite, stateHandlerTest) {
    ControlFSM fsm;
    EXPECT_NE(0, StateInterface::getNumStates());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

