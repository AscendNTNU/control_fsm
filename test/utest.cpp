//
// Created by haavard on 15.10.17.
//

//Include all unit testing definitions
#ifndef CONTROL_FSM_UNIT_TEST
#define CONTROL_FSM_UNIT_TEST
#endif

#include <geometry_msgs/Vector3.h>

#include <control/tools/target_tools.hpp>
#include <control/fsm/control_fsm.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <control/tools/config.hpp>
#include <control/planner/path_planner.hpp>
#include <control/tools/obstacle_avoidance.hpp>
#include <control/tools/obstacle_math.hpp>
#include <control/tools/drone_handler.hpp>
#include "gtest/gtest.h"
#include <sstream>
#include <tf2/LinearMath/Transform.h>

constexpr double PI_HALF = 1.57079632679;
constexpr double PI = 3.14159265359;

TEST(ControlTest, tfTest) {
    using control::DroneHandler;
    ros::Time start_time = ros::Time::now();
    
    while(ros::ok() &&
          !DroneHandler::isTransformsValid() &&
          !DroneHandler::isGlobalPoseValid()) {

        ros::Duration(0.1).sleep();
        ros::spinOnce();
        if(ros::Time::now() - start_time > ros::Duration(2.0)) {
            return;
        }
    }

    const auto local_to_global_tf = DroneHandler::getLocal2GlobalTf();
    const auto global_to_local_tf = DroneHandler::getGlobal2LocalTf();
    const auto local_pose = DroneHandler::getCurrentLocalPose();
    const auto global_pose = DroneHandler::getCurrentGlobalPose();

    auto expected_global_x = local_pose.pose.position.x + local_to_global_tf.transform.translation.x;
    auto expected_global_y = local_pose.pose.position.y + local_to_global_tf.transform.translation.y;

    //Get transform message
    auto tf = control::DroneHandler::getLocal2GlobalTf();


    //Get tf matrix
    tf2::Transform tf_matrix;
    tf2::convert(tf.transform, tf_matrix);

    //Get position goal matrix
    tf2::Vector3 local_vec3;
    tf2::convert(local_pose.pose.position, local_vec3);
    //Apply global to local transform
    tf2::Vector3 global_vec3 = tf_matrix * local_vec3;

    EXPECT_FLOAT_EQ(-1.0, local_to_global_tf.transform.translation.x);
    EXPECT_FLOAT_EQ(1.0, local_to_global_tf.transform.translation.y);
    EXPECT_FLOAT_EQ(0.0, local_to_global_tf.transform.translation.z);
    EXPECT_FLOAT_EQ(-1.0, global_to_local_tf.transform.translation.y);
    EXPECT_FLOAT_EQ(1.0, global_to_local_tf.transform.translation.x);
    EXPECT_FLOAT_EQ(0.0, global_to_local_tf.transform.translation.z);
    EXPECT_FLOAT_EQ(expected_global_x, global_pose.pose.position.x);
    EXPECT_FLOAT_EQ(expected_global_y, global_pose.pose.position.y);
    EXPECT_FLOAT_EQ(expected_global_x, global_vec3.x());
    EXPECT_FLOAT_EQ(expected_global_y, global_vec3.y());
    EXPECT_FLOAT_EQ(local_pose.pose.position.z, global_vec3.z());
}

TEST(ControlTest, configTest) {
    std::stringstream not_found;
    for(auto& s : control::Config::getMissingParamSet()) {
        not_found << s << "\n";
    }
    EXPECT_TRUE(control::Config::getMissingParamSet().empty()) << "Not all params found:\n" << not_found.str();
}

TEST(ControlTest, goToStateHelpers) {
    geometry_msgs::TwistStamped test_vel;
    test_vel.twist.linear.x = 0.0;
    test_vel.twist.linear.y = 0.0;
    test_vel.twist.linear.z = 0.0;
    EXPECT_EQ(droneNotMovingXY(test_vel), true);
    test_vel.twist.linear.x = 3.0;
    EXPECT_EQ(droneNotMovingXY(test_vel), false);

    EXPECT_NEAR(calculatePathYaw(2.0, 1.0), 0.0, 0.0001);
    EXPECT_NEAR(calculatePathYaw(1.0, 2.0), PI_HALF, 0.0001);
    EXPECT_NEAR(calculatePathYaw(-1.0, 2.0), PI_HALF, 0.0001);
    EXPECT_NEAR(calculatePathYaw(-2.0, 1.0), PI, 0.0001);
    EXPECT_NEAR(calculatePathYaw(-1.0, -2.0), -PI_HALF, 0.0001);
    EXPECT_NEAR(calculatePathYaw(2.0, -1.0), 0.0, 0.0001);
}

TEST(ControlTest, stateHandlerTest) {
    ///There should be multiple states in the state vector
    EXPECT_NE(0, StateInterface::getNumStates()) << "No states in state vector";
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

TEST(ControlTest, obstacleAvoidanceHelpers) {
    
    using obstacle_math::angleWrapper;
    using obstacle_math::rotateXY;
    // angleWrapper
    EXPECT_NEAR(0.f, angleWrapper(0.f), 0.001);
    EXPECT_NEAR(2.f, angleWrapper(2.f), 0.001);
    EXPECT_NEAR(4.f, angleWrapper(4.f), 0.001);
    EXPECT_NEAR(0.7168f, angleWrapper(7.f), 0.001);
    EXPECT_NEAR(2.8673f, angleWrapper(28.f), 0.001);
    EXPECT_NEAR(5.2832f, angleWrapper(-1.f), 0.001);
    EXPECT_NEAR(0.8496f, angleWrapper(-18.f), 0.001);

    geometry_msgs::Vector3 vec = {};
    geometry_msgs::Vector3 zero =  {};
    geometry_msgs::Vector3 unitx; unitx.x=1.0; unitx.y=0.0; unitx.z=0.0; 
    geometry_msgs::Vector3 unity; unity.x=0.0; unity.y=1.0; unity.z=0.0; 
    
    geometry_msgs::Vector3 unitz; unitz.x=0.0; unitz.y=0.0; unitz.z=1.0; 

    // rotateXY
    geometry_msgs::Vector3 result = rotateXY(unitx, PI/2);
    EXPECT_NEAR(unity.x, result.x, 0.001);
    EXPECT_NEAR(unity.y, result.y, 0.001);
    EXPECT_NEAR(unity.z, result.z, 0.001);
    
    result = rotateXY(unitx, PI);
    EXPECT_NEAR(-unitx.x, result.x, 0.001);
    EXPECT_NEAR(-unitx.y, result.y, 0.001);
    EXPECT_NEAR(-unitx.z, result.z, 0.001);

    result = rotateXY(unity, 3*PI/2);
    EXPECT_NEAR(unitx.x, result.x, 0.001);
    EXPECT_NEAR(unitx.y, result.y, 0.001);
    EXPECT_NEAR(unitx.z, result.z, 0.001);

    result = rotateXY(unitz, 10.0);
    EXPECT_NEAR(unitz.x, result.x, 0.001);
    EXPECT_NEAR(unitz.y, result.y, 0.001);
    EXPECT_NEAR(unitz.z, result.z, 0.001);

    vec.x=2.0; vec.y=4.0; vec.z=3.0;
    result = rotateXY(vec, 12.3);
    EXPECT_NEAR(2.9824, result.x, 0.001);
    EXPECT_NEAR(3.3325, result.y, 0.001);
    EXPECT_NEAR(3.0, result.z, 0.001);

    // calcVectorBetweenPoints
    

}

TEST(PathPlannerTest, coordToIndexTest) {
    control::pathplanner::PathPlanner obj1(1,0.4);
    EXPECT_EQ(obj1.coordToIndex(0.0), 0);
    EXPECT_EQ(obj1.coordToIndex(0.9), 2);
    EXPECT_EQ(obj1.coordToIndex(0.7999), 2);
    EXPECT_EQ(obj1.coordToIndex(0.5999), 1);
    EXPECT_EQ(obj1.coordToIndex(0.6), 2);

    control::pathplanner::PathPlanner obj2(1, 0.5);
    EXPECT_EQ(obj2.coordToIndex(0.0), 0);
    EXPECT_EQ(obj2.coordToIndex(0.5), 1);
    EXPECT_EQ(obj2.coordToIndex(0.2499), 0);
    EXPECT_EQ(obj2.coordToIndex(10), 20);
    EXPECT_EQ(obj2.coordToIndex(0.4999), 1);
}

TEST(ControlTest, arenaBoundariesTest) {
    using control::Config;
    double valid_x = Config::arena_lowest_x + (Config::arena_highest_x - Config::arena_lowest_x) / 2.0;
    double valid_y = Config::arena_lowest_y + (Config::arena_highest_y - Config::arena_lowest_y) / 2.0;
    double invalid_x = Config::arena_lowest_x - 1.0;
    double invalid_y = Config::arena_lowest_y - 1.0;
    tf2::Vector3 vec(valid_x, valid_y, 1.0);
    EXPECT_TRUE(targetWithinArena(vec));
    vec = tf2::Vector3(valid_x, invalid_y, 1.0);
    EXPECT_FALSE(targetWithinArena(vec));
    vec = tf2::Vector3(invalid_x, valid_y, 1.0);
    EXPECT_FALSE(targetWithinArena(vec));
    vec = tf2::Vector3(invalid_x, invalid_y, 1.0);
    EXPECT_FALSE(targetWithinArena(vec));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_fsm_unit_test");
    control::Config::loadParams();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
