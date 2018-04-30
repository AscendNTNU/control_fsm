#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <ascend_msgs/AIWorldObservation.h>
#include <ascend_msgs/LandDetector.h>


constexpr double land_height = 0.18; 

ros::Publisher pub_mocap; 
ros::Publisher pub_landing; 

ascend_msgs::AIWorldObservation ai_msg; 
ascend_msgs::LandDetector landing_msg;

ros::Time start_time;
ros::Duration diff;

void quadCallback(const geometry_msgs::PoseStamped& input) {
    geometry_msgs::PoseStamped quad = input;

    //Transform position from y up to z up
    quad.pose.position.z = input.pose.position.y;
    quad.pose.position.y = -input.pose.position.z;
    quad.pose.orientation.y = -input.pose.orientation.z;
    quad.pose.orientation.z = input.pose.orientation.y;
    quad.header = input.header;
    quad.header.frame_id = "map";

    diff = ros::Time::now() - start_time; 
	ai_msg.elapsed_time = diff.toSec();

    ai_msg.header = input.header;
    ai_msg.drone_position.x = quad.pose.position.x;
    ai_msg.drone_position.y = quad.pose.position.y;
    ai_msg.drone_position.z = quad.pose.position.z; 

    landing_msg.header.stamp = ros::Time::now(); 
    if (quad.pose.position.z < land_height){
		landing_msg.state = 1; //Landed
	}else if(quad.pose.position.z > land_height){
		landing_msg.state = 2; // In air
	}


    pub_mocap.publish(ai_msg);
    pub_landing.publish(landing_msg);
}

void roombaCallback(const geometry_msgs::PoseStamped& input) {
	//Transform to correct coordinate system 
	auto roomba_pose = input.pose.position;
    roomba_pose.y = -input.pose.position.z;
    roomba_pose.z =  input.pose.position.y;

    ai_msg.ground_robots[0].header = input.header;
    ai_msg.ground_robots[0].x = roomba_pose.x; 
    ai_msg.ground_robots[0].y = roomba_pose.y;

    if(fabs(roomba_pose.x - ai_msg.drone_position.x) < 0.5 && fabs(roomba_pose.y - ai_msg.drone_position.y) < 0.5){
    	ai_msg.ground_robots[0].visible = true; 
    	ai_msg.ground_robots[0].downward_tracked = true; 
    }else{
    	ai_msg.ground_robots[0].visible = false; 
    	ai_msg.ground_robots[0].downward_tracked = false; 
    }

    pub_mocap.publish(ai_msg); 
}


int main(int argc, char **argv){
    //Initalize ros node
    ros::init(argc, argv, "mocapConverter");
    ros::NodeHandle n;

    start_time = ros::Time::now();
    landing_msg.state = 0; // Undefined

    //Subsribes to position messages from motion capture
    ros::Subscriber sub_quad = n.subscribe("vrpn_client_node/quad/pose", 100, quadCallback);
    ros::Subscriber sub_roomba = n.subscribe("vrpn_client_node/roomba/pose", 100, roombaCallback); 

    pub_landing = n.advertise<ascend_msgs::LandDetector>("land_detector", 100); 
    pub_mocap = n.advertise<ascend_msgs::AIWorldObservation>("/ai/world_observation",100);
    //Let ROS do its magic
    ros::Rate loop_rate(30);
    while(ros::ok()){
    	ros::spinOnce(); 
    	loop_rate.sleep(); 
    }

}
