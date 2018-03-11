#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ascend_msgs/AIWorldObservation.h>
#include <ascend_msgs/LandDetector.h>

geometry_msgs::PoseStamped quad_pose;
geometry_msgs::PoseStamped roomba_pose; 

ascend_msgs::AIWorldObservation ai_message; 
ascend_msgs::LandDetector landing_message; 

ros::Time start_time; 
ros::Duration diff; 

ros::Publisher pub_landing; 
ros::Publisher pub_ai_data; 

void roomba_constant_pos(); 

void roomba_moving_in_straight_line(); 


void quadCallback(const geometry_msgs::PoseStamped& input){
	quad_pose = input; 
	if (quad_pose.pose.position.z < 0.05){
		landing_message.state = 1; //Landed
	}else if(quad_pose.pose.position.z > 0.05){
		landing_message.state = 2; // In air
	}
	diff = ros::Time::now() - start_time; 
	ai_message.elapsed_time = diff.toSec(); // Time since start in seconds
	std::cout << "Elapsed time is: " << diff.toSec() << std::endl; 

	
	ai_message.header = quad_pose.header;
	ai_message.drone_position.x = quad_pose.pose.position.x;
	ai_message.drone_position.y = quad_pose.pose.position.y;
	ai_message.drone_position.z = quad_pose.pose.position.z;

}

void roomba_constant_pos(){
	ai_message.ground_robots[0].header.stamp = ros::Time::now();
	ai_message.ground_robots[0].x = 1.0; 
	ai_message.ground_robots[0].y = 1.0;
	ai_message.ground_robots[0].theta = 0.0; // Dont know dont care
}

void roomba_moving_in_straight_line(){
	static float x = 1.0; 
	static float y = 1.0; 
	static float theta = 0.0; // Dont know, dont care
	static float speed = 0.3; // Speed of the roomba


	static ros::Time time = ros::Time::now(); 
	ros::Duration time_step = ros::Time::now() - time; // Find the time since last to this function

	x = x + speed * time_step.toSec(); 

	// Move in straight line between x = 3 and x = -1 at the speed 0,3 m/s
	if(x > 3.0 || x < -1.0){
		speed = -speed; //Turn
	}
	ai_message.ground_robots[0].header.stamp = ros::Time::now(); 
	ai_message.ground_robots[0].x = x;
	ai_message.ground_robots[0].y = y;
	ai_message.ground_robots[0].theta = theta; 
}


int main(int argc, char **argv){
	
	ros::init(argc, argv, "data_faker");
    ros::NodeHandle n;
    start_time = ros::Time::now();

    ros::Subscriber sub_quad = n.subscribe("/mavros/local_position/pose", 100, quadCallback); // Find the position of the quad

    pub_landing = n.advertise<ascend_msgs::LandDetector>("gazebo_land_detector", 100); 
    pub_ai_data = n.advertise<ascend_msgs::AIWorldObservation>("/ai/world_observation",100);

    landing_message.state = 0; // Undefined

    ros::Rate loop_rate(30);
    while(ros::ok()){
    	ros::spinOnce(); 
    	loop_rate.sleep(); 


    	roomba_constant_pos(); // Roomba standing still
    	//roomba_moving_in_straight_line(); // Roomba move in straight line in x dir
    	pub_landing.publish(landing_message); 
		pub_ai_data.publish(ai_message); 
    }
	return 0;
}