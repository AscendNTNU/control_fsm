#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pub_mocap; 

ascend_msgs::AIWorldObservation message; 

void quadCallback(const geometry_msgs::PoseStamped& input) {
    geometry_msgs::PoseStamped pose_ = input;

    //Transform position from y up to z up
    pose_.pose.position.z = input.pose.position.y;
    pose_.pose.position.y = -input.pose.position.z;
    pose_.pose.orientation.y = -input.pose.orientation.z;
    pose_.pose.orientation.z = input.pose.orientation.y;
    pose_.header = input.header;
    pose_.header.frame_id = "map";

    message.drone_position.x = roomba_pose.x;
    message.drone_position.y = roomba_pose.y;
    message.drone_position.z = roomba_pose.z; 


    


    pub_quad.publish(message);

}

void roombaCallback(const geometry_msgs::PoseStamped& input) {
	//Transform to correct coordinate system 
	auto roomba_pose = input.pose.position;
    roomba_pose.y = -input.pose.position.z;
    roomba_pose.z =  input.pose.position.y;




    pub_mocap.publish(message); 
}


int main(int argc, char **argv){
    //Initalize ros node
    ros::init(argc, argv, "mocapConverter");
    ros::NodeHandle n;

    //Subsribes to position messages from motion capture
    ros::Subscriber sub_quad = n.subscribe("vrpn_client_node/quad/pose", 100, quadCallback);
    ros::Subscriber sub_roomba = n.Subsribes("vrpn_client_node/roomba/pose", 100, roombaCallback); 
    //Publishing position to Mavros Motion Capture topic
    pub_mocap = n.advertise<ascend_msgs::AIWorldObservation>("/ai/world_observation", 100); 

    //Let ROS do its magic
    ros::spin();

}
