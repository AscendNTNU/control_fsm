#include <ros/ros.h>
#include <ascend_msgs/BoolStamped.h>
#include <geometry_msgs/PoseStamped.h>
namespace Config {
    std::string pubTopic = "/land_detector";
    std::string posTopic = "/mavros/local_position/pose";
    double onGroundThreshold = 0.1;
    double pubRateHz = 10; //Hz
};

ros::Publisher pub;

void loadStringParam(std::string name, std::string& value) {
    ros::NodeHandle n("~");
    if(!n.getParam(name, value)) {
        ROS_WARN("Couldn't find %s, using %s", name.c_str(), value.c_str());
    }
}

void loadDoubleParam(std::string name, double& value) {
    ros::NodeHandle n("~");
    if(!n.getParam(name, value)) {
        ROS_WARN("Couldn't find %s, using %f", name.c_str(), value);
    }
}

void lPosCB(const geometry_msgs::PoseStamped& msg) {
    ascend_msgs::BoolStamped pubMsg;
    pubMsg.value = (msg.pose.position.z < Config::onGroundThreshold);
    pub.publish(pubMsg);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "land_detector_node");
    ros::NodeHandle n;
    //Load params
    loadStringParam("pub_topic", Config::pubTopic);
    loadStringParam("pos_topic", Config::posTopic);
    loadDoubleParam("threshold", Config::onGroundThreshold);
    loadDoubleParam("pub_rate_hz", Config::pubRateHz);

    pub = n.advertise<ascend_msgs::BoolStamped>(Config::pubTopic, 1);
    ROS_INFO("[Land detector] Ready!");
    ros::Rate pubRate(Config::pubRateHz);
    while(ros::ok()) {
        pubRate.sleep();
        ros::spinOnce();
    }


    return 0;
}

