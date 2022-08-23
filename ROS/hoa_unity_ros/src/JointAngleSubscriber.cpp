#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "hoa_unity_ros/JointAngle.h"

hoa_unity_ros::JointAngle local;

void jointAnglesCallback(const hoa_unity_ros::JointAngleConstPtr& msg){
    local = *msg;
    std::vector<float> angles = local.jointAnglesCollection;
    ROS_INFO_STREAM("Q: " << angles.at(0) << " | " << angles.at(1) << " | " << angles.at(2) << " | "
                        << angles.at(3) << " | " << angles.at(4) << " | " << angles.at(5) << " | "
                        << angles.at(6));
}

int main(int argc, char **argv){
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joint_angles", 1000, jointAnglesCallback);
    ros::spin();

    return 0;
}
