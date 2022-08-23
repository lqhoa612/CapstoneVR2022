#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <stdio.h>

using std::vector;

void jointAnglesCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    vector<float> _list;
    for (int i = 0; i < msg->data.size(); i++){
        _list.push_back(msg->data.at(i));
    }
    ROS_INFO_STREAM("Q: "   << _list.at(0) << " | " << _list.at(1) << " | "
                            << _list.at(2) << " | " << _list.at(3) << " | "
                            << _list.at(4) << " | " << _list.at(5) << " | " 
                            << _list.at(6));
}

int main(int argc, char **argv){
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joint_angle", 1000, jointAnglesCallback);
    ros::spin();

    return 0;
}