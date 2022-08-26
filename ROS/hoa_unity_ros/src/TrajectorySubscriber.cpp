/*
    Subscriber to SourceDestination topic
    Uses MoveIt to compute a trajectory from the target to the destination
    Trajectory is then published to PickAndPlaceTrajectory topic
*/

#include "ros/ros.h"
#include "hoa_unity_ros/UR3MoveitJoints.h"

hoa_unity_ros::UR3MoveitJoints local;

void Callback(const hoa_unity_ros::UR3MoveitJointsConstPtr& msg){ 
    local = *msg;
    ROS_INFO_STREAM("Q: "   << local.joints.at(0) << " | " << local.joints.at(1) << " | " 
                            << local.joints.at(2) << " | " << local.joints.at(3) << " | "
                            << local.joints.at(4) << " | " << local.joints.at(5));
}

int main(int argc, char **argv){
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("ur3_joints", 1000, Callback);
    ros::spin();

    return 0;
}