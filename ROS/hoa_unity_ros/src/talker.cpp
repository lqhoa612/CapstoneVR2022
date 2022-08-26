#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"

#include <sstream>

int main(int argc, char **argv){
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatterPub = n.advertise<std_msgs::Float32MultiArray>("chatter", 1000);
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "ain't that the invisible cunt" << count;
        msg.data = ss.str();
        //ROS_INFO("%s", msg.data.c_str());

        std_msgs::Float32MultiArray arr;
        std::vector<float> arr_(7,1);
        arr.data = arr_;
        ROS_INFO_STREAM("" << arr_.at(3));

        chatterPub.publish(arr);

        ros::spinOnce();

        loop_rate.sleep();
        count++;
    }
    
    return 0;
    
}