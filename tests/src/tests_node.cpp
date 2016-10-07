#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tests/add.h"
#include <sstream>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tests_node");
    ros::NodeHandle nh;
    
    int sum = add(1,2);
    
    std::stringstream ss;
    ss << "Hello World." << sum;
    
    std_msgs::String msg;
    
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    
    return 0;    



}

