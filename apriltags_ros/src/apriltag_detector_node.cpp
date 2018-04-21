#include <apriltags_ros/apriltag_detector.h>
#include <ros/ros.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "apriltag_detector");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    apriltags_ros::AprilTagDetector detector(nh, pnh);

    // setup image source, window for drawing, serial port...
    detector.setupVideo();

    // main control loop = 25 Hz
    double dTimeStepSec = 0.04;
    ros::Timer timer = nh.createTimer(ros::Duration(dTimeStepSec), detector.imageCb);

    ros::spin();
}
