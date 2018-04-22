// Main change is: 
// 1) subcribe video source from usb capture, not /usb_cam/image_rect
// 2) Use timer driven at 25hz
// 3) Delete detection image publishing
// New added header
#include <ros/ros.h>
#include "opencv2/opencv.hpp"


// Original header
#include <apriltags_ros/apriltag_detector.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>
#include <XmlRpcException.h>

namespace apriltags_ros{


const char* windowName = "Apriltag with detections";

// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

AprilTagDetector::AprilTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh): it_(nh){
  
  
  // Parse the description in launch file and save into descriptions_
  XmlRpc::XmlRpcValue april_tag_descriptions;
  if(!pnh.getParam("tag_descriptions", april_tag_descriptions)){
    ROS_WARN("No april tags specified");
  }
  else{
    try{
      descriptions_ = parse_tag_descriptions(april_tag_descriptions);
    } catch(XmlRpc::XmlRpcException e){
      ROS_ERROR_STREAM("Error loading tag descriptions: "<<e.getMessage());
    }
  }

  // Get sensor_frame_id from launch file
  if(!pnh.getParam("sensor_frame_id", sensor_frame_id_)){
    sensor_frame_id_ = "";
  }

  // Set tag_family and Projected_optics 
  std::string tag_family;
  pnh.param<std::string>("tag_family", tag_family, "36h11");

  pnh.param<bool>("projected_optics", projected_optics_, false);

  // Determine tag_codes to decode
  const AprilTags::TagCodes* tag_codes;
  if(tag_family == "16h5"){
    tag_codes = &AprilTags::tagCodes16h5;
  }
  else if(tag_family == "25h7"){
    tag_codes = &AprilTags::tagCodes25h7;
  }
  else if(tag_family == "25h9"){
    tag_codes = &AprilTags::tagCodes25h9;
  }
  else if(tag_family == "36h9"){
    tag_codes = &AprilTags::tagCodes36h9;
  }
  else if(tag_family == "36h11"){
    tag_codes = &AprilTags::tagCodes36h11;
  }
  else{
    ROS_WARN("Invalid tag family specified; defaulting to 36h11");
    tag_codes = &AprilTags::tagCodes36h11;
  }

  // Initialize tag_detector and publisher & subscriber 
  // input: image_rect. image_rect_color: Rectified image, de-Bayered and undistorted 
  // output: tag_detection_image, tag_detections, tag_detections_pose
  tag_detector_= boost::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(*tag_codes));
  // image_sub_ = it_.subscribeCamera("image_rect", 1, &AprilTagDetector::imageCb, this);
  
  // image_pub_ = it_.advertise("tag_detections_image", 1);
  detections_pub_ = nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("tag_detections_pose", 1);


}
AprilTagDetector::~AprilTagDetector(){
  // image_sub_.shutdown();
}

void AprilTagDetector::setupVideo(){

    // find and open a USB camera (built in laptop camera, web cam etc)
    m_cap = cv::VideoCapture(0);
    if(!m_cap.isOpened()) {
      cerr << "ERROR: Can't find video device " << 0 << "\n";
      exit(1);
    }

    cv::namedWindow(windowName, 1);

    m_cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    cout << "Camera successfully opened (ignore error messages above...)" << endl;
    cout << "Actual resolution: "
         << m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
         << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
}

void AprilTagDetector::imageCb(const ros::TimerEvent& event){
  
  // argv: const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info
  // pass the image from image msg to opencv pointer
  // cv_bridge::CvImagePtr cv_ptr;
  // try{
  //   cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  // }
  // catch (cv_bridge::Exception& e){
  //   ROS_ERROR("cv_bridge exception: %s", e.what());
  //   return;
  // }

  // Convert color image to gray image
  cv::Mat image;
  cv::Mat gray;
  // capture frame
  m_cap >> image;
  cv::cvtColor(image, gray, CV_BGR2GRAY);

  double t0;
  t0 = tic();

  // ExtractTags
  // input: gray
  // output: detections
  std::vector<AprilTags::TagDetection>	detections = tag_detector_->extractTags(gray);
  ROS_DEBUG("%d tag detected", (int)detections.size());

  cout << detections.size() << " tags detected:" << endl;

  double dt = tic()-t0;
  cout << "Extracting tags took " << dt << " seconds." << endl;


  // Initialize focal length and principal point
  // From head_camera.yaml
  // use projected focal length and principal point
  double fx = 665.6666259765625;
  double fy = 664.1488647460938;
  double px = 334.6867364108111;
  double py = 252.8130248174875;
  // if (projected_optics_) {
  //   // use projected focal length and principal point
  //   // these are the correct values
  //   fx = cam_info->P[0];
  //   fy = cam_info->P[5];
  //   px = cam_info->P[2];
  //   py = cam_info->P[6];
  // } else {
  //   // use camera intrinsic focal length and principal point
  //   // for backwards compatability
  //   fx = cam_info->K[0];
  //   fy = cam_info->K[4];
  //   px = cam_info->K[2];
  //   py = cam_info->K[5];
  // }

  // if(!sensor_frame_id_.empty())
  //   cv_ptr->header.frame_id = sensor_frame_id_;

  // Initialize msg and data holder
  AprilTagDetectionArray tag_detection_array;
  geometry_msgs::PoseArray tag_pose_array;
  tag_pose_array.header.stamp = ros::Time::now();


  // for each detections, calculate translation and quaternion
  BOOST_FOREACH(AprilTags::TagDetection detection, detections){

    // Use iterator to find the tag id in description of launch file
    std::map<int, AprilTagDescription>::const_iterator description_itr = descriptions_.find(detection.id);
    if(description_itr == descriptions_.end()){
      ROS_WARN_THROTTLE(10.0, "Found tag: %d, but no description was found for it", detection.id);
      continue;
    }
    AprilTagDescription description = description_itr->second;
    double tag_size = description.size();

    // Draw detection frame on image
    detection.draw(image);

    // Prepare image/object points, solvePnP, get transform matrix
    Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size, fx, fy, px, py);
    // Crop rotation matrix
    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
    // Convert to Quaternion
    Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);

    // Prepare tag_pose & tag_detection msg
    geometry_msgs::PoseStamped tag_pose;
    tag_pose.pose.position.x = transform(0, 3);
    tag_pose.pose.position.y = transform(1, 3);
    tag_pose.pose.position.z = transform(2, 3);
    tag_pose.pose.orientation.x = rot_quaternion.x();
    tag_pose.pose.orientation.y = rot_quaternion.y();
    tag_pose.pose.orientation.z = rot_quaternion.z();
    tag_pose.pose.orientation.w = rot_quaternion.w();
    tag_pose.header = tag_pose_array.header;

    AprilTagDetection tag_detection;
    tag_detection.pose = tag_pose;
    tag_detection.id = detection.id;
    tag_detection.size = tag_size;

    tag_detection_array.detections.push_back(tag_detection);
    tag_pose_array.poses.push_back(tag_pose.pose);

    // tf is a package that lets the user keep track of multiple coordinate frames over time. 
    //tf maintains the relationship between coordinate frames in a tree structure buffered in time, 
    //and lets the user transform points, vectors, etc between any two coordinate frames at any desired point in time.
    tf::Stamped<tf::Transform> tag_transform;
    tf::poseStampedMsgToTF(tag_pose, tag_transform);
    tf_pub_.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, tag_transform.frame_id_, description.frame_name()));
  }


  detections_pub_.publish(tag_detection_array);
  pose_pub_.publish(tag_pose_array);

  cv::imshow("Apriltag with detections", image);
  // Not publishing image w/ detections
  // image_pub_.publish(cv_ptr->toImageMsg());
}


std::map<int, AprilTagDescription> AprilTagDetector::parse_tag_descriptions(XmlRpc::XmlRpcValue& tag_descriptions){
  std::map<int, AprilTagDescription> descriptions;
  ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < tag_descriptions.size(); ++i) {
    XmlRpc::XmlRpcValue& tag_description = tag_descriptions[i];
    ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    int id = (int)tag_description["id"];
    double size = (double)tag_description["size"];

    std::string frame_name;
    if(tag_description.hasMember("frame_id")){
      ROS_ASSERT(tag_description["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
      frame_name = (std::string)tag_description["frame_id"];
    }
    else{
      std::stringstream frame_name_stream;
      frame_name_stream << "tag_" << id;
      frame_name = frame_name_stream.str();
    }
    AprilTagDescription description(id, size, frame_name);
    ROS_INFO_STREAM("Loaded tag config: "<<id<<", size: "<<size<<", frame_name: "<<frame_name);
    descriptions.insert(std::make_pair(id, description));
  }
  return descriptions;
}


}
