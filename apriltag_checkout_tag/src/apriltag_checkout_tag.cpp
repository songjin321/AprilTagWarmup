//
// Created by sj on 17-9-18.
//

#include "apriltag_checkout_tag/apriltag_checkout_tag.h"
#include "apriltags/Tag36h11.h"

AprilTagDetector::AprilTagDetector(ros::NodeHandle& nh): it_(nh),initRobot(true){
    AprilTags::TagCodes tag_codes = AprilTags::tagCodes36h11;//load the apriltags tag library
    tag_detector = new AprilTags::TagDetector(tag_codes);//add library to the detector
    // Subscribe image from camera
    image_sub = it_.subscribe("/camera/image_raw", 1, &AprilTagDetector::imageCallback, this);
    // Subscribe CameraInfo
    camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("/camera/camera_info", 1, &AprilTagDetector::cameraInfoCallback, this);
    // Subscribe the odometry, delete it if you don not need odom
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom_topic", 1, &AprilTagDetector::odomCallback, this);
    // Advertise tag position, orientation and odometry
    tagPose_odom_pub = nh.advertise<apriltag_checkout_tag::PoseStampedArray>("/tagPose_odom", 1);
    // Advertise processed image
    image_processed_pub = it_.advertise("/tag_detections_image", 1);
}
// Deconstructor
AprilTagDetector::~AprilTagDetector(){
    image_sub.shutdown();
}
void AprilTagDetector::odomCallback(const nav_msgs::OdometryConstPtr& Odometry){
    odom.pose = Odometry->pose;
    odom.twist = Odometry->twist;
    odom.header = Odometry->header;
    odom.header.frame_id = "/world";
}
void AprilTagDetector::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera){
    camera_fx = camera->K[0];
    camera_fy = camera->K[4];
    camera_px = camera->K[2];
    camera_py = camera->K[5];
}
void AprilTagDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Create a array to store tagPose and odom information for publishing
    apriltag_checkout_tag::PoseStampedArray tagPose_odom_array;
    // Copy the odom information
    tagPose_odom_array.odom = odom;
    // Convert ros image to opencv mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        ROS_INFO("read image normal");
    }
    catch (...)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    cv::Mat cameraImage_cv;
    cameraImage_cv = cv_ptr->image;
    // Detect tag from image
    std::vector<AprilTags::TagDetection> detections = tag_detector->extractTags(cameraImage_cv);
    // Assign the header
    tagPose_odom_array.header = cv_ptr->header;
    // Loop to add each tage to tag_pose_array
    for (int i = 0; i < detections.size(); i++) {
        detections[i].draw(cv_ptr->image);//draw tags on the received image
        // getRelativeTransform(size of the tag in meter, fx,fy,px,py)
        Eigen::Matrix4d transform = detections[i].getRelativeTransform(tag_size, camera_fx, camera_fy, camera_px, camera_py);
        Eigen::Matrix3d rot = transform.block(0,0,3,3);
        Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);
        geometry_msgs::PoseStamped tag_pose;
        tag_pose.pose.position.x = transform(0,3);
        tag_pose.pose.position.y = transform(1,3);
        tag_pose.pose.position.z = transform(2,3);
        tag_pose.pose.orientation.x = rot_quaternion.x();
        tag_pose.pose.orientation.y = rot_quaternion.y();
        tag_pose.pose.orientation.z = rot_quaternion.z();
        tag_pose.pose.orientation.w = rot_quaternion.w();
        tag_pose.header = cv_ptr->header;
        // Store the tag ID in frame_id
        ostringstream convert;
        convert << detections[i].id;
        tag_pose.header.frame_id = convert.str();
        // pose_pub_1.publish(tag_pose);
        tagPose_odom_array.poses.push_back(tag_pose);
        ROS_ERROR("detect it!!!");
    }
    // cv::imshow("show", cv_bridge::toCvShare(msg, "mono8")->image);
    // cv::imshow("view", cv_ptr->image);//view the video in the open cv, this line could be hidded
    tagPose_odom_pub.publish(tagPose_odom_array);//publish the pose array
    image_processed_pub.publish(cv_ptr->toImageMsg()); // publish the processed image
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_checkout_tag");
    ros::NodeHandle nh;
    // Uncomment following 2 line to create a cv window
    //cv::namedWindow("view", CV_WINDOW_AUTOSIZE);
    //cv::startWindowThread();
    AprilTagDetector detector(nh);
    ros::spin();
    // Uncomment following line to close window when exit.
    // cv::destroyWindow("view");
}

