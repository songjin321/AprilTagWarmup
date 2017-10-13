//
// Created by sj on 17-9-18.
//

#ifndef PROJECT_APRILTAG_CHECKOUT_TAG_H
#define PROJECT_APRILTAG_CHECKOUT_TAG_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <apriltag_checkout_tag/PoseStampedArray.h>
#include <apriltags/TagDetector.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
/*
This class is used to process images from camera
It will detect the apriltags on the image and output the
tags information and the odom information at the time the
processed image is received
!!!YOU SHOULD CHANGE THE APRILTAG SIZE tag_size
*/
class AprilTagDetector {
public:
    AprilTagDetector(ros::NodeHandle &nh);

    ~AprilTagDetector();

private:
    // the image subscriber callback
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    // the odom subscriber callback
    void odomCallback(const nav_msgs::OdometryConstPtr &Odometry);

    // the cameraInfo subscriber callback
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &camera);

    // subscriber for images
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub;
    // subscriber for odom
    ros::Subscriber odom_sub;
    // subscriber for camera_info
    ros::Subscriber camera_info_sub;
    // publisher for the processed AprilTags image
    image_transport::Publisher image_processed_pub;
    // publisher for the tag detections with odom information
    ros::Publisher tagPose_odom_pub;
    // store current odom information
    nav_msgs::Odometry odom;
    // camera info will be used in apriltag detecting, initialized for pointgrey
    double camera_fx = 694.392686451998;
    double camera_fy = 694.644309876203;
    double camera_px = 383.093922546586;
    double camera_py = 235.687225828983;
    // apriltag_size meter
    double tag_size = 0.165;
    // init apriltag tag_detector
    AprilTags::TagDetector *tag_detector;
    //broadcast init tf info
    bool initRobot;
    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf::StampedTransform initTransform;
};
#endif //PROJECT_APRILTAG_CHECKOUT_TAG_H
