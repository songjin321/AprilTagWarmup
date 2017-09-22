//
// Created by sj on 17-9-18.
//
#ifndef PROJECT_APRILTAG_MAPPING_H
#define PROJECT_APRILTAG_MAPPING_H
#include <ros/ros.h>
#include <isam/isam.h>
#include <apriltag_checkout_tag/PoseStampedArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
using namespace std;
using namespace isam;
/* Class iSAMprocessor, subscribe PoseStampedArray from pose_publisher
process the apriltags an odometry information to generate map using iSAM.
And publish the interactive markers for rviz to display
*/
class iSAMprocessor{
public:
    iSAMprocessor(ros::NodeHandle& nh);
    ~iSAMprocessor();
    void Processing();
    // Function to output the map to a text file
    void Finish(string name);
private:
    // the callback of tagPose_odom subscriber
    void tagPose_odomCallback(const apriltag_checkout_tag::PoseStampedArrayConstPtr& tagPose_odom_array);
    // process tagPose_odom and publish odom path
    void odomPublish(const nav_msgs::Odometry& odom);
    // Initialize factor graph and set the parameters create first node.
    void InitSlam();
    // Add factor node between the now robot pose to last robot pose
    void AddNodeRobot2Robot(tf::Transform t);
    // Add factor nodes between the now robot pose to marks
    void AddNodesRobot2Marks();
    // Intialize marker
    void InitMarker();
    // Publish the map of markers of the trajectory and tags
    void PublishMarker();
    // Find point of the tagID correspond to
    int FindID(string ID);
    // the Subscriber of tagPose_odom
    ros::Subscriber tagPose_odom_sub;
    // Publisher the marker and odom path after isam process
    ros::Publisher marker_pub;
    ros::Publisher odom_path_pub;
    // Markers for rviz
    visualization_msgs::Marker tags, estimate_path;
    // the factor graph problem solver
    Slam slam;
    // Noise for slam
    Noise noise3;
    // robot's poses
    vector<Pose2d_Node*> robot_pose_nodes;
    // AprilTags' pose
    vector<Pose2d_Node*> tag_pose_nodes;
    // the path of odom
    nav_msgs::Path odom_Path;
    // A array to store IDs of AprilTags
    vector<string> ID_array;
    // Record for received PoseArray and Odometry
    apriltag_checkout_tag::PoseStampedArray new_posearray;
    bool copyEnable;
    // Information extract from odom
    tf::Vector3 old_position;
    tf::Vector3 new_position;
    tf::Quaternion new_orientation;
    tf::Quaternion old_orientation;
    tf::Transform old_transform;
    tf::Transform new_transform;
    // for tf
    tf::TransformBroadcaster br;
    tf::TransformListener listener;
};
#endif //PROJECT_APRILTAG_MAPPING_H
