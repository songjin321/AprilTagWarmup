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
#include <yaml-cpp/yaml.h>
#include <algorithm>
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
// aprilTag
struct AprilTag {
    unsigned int id;
    double x;
    double y;
    double theta;
};
// Get map information from map file
void getMap(string mapFile, vector<AprilTag> &aprilTags)
{
    aprilTags.clear();
    YAML::Node map = YAML::LoadFile(mapFile);
    YAML::Node tagNumberNode = map["tagNumber"];
    int tagNumber = map["tagNumber"].as<int>();
    for(std::size_t i = 0; i < tagNumber; i++)
    {
        string tag_name = "tag" + std::to_string(i);
        YAML::Node _tag = map[tag_name];
        AprilTag tag;
        tag.id = i;
        tag.x = _tag["x"].as<double>();
        tag.y = _tag["y"].as<double>();
        tag.theta = _tag["theta"].as<double>();
        aprilTags.push_back(tag);
    }
}
//　Use id as an index find corresponding AprilTag
void getAprilTag(const vector<AprilTag> &aprilTags, unsigned int id, AprilTag &re)
{
    auto re_ite = find_if(aprilTags.begin(), aprilTags.end(), [id](const AprilTag &a){
        return id == a.id;
    });
    if (re_ite != aprilTags.end())
        re = *re_ite;
}
// calculate the angle and distance
void get_odom_angle_translation(nav_msgs::Odometry &old_odom, nav_msgs::Odometry &new_odom, double &angle, double &distance)
{
    tf::Quaternion old_orientation(old_odom.pose.pose.orientation.x,
                                   old_odom.pose.pose.orientation.y,
                                   old_odom.pose.pose.orientation.z,
                                   old_odom.pose.pose.orientation.w);
    tf::Quaternion new_orientation(new_odom.pose.pose.orientation.x,
                                   new_odom.pose.pose.orientation.y,
                                   new_odom.pose.pose.orientation.z,
                                   new_odom.pose.pose.orientation.w);
    tf::Vector3 old_position(old_odom.pose.pose.position.x,
                             old_odom.pose.pose.position.y,
                             old_odom.pose.pose.position.z);
    tf::Vector3 new_position(new_odom.pose.pose.position.x,
                             new_odom.pose.pose.position.y,
                             new_odom.pose.pose.position.z);
    // Angle between two pose, (angle * axis at z) is the actual angle of turtlebot
    angle = new_orientation.getAngle() * new_orientation.getAxis().getZ() -
            old_orientation.getAngle() * new_orientation.getAxis().getZ();
    // Distance between two pose
    distance = new_position.distance(old_position);
}
#endif //PROJECT_APRILTAG_MAPPING_H
