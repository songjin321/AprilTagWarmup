//
// Created by sj on 17-9-18.
//

#include "apriltag_mapping/apriltag_mapping.h"
#include <yaml-cpp/yaml.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_mapping");
    ros::NodeHandle nh;
    iSAMprocessor processor(nh);
    while(ros::ok()){
        ros::spinOnce();
    }
    // The name of the slam output file, which will be generated when closing
    processor.Finish("result.txt");
}
iSAMprocessor::iSAMprocessor(ros::NodeHandle& nh){

    // Initialize Publisher and subscriber
    tagPose_odom_sub = nh.subscribe<apriltag_checkout_tag::PoseStampedArray>
                       ("/tagPose_odom", 1, &iSAMprocessor::tagPose_odomCallback,this);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/path_tag", 1);
    odom_path_pub = nh.advertise<nav_msgs::Path>("/odom_path",1);
    copyEnable=true;
    InitMarker();
    InitSlam();
}
// Shutdown the rosnode
iSAMprocessor::~iSAMprocessor(){
    ros::shutdown();
}
void iSAMprocessor::odomPublish(const nav_msgs::Odometry& odom)
{
    odom_Path.header = odom.header;
    odom_Path.header.frame_id = "/world";
    geometry_msgs::PoseStamped odom_pose;
    odom_pose.header = odom.header;
    odom_pose.header.frame_id = "/world";
    odom_pose.pose = odom.pose.pose;
    odom_Path.poses.push_back(odom_pose);
    odom_path_pub.publish(odom_Path);
}
// Save the PoseArray and odom Information
void iSAMprocessor::tagPose_odomCallback(const apriltag_checkout_tag::PoseStampedArrayConstPtr& _tagPose_odom_array){
    new_posearray.header=_tagPose_odom_array->header;
    new_posearray.poses=_tagPose_odom_array->poses;
    new_orientation=tf::Quaternion(_tagPose_odom_array->odom.pose.pose.orientation.x,
                                   _tagPose_odom_array->odom.pose.pose.orientation.y,
                                   _tagPose_odom_array->odom.pose.pose.orientation.z,
                                   _tagPose_odom_array->odom.pose.pose.orientation.w);;
    new_position=tf::Vector3(_tagPose_odom_array->odom.pose.pose.position.x,
                             _tagPose_odom_array->odom.pose.pose.position.y,
                             _tagPose_odom_array->odom.pose.pose.position.z);
    new_transform.setRotation(new_orientation);
    new_transform.setOrigin(new_position);
    // Only copy once, this if condition can make the slam always start from the origin without restart the robot
    if(copyEnable){
        old_position = new_position;
        old_orientation = new_orientation;
        old_transform = new_transform;
        copyEnable=false;
    }
    odomPublish(_tagPose_odom_array->odom);
    Processing();
}
void iSAMprocessor::InitSlam() {
    // Set noise
    noise3 = Information(30. * eye(3));
    // Create a first pose (a node)
    Pose2d_Node* new_pose_node = new Pose2d_Node();
    // Add it to the graph
    slam.add_node(new_pose_node);
    robot_pose_nodes.push_back(new_pose_node);
    // Create a prior measurement (a factor)
    Pose2d origin;
    Pose2d_Factor* prior = new Pose2d_Factor(robot_pose_nodes[0], origin, noise3);
    // Add it to the graph
    slam.add_factor(prior);
}
void iSAMprocessor::InitMarker(){
    // set for tags
    tags.type=visualization_msgs::Marker::CUBE_LIST;
    tags.header.frame_id = "/world";
    tags.header.stamp = ros::Time::now();
    tags.ns = "tags";
    tags.id=0;
    tags.action = visualization_msgs::Marker::ADD;
    tags.pose.orientation.w = 1.0;
    tags.scale.x=0.1;
    tags.scale.y=0.1;
    tags.scale.z=0.1;
    tags.color.g=1.0;
    tags.color.a=1.0;
    // set for estimate_path
    estimate_path.type = visualization_msgs::Marker::LINE_STRIP;
    estimate_path.header.frame_id  = "/world";
    estimate_path.header.stamp  = ros::Time::now();
    estimate_path.ns = "paths";
    estimate_path.id = 0;
    estimate_path.action = visualization_msgs::Marker::ADD;
    estimate_path.pose.orientation.w = 1.0;
    estimate_path.scale.x = 0.01;
    estimate_path.color.r=1.0;
    estimate_path.color.a=1.0;
    // set for robot particle

}
void iSAMprocessor::AddNodeRobot2Robot(tf::Transform t){
    // Create new node and put into robot_pose_nodes
    Pose2d_Node* new_pose_node = new Pose2d_Node();
    slam.add_node(new_pose_node);
    robot_pose_nodes.push_back(new_pose_node);
    // get the Pose2d from t
    double roll,pitch,yaw;
    t.getBasis().getEulerYPR(yaw, pitch, roll);
    Pose2d odometry(t.getOrigin().getX(), t.getOrigin().getY(),yaw);
    // Create constraint
    Pose2d_Pose2d_Factor* constraint = new Pose2d_Pose2d_Factor(robot_pose_nodes[robot_pose_nodes.size()-2],
                                     robot_pose_nodes[robot_pose_nodes.size()-1], odometry, noise3);
    // Add constraint to slam
    slam.add_factor(constraint);
}
void iSAMprocessor::AddNodesRobot2Marks(){
    for(int i = 0; i < new_posearray.poses.size(); i++){
        int id = FindID(new_posearray.poses[i].header.frame_id);
        // If did not find id in list than add new point
        if(id == tag_pose_nodes.size()){
            Pose2d_Node* new_point_node = new Pose2d_Node();
            slam.add_node(new_point_node);
            tag_pose_nodes.push_back(new_point_node);
        }

        // Create new measure
        tf::Transform new_pose;
        tf::Vector3 position;
        tf::Quaternion orientation;
        orientation=tf::Quaternion(new_posearray.poses[i].pose.orientation.x, new_posearray.poses[i].pose.orientation.y,
                                   new_posearray.poses[i].pose.orientation.z, new_posearray.poses[i].pose.orientation.w);;
        position=tf::Vector3(new_posearray.poses[i].pose.position.x,new_posearray.poses[i].pose.position.y,
                             new_posearray.poses[i].pose.position.z);
        new_pose.setRotation(orientation);
        new_pose.setOrigin(position);
        double roll,pitch,yaw;
        new_pose.getBasis().getEulerYPR(yaw, pitch, roll);
        Pose2d measure(new_pose.getOrigin().getX(), new_pose.getOrigin().getY(), yaw);
        // Create measurement
        Pose2d_Pose2d_Factor * measurement =
                new Pose2d_Pose2d_Factor(robot_pose_nodes[robot_pose_nodes.size() - 1],
                                         tag_pose_nodes[id], measure, noise3);
        // Add measurement to slam
        slam.add_factor(measurement);
    }
}
void iSAMprocessor::PublishMarker(){
    // Clear the estimate_path
    estimate_path.points.clear();
    // Set the current time
    tags.header.stamp = estimate_path.header.stamp  = ros::Time::now();
    // Add each node to estimate_path after slam update
    for(int i=1; i < robot_pose_nodes.size(); i++){
        geometry_msgs::Point p;
        p.x = robot_pose_nodes[i]->value().x();
        p.y = robot_pose_nodes[i]->value().y();
        estimate_path.points.push_back(p);
    }
    // Clear tags
    tags.points.clear();
    // Add each tag to tags after slam update
    for(int i = 0; i < tag_pose_nodes.size(); i++){
        geometry_msgs::Point p;
        p.x = tag_pose_nodes[i]->value().x();
        p.y = tag_pose_nodes[i]->value().y();
        tags.points.push_back(p);
    }
    // Publish the markers
    marker_pub.publish(tags);
    marker_pub.publish(estimate_path);
}
void iSAMprocessor::Processing(){
    // Angle between two pose, (angle * axis at z) is the actual angle of turtlebot
    double angle = new_orientation.getAngle() * new_orientation.getAxis().getZ() -
                   old_orientation.getAngle() * new_orientation.getAxis().getZ();
    // Distance between two pose
    double distance = new_position.distance(old_position);
    // If the changes meets the restrain, moving distance or angle larger than setted value
    if(distance > 0.03 || abs(angle) > 0.01){
        tf::Transform tf_test;
        tf_test.mult(old_transform.inverse(),new_transform);
        AddNodeRobot2Robot(tf_test);
        AddNodesRobot2Marks();
        // Save the new to the old
        old_position = new_position;
        old_orientation = new_orientation;
        old_transform = new_transform;
        // Update the map
        slam.update();
        PublishMarker();
    }
}
// Find the landmark index number base on ID string.
int iSAMprocessor::FindID(string ID){
// Get the landmark number
    for(int i=0;i<ID_array.size();i++){
        if(ID==ID_array[i]){
            return i;
        }
    }
    ID_array.push_back(ID);
    return ID_array.size()-1;
}
// Save the processed mapping information to the file example: run1.txt
void iSAMprocessor::Finish(string name){
    // write the map to map.yaml, just save the (x y theta) of apriltags
    ofstream fout("map.yaml");
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "tagNumber";
    out << YAML::Value << tag_pose_nodes.size();
    for(int i = 0; i < tag_pose_nodes.size(); i++){
        string index = std::to_string(i);
        double x = tag_pose_nodes[i]->value().x();
        double y = tag_pose_nodes[i]->value().y();
        double theta = tag_pose_nodes[i]->value().t();
        string id = ID_array[i];
        out << YAML::Key << "tag" + index;
        out << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "id" << YAML::Value << id;
        out << YAML::Key << "x" << YAML::Value << x;
        out << YAML::Key << "y" << YAML::Value << y;
        out << YAML::Key << "theta" << YAML::Value << theta;
        out << YAML::EndMap;
    }
    out << YAML::EndMap;
    fout << out.c_str();
    // write the slam result to file
    slam.batch_optimization();
    slam.print_graph();
    slam.save(name);
}


