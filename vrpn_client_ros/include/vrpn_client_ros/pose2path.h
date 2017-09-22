//
// Created by sj on 17-8-18.
//

#ifndef VRPN_CLIENT_ROS_POSE2PATH_H
#define VRPN_CLIENT_ROS_POSE2PATH_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
/*this class subscribe the orign topic published by vrpn and convert it to path
 * in world coordinate(robot initialize coordinate))*/
namespace vrpn_client_ros
{
    class Pose2Path
    {
    public:
        Pose2Path(ros::NodeHandle nh)
        {
        //    first_tf.setIdentity();
            initRobot = true;
            path_sub = nh.subscribe("/mavros/vision_pose/pose", 1, &Pose2Path::addPoseStampedCallback, this);
            path_pub = nh.advertise<nav_msgs::Path>("/true_path", 1);
        }
    private:
        void addPoseStampedCallback(const geometry_msgs::PoseStamped& msg)
        {
            static tf::TransformBroadcaster br;
            static tf::TransformListener listener;
            tf::Transform tr;
            tf::StampedTransform stamped_tr;
            tf::poseMsgToTF(msg.pose,tr);
            if(initRobot) {
                initTransform = tr;
                initRobot = false;
                return;
            }
            br.sendTransform(tf::StampedTransform(initTransform, ros::Time::now(), "/vrpn_coordinate", "/world"));
            br.sendTransform(tf::StampedTransform(tr, ros::Time::now(), "/vrpn_coordinate", "/vrpn"));
            //br.sendTransform(tf::StampedTransform(initTransform.inverse() * tr, msg.header.stamp, "/world", "/vrpn"));
            try{
                listener.lookupTransform("/world","/vrpn",ros::Time(0),stamped_tr);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            geometry_msgs::PoseStamped _poseStamped;
            _poseStamped.pose.position.x = stamped_tr.getOrigin().getX();
            _poseStamped.pose.position.y = stamped_tr.getOrigin().getY();
            _poseStamped.pose.position.z = 0;
            _poseStamped.header = msg.header;
            _poseStamped.header.frame_id = "/world";
            _path.header.frame_id = "/world";
            _path.poses.push_back(_poseStamped);
            path_pub.publish(_path);
        }
        bool initRobot;
        nav_msgs::Path _path;
        ros::Publisher path_pub;
        ros::Subscriber path_sub;
        tf::Transform initTransform;
    };
}
#endif //VRPN_CLIENT_ROS_POSE2PATH_H
