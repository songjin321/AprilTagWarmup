/*********************************************************************
* Author: Johnny Sun
*********************************************************************/
#include <tmcl.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>


namespace tmcl {
tmcl::tmcl(tf::TransformListener& tf) :
    tf_(tf),
  {
    ros::NodeHandle nh;

    tagPose_odom_sub_ = nh.subscribe<apriltag_checkout_tag::PoseStampedArray>("/tagPose_odom", 1, &tmcl::dataCB, this);
 

    real_observation=[0,0,0]; %[index of landmark,x,y]
    ideal_observation=real_observation;
    real_observation_relative=real_observation;
    current_observation=real_observation;
    real_v=1;%robto real v
    real_omega=1;%robot real omega
    robot_pose=[2,2,pi/2];%Robot real pose
    robot_vel=[0.5,-pi/4];%Robot real speed
    orig_pose=robot_pose;
    orig_vel=robot_vel;
    dt=0.05;
    real_traj=orig_pose;
    odom_u=robot_vel;%odometry represented by v and omega
    R_observation=0.5;%variance on observation;
    R_odom=[0.02,0.05];% times
    R_compare=0.1;
    N_particle=200;
    bel_pose=[0,0,0];
    
    pose_base=orig_pose;
    randxy=normrnd(2,0.2,N_particle,2)+0.2;
    randtheta=normrnd(pi/2,0.3,N_particle,2)+0.3;
    
    % randxy=(rand(N_particle,2)-0.5)*2+pose_base(1:2);
    % randtheta=(rand(N_particle,1)-0.5)*pi/2+pose_base(3);
    pose_particle=[randxy randtheta ones(N_particle,1)*1/N_particle];
    robot_particle_bel=pose_particle;
    it=0;
    
  }

  tmcl::~tmcl(){

  }

  // geometry_msgs::PoseStamped tmcl::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
  //   std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
  //   tf::Stamped<tf::Pose> goal_pose, global_pose;
  //   poseStampedMsgToTF(goal_pose_msg, goal_pose);

  //   //just get the latest available transform... for accuracy they should send
  //   //goals in the frame of the planner
  //   goal_pose.stamp_ = ros::Time();

  //   try{
  //     tf_.transformPose(global_frame, goal_pose, global_pose);
  //   }
  //   catch(tf::TransformException& ex){
  //     ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
  //         goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
  //     return goal_pose_msg;
  //   }

  //   geometry_msgs::PoseStamped global_pose_msg;
  //   tf::poseStampedTFToMsg(global_pose, global_pose_msg);
  //   return global_pose_msg;
  // }

  // double tmcl::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  // {
  //   return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  // }


void tmcl::dataCB(const apriltag_checkout_tag::PoseStampedArrayConstPtr& _tagPose_odom_array){

  new_posearray.header=_tagPose_odom_array->header;
  new_posearray.poses=_tagPose_odom_array->poses;


  
  new_position=tf::Vector3(_tagPose_odom_array->odom.twist.twist.linear.x,
                            _tagPose_odom_array->odom.twist.twist.linear.y,);

  landmark_observation.clear();                          
  landmark_observation.insert(_tagPose_odom_array.poses.begin(),
                        _tagPose_odom_array.begin(),_tagPose_odom_array.end());
  
  odom_last = _tagPose_odom_array->odom;
}

};
