/*********************************************************************
* Author: Johnny Sun
*********************************************************************/
#ifndef TMCL_H_
#define TMCL_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <apriltag_checkout_tag/PoseStampedArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <boost/math/distributions/normal.hpp>

//#include "glog/logging.h"
//end debug

using namespace std;

namespace tmcl {

  enum tmclState {
    INIT,
    LOST,
    CONVERGING,
    STABLE,
  }tmclState;

  class RobotParticle {
  public:
    double x;
    double y;
    double theta;
    double w; //weight or possibility of this particle
    velocitymodel();

  }

  class NormalDistribute {
  public:
    double x;
    double y;
    double theta;
    double w; //weight or possibility of this particle
    velocitymodel();

  }
  /**
   * @class tmcl
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  class tmcl
  {
  public:
    /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
    tmcl(tf::TransformListener &tf);

    /**
       * @brief  Destructor - Cleans up
       */
    virtual ~tmcl();
    RobotParticle *particles;
    int particle_N;
    tmclState State = INIT;
    double R_odom;
    double R_compare;

  private:

    //double distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2);

    //void SendNavigation_Failed(int resualt);
    //geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped &goal_pose_msg);

    tf::TransformListener &tf_;

    ros::Publisher particle_pub_,robot_pose_;
    ros::Subscriber tagPose_odom_sub_;

    ros::Time last_control_;

    nav_msgs::Odometry odom_last;

    std::vector<geometry_msgs::PoseStamped_> landmark_observation;
    std::vector<geometry_msgs::PoseStamped_> landmark_world;
    std::vector<int> landmark_index;
    //set up plan triple buffer
    //std::vector<geometry_msgs::PoseStamped> *planner_plan_;

    //set up the planner's thread
    // bool runPlanner_;
    // boost::mutex planner_mutex_;
    // boost::condition_variable planner_cond_;
    // geometry_msgs::PoseStamped planner_goal_;
    // boost::thread *planner_thread_;

    // boost::recursive_mutex configuration_mutex_;
    // dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;

    // void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

    // move_base::MoveBaseConfig last_config_;
    // move_base::MoveBaseConfig default_config_;
    // bool setup_, p_freq_change_, c_freq_change_;
    // bool new_global_plan_;
    // bool bumper_antidrop_stop_, emergent_stop_;
    // //debug
    // int transition;
  };
};
#endif

