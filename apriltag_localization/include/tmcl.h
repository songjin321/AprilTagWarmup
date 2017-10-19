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
#include <stdlib.h>
#include "apriltag_mapping/apriltag_mapping.h"
#include <boost/math/distributions/normal.hpp>
#include <boost/random.hpp>
#include <yaml-cpp/yaml.h>
//#include "glog/logging.h"
//end debug

using namespace std;

// aprilTag
struct AprilTag {
    unsigned int id;
    double x;
    double y;
    double theta;
};
// Get map information from map file
void getMap(string mapFile, vector<AprilTag> &aprilTags);
//　Use id as an index find corresponding AprilTag
void getAprilTag(const vector<AprilTag> &aprilTags, unsigned int id, AprilTag &re);
// calculate the angle and distance
void get_odom_angle_translation(nav_msgs::Odometry &old_odom, nav_msgs::Odometry &new_odom,
                                double &angle, double &distance);

namespace Tmcl {

 typedef enum {
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
    RobotParticle()
    {}
      ~RobotParticle()
      {}
    void MotionModel(double* odom_u)
    {
      x += odom_u[0];
      y += odom_u[1];
      theta += odom_u[2];
    }
    bool operator < (const RobotParticle &m)const {
      return w < m.w;
    }
    bool operator > (const RobotParticle &m)const {
      return w > m.w;
    }
  };
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
    std::vector<RobotParticle> particles;
    std::vector<RobotParticle> particles_resampled;
    int particle_N;
    tmclState State;
    double R_odom[3];
    int CBcount;
    double R_compare;
    double robot_pose[3];

    double odom_u[3];
    /*
      R: standard deviation
    */
    int relocalize(double *robot_pose_, double* R);
    void transformRobot2World(RobotParticle &p, std::vector<AprilTag> &tag_r, std::vector<AprilTag> &tag_w);
    void resample(std::vector<RobotParticle> &particles);
    void normalize(std::vector<RobotParticle> &particles);
    void calcRobotPose(int front_num);

  private:
    //double distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2);

    //void SendNavigation_Failed(int resualt);
    //geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped &goal_pose_msg);
  
    tf::TransformListener &tf_;

    ros::Publisher particle_pub_,robot_pose_pub_;
    ros::Subscriber tagPose_odom_sub_;

    ros::Time last_control_;
    ros::Time dt;

    nav_msgs::Odometry odom_last,odom_new;

    std::vector<AprilTag> landmark_observation;
    //std::vector<AprilTag> landmark_observation_world;
    std::vector<AprilTag> aprilTag_map;
    
    void dataCB(const apriltag_checkout_tag::PoseStampedArrayConstPtr &_tagPose_odom_array);
  };
};

#endif

