/*********************************************************************
* Author: Johnny Sun
*********************************************************************/
#include <tmcl.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

namespace Tmcl {
tmcl::tmcl(tf::TransformListener& tf) :
    tf_(tf)
  {
    ros::NodeHandle nh;
    State = INIT;
    //current_observation = [ 0, 0, 0 ];
    landmark_observation.clear();
    landmark_observation.reserve(10);
    //real_observation;
    aprilTag_map.clear();
    memset(robot_pose, 0, sizeof(robot_pose));
    //robot_vel=[0.5,-pi/4];
    CBcount = 0;
    last_control_= ros::Time::now();
    memset(odom_u,0,sizeof(odom_u));//odometry represented by dx,dy and domega
    srand((int)(ros::Time::now().toSec()*10000));
    R_odom[0] = 0.02;
    R_odom[1] = 0.05;
  R_compare = 0.1;

  // AprilTag tag;
  // tag.id = i;
  // tag.x = _tag["x"].as<double>();
  // tag.y = _tag["y"].as<double>();
  // tag.theta = _tag["theta"].as<double>();
  // Get map information from map file
  getMap("map", aprilTag_map);
  //　Use id as an index find corresponding AprilTag
  double R_loc[3] = {1, 1, 2};
  relocalize(robot_pose, R_loc);
  //particle_pub_ = nh.advertise<std::vector<geometry_msgs::PoseStamped>>("/robot_particles", 1);
  robot_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/robot_pose", 1);

  tagPose_odom_sub_ = nh.subscribe<apriltag_checkout_tag::PoseStampedArray>("/tagPose_odom", 1, &tmcl::dataCB, this);
  State = INIT;
  }

  tmcl::~tmcl(){

  }

  int tmcl::relocalize(double *robot_pose_, double* R)
  {
    CBcount = 0;
    particle_N = 1000;
    particles.clear();
    particles.reserve(particle_N);

    boost::mt19937 rng;
    boost::normal_distribution<> ndx(0,R[0]);
    boost::variate_generator<boost::mt19937&, 
    boost::normal_distribution<> > var_norx(rng, ndx);

    boost::normal_distribution<> ndy(0,R[1]);
    boost::variate_generator<boost::mt19937&, 
    boost::normal_distribution<> > var_nory(rng, ndy);

    boost::normal_distribution<> ndth(0,R[2]);
    boost::variate_generator<boost::mt19937&, 
    boost::normal_distribution<> > var_north(rng, ndth);

    //初始撒点
    for(int i=0;i<particle_N;i++)
    {
      RobotParticle p;
      p.x=robot_pose_[0]+var_norx();
      p.y=robot_pose_[1]+var_nory();
      p.theta=robot_pose_[2]+var_north();
      p.w = 1 / particle_N;
      particles.push_back(p);
    }
  }
  void tmcl::transformRobot2World(RobotParticle &p,std::vector<AprilTag> &tag_r,std::vector<AprilTag> &tag_w)
  {
    tag_w.clear();
    if (!tag_r.empty())
    {
      for (int i = 0; i < tag_r.size();i++)
      {
        double cs = cos(tag_r[i].theta);
        double sn = sin(tag_r[i].theta);
        AprilTag tag = tag_r[i];
        tag.x = p.x + tag_r[i].x * cs - tag_r[i].y * sn;
        tag.y = p.y + tag_r[i].y * cs + tag_r[i].x * sn;
        tag.theta = p.theta + tag_r[i].theta;
        tag_w.push_back(tag);
      }
    }
    //T=[cos(a),-sin(a),x;sin(a),cos(a),y;0,0,1];
  }
  void get_odom_trans(nav_msgs::Odometry &old_odom, nav_msgs::Odometry &new_odom, double &dx,double &dy,double &dtheta)
  {
      tf::Quaternion old_orientation(old_odom.pose.pose.orientation.x,
                            old_odom.pose.pose.orientation.y,
                            old_odom.pose.pose.orientation.z,
                            old_odom.pose.pose.orientation.w);
      tf::Quaternion new_orientation(new_odom.pose.pose.orientation.x,
                            new_odom.pose.pose.orientation.y,
                            new_odom.pose.pose.orientation.z,
                            new_odom.pose.pose.orientation.w);
      // tf::Vector3 old_position(old_odom.pose.pose.position.x,
      //                           old_odom.pose.pose.position.y,
      //                           old_odom.pose.pose.position.z);
      // tf::Vector3 new_position(new_odom.pose.pose.position.x,
      //                           new_odom.pose.pose.position.y,
      //                           new_odom.pose.pose.position.z);
      // Angle between two pose, (angle * axis at z) is the actual angle of turtlebot
      dtheta = new_orientation.getAngle() * new_orientation.getAxis().getZ() -
                      old_orientation.getAngle() * new_orientation.getAxis().getZ();
      dx = new_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
      dy = new_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
      // Distance between two pose
      //distance = new_position.distance(old_position);
  }
  double get_angle(geometry_msgs::PoseStamped &pose_)
  {
      tf::Quaternion orientation_(pose_.pose.orientation.x,
        pose_.pose.orientation.y,
        pose_.pose.orientation.z,
        pose_.pose.orientation.w);
      return orientation_.getAngle() * orientation_.getAxis().getZ();
  }
  void tmcl::normalize(std::vector<RobotParticle> &particles)
  {
    double w_sum = 0;
    for (int i = 0; i < particles.size(); i++)
    {
      w_sum += particles[i].w;
    }
    for (int i = 0; i < particles.size(); i++)
    {
      particles[i].w /=w_sum;
    }
  }
  void tmcl::resample(std::vector<RobotParticle> &particles)
  {
    std::vector<RobotParticle> particles_t(particles);
    double one_particle_N = 1 / (double)particle_N;
    double w_set = rand() * one_particle_N;
    double i_set = 0;
    double i_observed = 0;
    double w_current = 0;
    while ((w_set <= 1) && (i_observed < particle_N))
    {
      i_observed++;
      w_current = w_current + particles_t[i_observed].w;
      while (w_current > w_set)
      {
        i_set++;
      particles[i_set] = particles_t[i_observed];
      w_set = w_set + one_particle_N;
      }
    }
  }
  void tmcl::calcRobotPose(int front_num)
  {
    double w_sum = 0;
    double x_sum=0;
    double y_sum=0;
    double th_sum=0;
    if (front_num>=particles.size())
      front_num = particles.size();
    for (int i = 0; i < front_num; i++)
    {
      w_sum += particles[i].w;
    }
    double w_scale = 0;
    for (int i = 0; i < front_num; i++)
    {
      w_scale = particles[i].w / w_sum;
      x_sum += particles[i].x *w_scale;
      y_sum += particles[i].y*w_scale;
      th_sum += particles[i].theta*w_scale;
    }
    robot_pose[0] = x_sum;
    robot_pose[1] = y_sum;
    robot_pose[2] = th_sum;
  }

  void tmcl::dataCB(const apriltag_checkout_tag::PoseStampedArrayConstPtr &_tagPose_odom_array)
  {
    if(CBcount++==0)
    {
      odom_last = _tagPose_odom_array->odom;
    }
    odom_new=_tagPose_odom_array->odom;
    //获取位置增量
    get_odom_trans(odom_last, odom_new, odom_u[0], odom_u[1], odom_u[2]);
    odom_last = odom_new;

    //获取观测量
    landmark_observation.clear();
    geometry_msgs::PoseStamped tag_pose;
    if(!_tagPose_odom_array->poses.empty())
    {
      for (int i = 0; i < _tagPose_odom_array->poses.size();i++)
      {
        tag_pose = _tagPose_odom_array->poses[i];
        AprilTag tag;
        istringstream in_(tag_pose.header.frame_id);  
        int id_;
        in_ >> id_;
        tag.id = id_;
        tag.x = tag_pose.pose.position.x;
        tag.y = tag_pose.pose.position.y;
        tag.theta = get_angle(tag_pose);
        landmark_observation.push_back(tag);
      }
    }
    //对当前粒子进行一次航迹推算，考虑误差
    for (int i = 0; i < particle_N;i++)
    {
      particles[i].MotionModel(odom_u);
      double odom_v[3] = {
        rand() * R_odom[0],
        rand() * R_odom[1],
        rand() * R_odom[2]};
      particles[i].MotionModel(odom_v);
    }
    double w_sum = 0;
    if(0!=landmark_observation.size())
    {//有观测
      //考虑每个粒子
      for (int i = 0; i < particle_N;i++)
      {
        std::vector<AprilTag> current_tag_w;
        //转换 当前粒子 机器人坐标系下的观测 到 全局坐标系下
        transformRobot2World(particles[i], landmark_observation, current_tag_w);
        //计算观测到的与地图中的tag误差
        double e_sum=0;
        for (int j = 0; j < current_tag_w.size();j++)
        {//考虑每个观测
            AprilTag tag_map;
            getAprilTag(aprilTag_map, current_tag_w[j].id, tag_map);

            double dx=(current_tag_w[j].x-tag_map.x);
            double dy=(current_tag_w[j].y-tag_map.y);
            double dd=dx*dx+dy*dy;

            boost::math::normal_distribution<> nd(0, R_compare);  
            double e=boost::math::pdf(nd, dd);
            e_sum=e_sum+e;
        }
        particles[i].w = e_sum / current_tag_w.size();
        //求权重和
        w_sum += particles[i].w;
      }
    }
    //考虑每个粒子
    for (int i = 0; i < particle_N; i++)
    {
      //归一化
      particles[i].w /= w_sum;
    }
    //sort particle according to its w
    std::sort(particles.begin(), particles.end(), greater<RobotParticle>());

    //robot_particle_resampled = robot_particle_observed;

    //resampling
    resample(particles);
    normalize(particles);
    calcRobotPose(particles.size()/5);

    geometry_msgs::Quaternion rb_quat = tf::createQuaternionMsgFromYaw(robot_pose[2]);
    geometry_msgs::PoseStamped robot_pose_t;

    robot_pose_t.header.stamp=ros::Time::now();
    robot_pose_t.header.frame_id="/world";
    robot_pose_t.pose.orientation.x = rb_quat.x;
    robot_pose_t.pose.orientation.y = rb_quat.y;
    robot_pose_t.pose.orientation.z = rb_quat.z;
    robot_pose_t.pose.orientation.w = rb_quat.w;
    robot_pose_t.pose.position.x = robot_pose[0];
    robot_pose_t.pose.position.y = robot_pose[1];
    robot_pose_t.pose.position.z = 0;

    //particle_pub_.publish();   // = nh.advertise<geometry_msgs::PoseStamped>("/robot_particles",1);
    robot_pose_pub_.publish(robot_pose_t); //= nh.advertise<geometry_msgs::PoseStamped>("/robot_pose",1);
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


};

void getMap(string mapFile, vector<AprilTag> &aprilTags) {
  aprilTags.clear();
  YAML::Node map = YAML::LoadFile(mapFile);
  YAML::Node tagNumberNode = map["tagNumber"];
  int tagNumber = map["tagNumber"].as<int>();
  for (std::size_t i = 0; i < tagNumber; i++) {
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
void getAprilTag(const vector<AprilTag> &aprilTags, unsigned int id, AprilTag &re)
{
  auto re_ite = find_if(aprilTags.begin(), aprilTags.end(), [id](const AprilTag &a){
      return id == a.id;
  });
  if (re_ite != aprilTags.end())
    re = *re_ite;
}
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