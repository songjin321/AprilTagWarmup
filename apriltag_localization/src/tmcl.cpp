/*********************************************************************
* Author: Johnny Sun
*********************************************************************/
#include <tmcl.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#define GOOGLE_STRIP_LOG 0
#include <glog/logging.h>
#include <chrono>
#include <random>
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
static std::default_random_engine generator(seed);
inline void pause_my()
{
    do
    {
        cout << '\n' << "Press a key to continue...";
    } while (cin.get() != '\n');
}
namespace Tmcl {
    tmcl::tmcl(tf::TransformListener& tf) :
            tf_(tf)
    {
        ros::NodeHandle nh;
        State = INIT;
        //current_observation = [ 0, 0, 0 ];
        landmark_observation.clear();
        //real_observation;
        aprilTag_map.clear();
        memset(robot_pose, 0, sizeof(robot_pose));
        //robot_vel=[0.5,-pi/4];
        CBcount = 0;
        last_control_= ros::Time::now();
        memset(odom_u,0,sizeof(odom_u));//odometry represented by dx,dy and domega

        R_odom[0] = 0.05;
        R_odom[1] = 0.05;
        R_odom[2] = 0.01;
        R_compare = 1;

        robot_pose[0] = 0;
        robot_pose[1] = 0;
        robot_pose[2] = 0;

        InitMarker();
        // Get map information from map file
        getMap("/home/sj/map.yaml", aprilTag_map);
        LOG(INFO) << "read map normal" << endl;
        //　Use id as an index find corresponding AprilTag
        relocalize();
        particle_pub_ = nh.advertise<visualization_msgs::Marker>("/robot_particles", 1);
        robot_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/robot_pose", 1);

        tagPose_odom_sub_ = nh.subscribe<apriltag_checkout_tag::PoseStampedArray>("/tagPose_odom", 1, &tmcl::dataCB, this);
        State = INIT;
    }

    tmcl::~tmcl(){

    }
    void tmcl::InitMarker(){
        tags.type=visualization_msgs::Marker::POINTS;
        tags.header.frame_id = "/world";
        tags.header.stamp = ros::Time::now();
        tags.ns = "tags";
        tags.id=0;
        tags.action = visualization_msgs::Marker::ADD;
        tags.pose.orientation.w = 1.0;
        tags.scale.x=0.05;
        tags.scale.y=0.05;
        tags.scale.z=0.05;
        tags.color.g=1.0;
        tags.color.a=1.0;
        // set for estimate_path
        robotPosition.type = visualization_msgs::Marker::POINTS;
        robotPosition.header.frame_id  = "/world";
        robotPosition.header.stamp  = ros::Time::now();
        robotPosition.ns = "position";
        robotPosition.id = 0;
        robotPosition.action = visualization_msgs::Marker::ADD;
        robotPosition.pose.orientation.w = 1.0;
        robotPosition.scale.x = 0.1;
        robotPosition.scale.y = 0.1;
        robotPosition.scale.z = 0.1;
        robotPosition.color.r=1.0;
        robotPosition.color.a=1.0;
    }
    int tmcl::relocalize()
    {
        CBcount = 0;
        particle_N = 1000;
        particles.clear();
        particles.reserve(particle_N);

        std::uniform_real_distribution<double> distribution(-3, 3);//3

        //初始撒点
        for(int i=0;i<particle_N;i++)
        {
            RobotParticle p;
            p.x=robot_pose[0]+distribution(generator);
            p.y=robot_pose[1]+distribution(generator);
            p.theta=robot_pose[2]+distribution(generator);
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
        std::uniform_real_distribution<double> distribution(0.0,one_particle_N);
        double r = distribution(generator);
        double c = particles_t[0].w;
        int i = 0;
        for (int m = 0; m < particle_N; m++)
        {
            double U = r + m*one_particle_N;
            while(U > c)
            {
                i = i + 1;
                c = c + particles_t[i].w;
            }
            particles[m] = particles_t[i];
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
     //   output("init");
     //   pause_my();
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
        std::normal_distribution<double> distribution_x(0.0, R_odom[0]);
        std::normal_distribution<double> distribution_y(0.0, R_odom[1]);
        std::normal_distribution<double> distribution_theta(0.0, R_odom[2]);
        for (int i = 0; i < particle_N;i++)
        {
            particles[i].MotionModel(odom_u);
            double odom_v[3] = {
                    distribution_x(generator),
                    distribution_y(generator),
                    distribution_theta(generator)};
            particles[i].MotionModel(odom_v);
        }
     //   output("PDR");
     //   pause_my();
        LOG(INFO) << "PDR OK" << endl;
        double w_sum = 0;
        if( 0 != landmark_observation.size())
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
                    if (&tag_map == nullptr)
                    {
                        cout << "no correspand id" << endl;
                        continue;
                    }
                    double dx=(current_tag_w[j].x-tag_map.x);
                    double dy=(current_tag_w[j].y-tag_map.y);
                    double dd=dx*dx+dy*dy;
                    double e = normal_pdf(dd, 0, R_compare);
                    e_sum=e_sum+e;
                }
                particles[i].w = e_sum / current_tag_w.size();
                //求权重和
                w_sum += particles[i].w;
            }
            if (w_sum == 0)
            {
                cout << "landmarkSize = " << landmark_observation.size()
                     << " w_sum = " << w_sum << endl;
            }
            //考虑每个粒子
            for (int i = 0; i < particle_N; i++)
            {
                //归一化
                    particles[i].w /= w_sum;
            }

            //resampling
            resample(particles);
            LOG(INFO) << "resample OK" << endl;
            normalize(particles);
            //   output("resample");
            //   pause_my();
            LOG(INFO) << "normalize OK" << endl;
        }
     //   output("Observe");
     //   pause_my();
        LOG(INFO) << "Observe OK" << endl;
        //sort particle according to its w form big to little
        std::sort(particles.begin(), particles.end(),
                [](const RobotParticle &a, const RobotParticle &b)
                {
                    return a.w > b.w;
                });
        for_each(particles.begin(), particles.begin() + 10, [](const RobotParticle &a)
                                                     {
                                                         cout << "w = " << a.w << endl;
                                                     });
        calcRobotPose(particles.size()/20);
     //   CHECK(robot_pose[0] < 10) << "robot x failed";
     //   CHECK(robot_pose[1] < 10) << "robot y failed";
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

        output("result");
        robot_pose_pub_.publish(robot_pose_t); //= nh.advertise<geometry_msgs::PoseStamped>("/robot_pose",1);
        LOG(INFO) << "CBcount = " << CBcount << endl;
    }
    void tmcl::output(string state){

        // Set the current time
        tags.header.stamp = robotPosition.header.stamp  = ros::Time::now();
        robotPosition.header.stamp = robotPosition.header.stamp  = ros::Time::now();

        // Clear particles
        tags.points.clear();
        // Add each particle to particles after slam update
        for(int i = 0; i < particles.size(); i++){
            geometry_msgs::Point p;
            p.x = particles[i].x;
            p.y = particles[i].y;
            p.z = particles[i].w * 20;
            tags.points.push_back(p);
        //    LOG_EVERY_N(WARNING, 100) << google::COUNTER <<" "<< state + " output"
        //                           << " x = " << p.x << " y = " << p.y
        //                              <<" w = " << p.z << endl;
        }

        robotPosition.points.clear();
        // Add point to estimate_path
        geometry_msgs::Point p;
        p.x = robot_pose[0];
        p.y = robot_pose[1];
        robotPosition.points.push_back(p);

        // Publish the markers
        particle_pub_.publish(tags);
        particle_pub_.publish(robotPosition);
    }
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
        tag.id = _tag["id"].as<int>();
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
double normal_pdf(double x, double u, double sigma)
{
    return exp(-(x-u)*(x-u)/(2*sigma*sigma))/(sigma*sqrt(2*M_PI));
}

