/*
 * Johnny Sun @NRSL
 *
 */

#include <tmcl.h>
#include <glog/logging.h>
using namespace Tmcl;
int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "apriltag_localization_node");
    tf::TransformListener tf(ros::Duration(10));
    Tmcl::tmcl tmcl( tf );

    ros::spin();
    return(0);
}