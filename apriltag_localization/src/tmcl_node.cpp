/*
 * Johnny Sun @NRSL
 *
 */
 
#include <tmcl.h>
using namespace Tmcl;

int main(int argc, char** argv){

  ros::init(argc, argv, "apriltag_localization_node");
  tf::TransformListener tf(ros::Duration(10));
  Tmcl::tmcl tmcl( tf );
  
  ros::spin();
  return(0);
}