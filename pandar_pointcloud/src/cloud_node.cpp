#include <ros/ros.h>
#include "pandar_pointcloud/pandar_cloud.hpp"


int main(int argc, char * argv[])
{
  ros::init(argc, argv, "pandar_cloud_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  pandar_pointcloud::PandarCloud conv(node, private_nh);

  ros::spin();

  return 0;
}
