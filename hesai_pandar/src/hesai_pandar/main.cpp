#include <ros/ros.h>
#include <hesai_pandar/hesai_pandar_node.h>


int main(int argc, char * argv[])
{
  ros::init(argc, argv, "hesai_pandar");

  HesaiPandarNode node;

  ros::spin();

  return 0;
}
