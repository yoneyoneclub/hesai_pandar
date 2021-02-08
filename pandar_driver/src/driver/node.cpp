#include <ros/ros.h>
#include <pandar_driver/pandar_driver.h>


int main(int argc, char * argv[])
{
  ros::init(argc, argv, "pandar_driver_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  pandar_driver::PandarDriver driver(node, private_nh);

  // loop until shut down or end of file
  while (ros::ok() && driver.poll()) {
    ros::spinOnce();
  }

  return 0;
}
