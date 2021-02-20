#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <memory>

#include "pandar_pointcloud/pandar_cloud.hpp"

namespace pandar_pointcloud
{
class CloudNodelet : public nodelet::Nodelet
{
public:
  CloudNodelet() {}
  ~CloudNodelet() {}

private:
  virtual void onInit(void);
  std::shared_ptr<PandarCloud> cloud_;
};

void CloudNodelet::onInit()
{
  cloud_.reset(new PandarCloud(getNodeHandle(), getPrivateNodeHandle()));
}

}  // namespace pandar_pointcloud

PLUGINLIB_EXPORT_CLASS(pandar_pointcloud::CloudNodelet, nodelet::Nodelet)
