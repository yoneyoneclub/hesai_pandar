#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <string>
#include <thread>

#include <pandar_driver/pandar_driver.h>

namespace pandar_driver
{
class DriverNodelet : public nodelet::Nodelet
{
public:
  DriverNodelet() : running_(false) {}

  ~DriverNodelet()
  {
    NODELET_INFO("shutting down driver thread");
    running_ = false;
    if(deviceThread_.joinable()){
      deviceThread_.join();
    }
    NODELET_INFO("driver thread stopped");
  }

private:
  virtual void onInit(void);
  virtual void devicePoll(void);

  volatile bool running_;
  std::thread deviceThread_;
  std::shared_ptr<PandarDriver> driver_;
};

void DriverNodelet::onInit()
{
  driver_.reset(new PandarDriver(getNodeHandle(), getPrivateNodeHandle()));

  running_ = true;
  deviceThread_ = std::thread(&DriverNodelet::devicePoll, this);
}

void DriverNodelet::devicePoll()
{
  while (ros::ok()) {
    running_ = driver_->poll();
    if (!running_) break;
  }
  running_ = false;
}

}  // namespace pandar_driver

PLUGINLIB_EXPORT_CLASS(pandar_driver::DriverNodelet, nodelet::Nodelet)
