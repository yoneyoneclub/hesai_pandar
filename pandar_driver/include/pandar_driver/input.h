#pragma once
#include <pandar_msgs/PandarPacket.h>

namespace pandar_driver
{
class Input
{
public:
  virtual ~Input(){};
  virtual int getPacket(pandar_msgs::PandarPacket * pkt) = 0;
};
}  // namespace pandar_driver
