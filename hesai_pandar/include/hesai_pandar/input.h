#pragma once
#include <pandar_msgs/PandarPacket.h>


class Input
{
public:
  virtual ~Input(){};
  virtual int getPacket(pandar_msgs::PandarPacket * pkt) = 0;
};