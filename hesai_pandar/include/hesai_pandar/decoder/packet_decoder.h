#pragma once

#include <fstream>
#include <vector>
#include <pandar_msgs/PandarPacket.h>
#include "hesai_pandar/input.h"
#include "hesai_pandar/point_types.h"


class PacketDecoder
{
public:
  virtual ~PacketDecoder(){};
  virtual void unpack(const pandar_msgs::PandarPacket & raw_packet) = 0;
  virtual bool hasScanned() = 0;
  virtual PointcloudXYZIRADT getPointcloud() = 0;
};