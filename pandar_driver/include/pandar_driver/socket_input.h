#pragma once

#include <string>
#include <boost/asio.hpp>
#include "pandar_driver/input.h"

namespace pandar_driver
{
using boost::asio::ip::udp;

class SocketInput : public Input
{
public:
  SocketInput(const std::string& device_ip, uint16_t port, uint16_t gps_port, int timeout=1000);
  ~SocketInput();
  PacketType getPacket(pandar_msgs::PandarPacket* pkt) override;

private:
  // void on_receive();
  void checkDeadline();

  boost::asio::io_service io_service_;
  std::unique_ptr<udp::socket> lidar_socket_;
  std::unique_ptr<udp::socket> gps_socket_;
  std::unique_ptr<boost::asio::deadline_timer> deadline_;

  boost::asio::ip::address device_ip_;
  int timeout_;
};

}  // namespace pandar_driver
