#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pandar_msgs/PandarScan.h>
#include "hesai_pandar/input.h"
#include "hesai_pandar/tcp_command_client.h"
#include "hesai_pandar/decoder/packet_decoder.h"
#include "hesai_pandar/calibration.h"


class HesaiPandarNode
{
public:
  HesaiPandarNode();
  ~HesaiPandarNode();

private:
  int setupCalibration();
  void receivePacket();
  void processPacket();
  void onScan(const pandar_msgs::PandarScan::ConstPtr & msg);
  pcl::PointCloud<PointXYZIR>::Ptr convertPointcloud(
    const pcl::PointCloud<PointXYZIRADT>::ConstPtr & input_pointcloud);

  std::string device_ip_;
  int lidar_port_;
  int gps_port_;
  double scan_phase_;
  bool publish_packet_;

  std::string model_;
  std::string frame_id_;
  std::string calibration_path_;
  std::string pcap_path_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher pandar_packet_pub_;
  ros::Publisher pandar_points_pub_;
  ros::Publisher pandar_points_ex_pub_;
  ros::Subscriber pandar_packet_sub_;

  std::shared_ptr<Input> input_;
  std::shared_ptr<PacketDecoder> decoder_;
  std::shared_ptr<TcpCommandClient> tcp_client_;
  Calibration calibration_;

  bool enable_thread_;
  std::thread receive_thread_;
  std::thread process_thread_;
  std::mutex packet_mutex_;
  std::condition_variable packet_condition_;

  std::deque<pandar_msgs::PandarPacket> packet_queue_;
};