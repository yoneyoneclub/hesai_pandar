#include "hesai_pandar/hesai_pandar_node.h"
#include <pandar_msgs/PandarScan.h>
#include "hesai_pandar/calibration.h"
#include "hesai_pandar/pcap_input.h"
#include "hesai_pandar/socket_input.h"
#include "hesai_pandar/decoder/pandar40_decoder.h"
#include "hesai_pandar/decoder/pandar_qt_decoder.h"

namespace
{
const uint16_t TCP_COMMAND_PORT = 9347;
const size_t TCP_RETRY_NUM = 5;
const size_t TCP_RETRY_WAIT = 100;
}  // namespace

HesaiPandarNode::HesaiPandarNode() : nh_(""), private_nh_("~")
{
  private_nh_.getParam("pcap", pcap_path_);
  private_nh_.getParam("device_ip", device_ip_);
  private_nh_.getParam("lidar_port", lidar_port_);
  private_nh_.getParam("gps_port", gps_port_);
  private_nh_.getParam("scan_phase", scan_phase_);
  private_nh_.getParam("calibration", calibration_path_);
  private_nh_.getParam("model", model_);
  private_nh_.getParam("frame_id", frame_id_);
  private_nh_.getParam("publish_packet", publish_packet_);

  pandar_packet_pub_ = nh_.advertise<pandar_msgs::PandarScan>("pandar_packets", 10);
  pandar_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pandar_points", 10);
  pandar_points_ex_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pandar_points_ex", 10);

  if (!pcap_path_.empty()) {
    input_.reset(new PcapInput(pcap_path_, model_));
  } else {
    input_.reset(new SocketInput(lidar_port_, gps_port_));
    tcp_client_ = std::make_shared<TcpCommandClient>(device_ip_, TCP_COMMAND_PORT);
  }

  if (setupCalibration() != 0) {
    ROS_ERROR("Unable to load calibration data");
    return;
  }

  if(model_ == "Pandar40P" || model_ == "Pandar40M"){
    decoder_ = std::make_shared<pandar40::Pandar40Decoder>(calibration_, scan_phase_, pandar40::Pandar40Decoder::ReturnMode::DUAL);
  }else if(model_ == "PandarQT"){
    decoder_ = std::make_shared<pandar_qt::PandarQTDecoder>(calibration_, scan_phase_);
  }else{
    // TODO : Add other models
    ROS_ERROR("Invalid model name");
    return;
  }


  enable_thread_ = true;
  if (publish_packet_) {
    receive_thread_ = std::thread(&HesaiPandarNode::receivePacket, this);
  } else {
    pandar_packet_sub_ = nh_.subscribe(
      "pandar_packets", 10, &HesaiPandarNode::onScan, this, ros::TransportHints().tcpNoDelay(true));
  }

  process_thread_ = std::thread(&HesaiPandarNode::processPacket, this);
}

HesaiPandarNode::~HesaiPandarNode()
{
  enable_thread_ = false;
  packet_condition_.notify_all();
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  if (process_thread_.joinable()) {
    process_thread_.join();
  }
  return;
}

int HesaiPandarNode::setupCalibration()
{
  if (tcp_client_) {
    std::string content("");
    for (size_t i = 0; i < TCP_RETRY_NUM; ++i) {
      auto ret = tcp_client_->getLidarCalibration(content);
      if (ret == TcpCommandClient::PTC_ErrCode::PTC_ERROR_NO_ERROR) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(TCP_RETRY_WAIT));
    }
    if (!content.empty()) {
      calibration_.loadContent(content);
      return 0;
    }
  }
  if (!calibration_path_.empty() && calibration_.loadFile(calibration_path_) == 0) {
    return 0;
  } else {
    return -1;
  }
}

void HesaiPandarNode::receivePacket()
{
  while (enable_thread_) {
    pandar_msgs::PandarPacket packet;
    int packet_type = input_->getPacket(&packet);
    if (packet_type == -1) {
      continue;
    } else {
      {
        std::lock_guard<std::mutex> lock(packet_mutex_);
        packet_queue_.push_back(packet);
      }
      packet_condition_.notify_all();
    }
  }
}

void HesaiPandarNode::processPacket()
{
  PointcloudXYZIRADT pointcloud;

  pandar_msgs::PandarPacket pkt;
  pandar_msgs::PandarScanPtr scan(new pandar_msgs::PandarScan);

  while (enable_thread_) {
    {
      std::unique_lock<std::mutex> lock(packet_mutex_);
      if (packet_queue_.empty()) {
        packet_condition_.wait(lock);
      }
      if (packet_queue_.empty()) {
        // go to check enable_thread_
        continue;
      }
      pkt = packet_queue_.front();
      packet_queue_.pop_front();
    }

    scan->packets.push_back(pkt);

    decoder_->unpack(pkt);
    if (decoder_->hasScanned()) {
      pointcloud = decoder_->getPointcloud();
      pointcloud->header.stamp =
        pcl_conversions::toPCL(ros::Time(pointcloud->points[0].time_stamp));
      pointcloud->header.frame_id = frame_id_;
      pointcloud->height = 1;

      pandar_points_ex_pub_.publish(pointcloud);
      if(pandar_points_pub_.getNumSubscribers() > 0){
        pandar_points_pub_.publish(convertPointcloud(pointcloud));
      }

      if (publish_packet_) {
        pandar_packet_pub_.publish(scan);
      }
      scan.reset(new pandar_msgs::PandarScan);
    }
  }
}

void HesaiPandarNode::onScan(const pandar_msgs::PandarScan::ConstPtr & msg)
{
  for (const auto & packet : msg->packets) {
    {
      std::lock_guard<std::mutex> lock(packet_mutex_);
      packet_queue_.push_back(packet);
    }
    packet_condition_.notify_all();
  }

  return;
}

pcl::PointCloud<PointXYZIR>::Ptr HesaiPandarNode::convertPointcloud(
  const pcl::PointCloud<PointXYZIRADT>::ConstPtr & input_pointcloud)
{
  pcl::PointCloud<PointXYZIR>::Ptr output_pointcloud(new pcl::PointCloud<PointXYZIR>);
  output_pointcloud->reserve(input_pointcloud->points.size());
  PointXYZIR point;
  for (const auto & p : input_pointcloud->points) {
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.intensity;
    point.ring = p.ring;
    output_pointcloud->points.push_back(point);
  }

  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}