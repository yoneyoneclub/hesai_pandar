#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"

using namespace std;

class HesaiLidarClient
{
public:
  HesaiLidarClient(ros::NodeHandle node, ros::NodeHandle nh)
  {
    pandar_packet_pub_ = node.advertise<hesai_lidar::PandarScan>("pandar_packets",10);
    pandar_points_pub_ = node.advertise<sensor_msgs::PointCloud2>("pandar_points", 10);
    pandar_points_ex_pub_ = node.advertise<sensor_msgs::PointCloud2>("pandar_points_ex", 10);

    string device_ip;
    int lidar_port;
    int gps_port;
    double scan_phase;
    string calibration;  // Get local correction when getting from lidar failed
    string model;
    string frame_id;
    int time_zone;
    string pcap;

    nh.getParam("pcap", pcap);
    nh.getParam("device_ip", device_ip);
    nh.getParam("lidar_port", lidar_port);
    nh.getParam("gps_port", gps_port);
    nh.getParam("scan_phase", scan_phase);
    nh.getParam("calibration", calibration);
    nh.getParam("model", model);
    nh.getParam("frame_id", frame_id);
    nh.getParam("time_zone", time_zone);
    nh.getParam("publish_packet", publish_packet_);
    nh.getParam("timestamp_type", timestamp_type_);

    if(!pcap.empty()){
      sdk_.reset(new PandarGeneralSDK(pcap, boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(scan_phase * 100 + 0.5), time_zone, model, frame_id, timestamp_type_));
    }
    else if (!publish_packet_){
      sdk_.reset(new PandarGeneralSDK("", boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(scan_phase * 100 + 0.5), time_zone, model, frame_id, timestamp_type_));
      if (sdk_ != nullptr) {
        ROS_INFO("Subscribe pandar_packets");
        packetSubscriber = node.subscribe("pandar_packets", 10, &HesaiLidarClient::scanCallback, (HesaiLidarClient*)this, ros::TransportHints().tcpNoDelay(true));
      }
    }
    else {
      sdk_.reset(new PandarGeneralSDK(device_ip, lidar_port, gps_port, \
        boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
        NULL, static_cast<int>(scan_phase * 100 + 0.5), time_zone, model, frame_id, timestamp_type_));
    }

    if (sdk_ != nullptr) {
        sdk_->Start();
        if(!calibration.empty()){
          ROS_INFO("Load calibration data from csv");
          sdk_->LoadLidarCorrectionFile(loadCalibrationString(calibration));
        }
    } else {
        ROS_ERROR("create sdk fail\n");
    }
  }


  void lidarCallback(pcl::PointCloud<PointXYZIRADT>::Ptr cld, double timestamp, hesai_lidar::PandarScanPtr scan) // the timestamp from first point cloud of cld
  {
    if(publish_packet_){
      pandar_packet_pub_.publish(scan);
      // printf("raw size: %zu.\n", scan->packets.size());
    }
    cld->header.stamp = pcl_conversions::toPCL(ros::Time(timestamp));
    pandar_points_ex_pub_.publish(cld);

    const auto points_xyzir = convert(cld);
    pandar_points_pub_.publish(points_xyzir);      
    // printf("timestamp: %f, point size: %ld.\n",timestamp, cld->points.size());
  }
  

  void scanCallback(const hesai_lidar::PandarScanPtr scan)
  {
    // printf("pandar_packets topic message received,\n");
    sdk_->PushScanPacket(scan);
  }

  pcl::PointCloud<PointXYZIR>::Ptr convert(
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

  std::string loadCalibrationString(std::string lidarCorrectionFile)
  {
    ifstream fin(lidarCorrectionFile);
    int length = 0;
    std::string strlidarCalibration;
    fin.seekg(0, std::ios::end);
    length = fin.tellg();
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[length];
    fin.read(buffer, length);
    fin.close();
    strlidarCalibration = buffer;
    return strlidarCalibration;
  }

private:
  ros::Publisher pandar_packet_pub_;
  ros::Publisher pandar_points_pub_;
  ros::Publisher pandar_points_ex_pub_;

  // PandarGeneralSDK* hsdk;
  boost::shared_ptr<PandarGeneralSDK> sdk_;
  string timestamp_type_;
  bool publish_packet_;
  ros::Subscriber packetSubscriber;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pandar");
  ros::NodeHandle nh("~");
  ros::NodeHandle node;
  HesaiLidarClient pandarClient(node, nh);

  ros::spin();
  return 0;
}
