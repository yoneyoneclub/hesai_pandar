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
    packetPublisher = node.advertise<hesai_lidar::PandarScan>("pandar_packets",10);
    lidarPublisher = node.advertise<sensor_msgs::PointCloud2>("pandar_points", 10);
    lidarExPublisher = node.advertise<sensor_msgs::PointCloud2>("pandar_points_ex", 10);

    string serverIp;
    int lidarRecvPort;
    int gpsPort;
    double startAngle;
    string lidarCorrectionFile;  // Get local correction when getting from lidar failed
    string model;
    string frame_id;
    int timeZone;
    int pclDataType;
    string pcapFile;
    string dataType;

    nh.getParam("pcap_file", pcapFile);
    nh.getParam("server_ip", serverIp);
    nh.getParam("lidar_recv_port", lidarRecvPort);
    nh.getParam("gps_port", gpsPort);
    nh.getParam("start_angle", startAngle);
    nh.getParam("lidar_correction_file", lidarCorrectionFile);
    nh.getParam("model", model);
    nh.getParam("frame_id", frame_id);
    nh.getParam("time_zone", timeZone);
    nh.getParam("pcldata_type", pclDataType);
    nh.getParam("publish_type", m_sPublishType);
    nh.getParam("timestamp_type", m_sTimestampType);
    nh.getParam("data_type", dataType);

    if(!pcapFile.empty()){
      hsdk = new PandarGeneralSDK(pcapFile, boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(startAngle * 100 + 0.5), timeZone, pclDataType, model, frame_id, m_sTimestampType);
      if (hsdk != NULL) {
        // ifstream fin(lidarCorrectionFile);
        // int length = 0;
        // std::string strlidarCalibration;
        // fin.seekg(0, std::ios::end);
        // length = fin.tellg();
        // fin.seekg(0, std::ios::beg);
        // char *buffer = new char[length];
        // fin.read(buffer, length);
        // fin.close();
        // strlidarCalibration = buffer;
        // hsdk->LoadLidarCorrectionFile(strlidarCalibration);
      }
    }
    else if ("rosbag" == dataType){
      hsdk = new PandarGeneralSDK("", boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(startAngle * 100 + 0.5), timeZone, pclDataType, model, frame_id, m_sTimestampType);
      if (hsdk != NULL) {
        // ifstream fin(lidarCorrectionFile);
        // int length = 0;
        // std::string strlidarCalibration;
        // fin.seekg(0, std::ios::end);
        // length = fin.tellg();
        // fin.seekg(0, std::ios::beg);
        // char *buffer = new char[length];
        // fin.read(buffer, length);
        // fin.close();
        // strlidarCalibration = buffer;
        // hsdk->LoadLidarCorrectionFile(strlidarCalibration);
        packetSubscriber = node.subscribe("pandar_packets", 10, &HesaiLidarClient::scanCallback, (HesaiLidarClient*)this, ros::TransportHints().tcpNoDelay(true));
      }
    }
    else {
      hsdk = new PandarGeneralSDK(serverIp, lidarRecvPort, gpsPort, \
        boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
        NULL, static_cast<int>(startAngle * 100 + 0.5), timeZone, pclDataType, model, frame_id, m_sTimestampType);
    }
    
    if (hsdk != NULL) {
        hsdk->Start();
        // hsdk->LoadLidarCorrectionFile("...");  // parameter is stream in lidarCorrectionFile
    } else {
        printf("create sdk fail\n");
    }
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

  void lidarCallback(pcl::PointCloud<PointXYZIRADT>::Ptr cld, double timestamp, hesai_lidar::PandarScanPtr scan) // the timestamp from first point cloud of cld
  {
    if(m_sPublishType == "both" || m_sPublishType == "points"){
      cld->header.stamp = pcl_conversions::toPCL(ros::Time(timestamp));
      lidarExPublisher.publish(cld);

      const auto points_xyzir = convert(cld);
      lidarPublisher.publish(points_xyzir);
      
      printf("timestamp: %f, point size: %ld.\n",timestamp, cld->points.size());
    }
    if(m_sPublishType == "both" || m_sPublishType == "raw"){
      packetPublisher.publish(scan);
      printf("raw size: %zu.\n", scan->packets.size());
    }
  }

  void scanCallback(const hesai_lidar::PandarScanPtr scan)
  {
    // printf("pandar_packets topic message received,\n");
    hsdk->PushScanPacket(scan);
  }

private:
  ros::Publisher packetPublisher;
  ros::Publisher lidarPublisher;
  ros::Publisher lidarExPublisher;

  PandarGeneralSDK* hsdk;
  string m_sPublishType;
  string m_sTimestampType;
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
