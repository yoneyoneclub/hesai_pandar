#pragma once

#include <array>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include "pandar_pointcloud/calibration.hpp"
#include "packet_decoder.hpp"
#include "pandar_xt16.hpp"

namespace pandar_pointcloud
{
namespace pandar_xt16
{
class PandarXT16Decoder : public PacketDecoder
{
public:
  enum class ReturnMode : int8_t
  {
    DUAL,
    FIRST,
    STRONGEST,
    LAST,
  };

  PandarXT16Decoder(rclcpp::Node & node, Calibration& calibration, float scan_phase = 0.0f, double dual_return_distance_threshold = 0.1, ReturnMode return_mode = ReturnMode::DUAL, const std::vector<long>& disable_rings = {});
  void unpack(const pandar_msgs::msg::PandarPacket& raw_packet) override;
  bool hasScanned() override;
  PointcloudXYZIRADT getPointcloud() override;

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  bool parsePacket(const pandar_msgs::msg::PandarPacket& raw_packet);
  PointcloudXYZIRADT convert(const int block_id);
  PointcloudXYZIRADT convert_dual(const int block_id);

  std::array<float, UNIT_NUM> elev_angle_;
  std::array<float, UNIT_NUM> azimuth_offset_;

  std::array<float, UNIT_NUM> firing_offset_;
  std::array<float, BLOCK_NUM> block_offset_single_;
  std::array<float, BLOCK_NUM> block_offset_dual_;
  std::vector<long> disable_rings_;

  ReturnMode return_mode_;
  Packet packet_;

  PointcloudXYZIRADT scan_pc_;
  PointcloudXYZIRADT overflow_pc_;

  uint16_t scan_phase_;
  int last_phase_;
  bool has_scanned_;
};

}  // namespace pandar_xt16
}  // namespace pandar_pointcloud
