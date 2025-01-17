#include "pandar_pointcloud/decoder/pandar_128_e4x_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar_128_e4x.hpp"

namespace
{
static inline float deg2rad(float degrees)
{
  return degrees * M_PI / 180.0f;
}
}  // namespace

namespace pandar_pointcloud
{
namespace pandar_128_e4x
{
Pandar128E4XDecoder::Pandar128E4XDecoder(Calibration& calibration,
                                         float scan_phase,
                                         double dual_return_distance_threshold,
                                         ReturnMode return_mode)
{
  for (uint8_t laser = 0; laser < LASER_COUNT; ++laser) {
    elev_angle_[laser] = calibration.elev_angle_map[laser];
    azimuth_offset_[laser] = calibration.azimuth_offset_map[laser];
  }

  uint8_t i = 0;
  for (const auto &angle:elev_angle_){
    auto rads = deg2rad(angle);
    elev_angle_rad_[i] = rads;
    cos_elev_angle_[i] = cosf(rads);
    sin_elev_angle_[i++] = sinf(rads);
  }

  scan_phase_ = static_cast<uint16_t>(scan_phase * 100.0f);
  dual_return_distance_threshold_ = dual_return_distance_threshold;

  last_phase_ = 0;
  has_scanned_ = false;

  scan_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
  scan_pc_->reserve(LASER_COUNT*MAX_AZIMUTH_STEPS);
  overflow_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
  overflow_pc_->reserve(LASER_COUNT*MAX_AZIMUTH_STEPS);
}

bool Pandar128E4XDecoder::hasScanned()
{
  return has_scanned_;
}

PointcloudXYZIRADT Pandar128E4XDecoder::getPointcloud()
{
  return scan_pc_;
}

bool Pandar128E4XDecoder::parsePacket(const pandar_msgs::PandarPacket& raw_packet)
{
  if (raw_packet.size != sizeof(Packet)) {
    std::cerr << "Packet size mismatch:" << raw_packet.size
              << "| Expected:" << sizeof(Packet) << std::endl;
    return false;
  }
  if (std::memcpy(&packet_, raw_packet.data.data(), sizeof(Packet))) {
    return true;
  }
  std::cerr << "Invalid SOF " << std::hex << packet_.header.SOP << " Packet" << std::endl;
  return false;
}

void Pandar128E4XDecoder::unpack(const pandar_msgs::PandarPacket& raw_packet)
{
  if (!parsePacket(raw_packet)) {
    return;
  }
  if (has_scanned_) {
    scan_pc_ = overflow_pc_;
    overflow_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
    overflow_pc_->reserve(LASER_COUNT*MAX_AZIMUTH_STEPS);
    has_scanned_ = false;
  }

  bool dual_return = false;
  if (packet_.tail.return_mode == DUAL_LAST_STRONGEST_RETURN
      || packet_.tail.return_mode == DUAL_LAST_FIRST_RETURN
      || packet_.tail.return_mode == DUAL_FIRST_STRONGEST_RETURN) {
    dual_return = true;
  }

  auto block_pc = convert();
  int current_phase =
      (static_cast<int>(packet_.body.azimuth_1) - scan_phase_ + 36000) % 36000;
  if (current_phase > last_phase_ && !has_scanned_) {
    *scan_pc_ += *block_pc;
  } else {
    *overflow_pc_ += *block_pc;
    has_scanned_ = true;
  }
  last_phase_ = current_phase;
}

PointXYZIRADT Pandar128E4XDecoder::build_point(const Block& block,
                                               const size_t& laser_id,
                                               const uint16_t& azimuth,
                                               const double& unix_second)
{

  PointXYZIRADT point{};

  float xyDistance = static_cast<float>(block.distance) * DISTANCE_UNIT * cos_elev_angle_[laser_id];

  //TODO: Create HASH TABLE to accelerate deg2rad
  point.x = (
      xyDistance *
          sinf(deg2rad(azimuth_offset_[laser_id] + (static_cast<float>(azimuth)) / 100.0)));
  point.y = (
      xyDistance *
          cosf(deg2rad(azimuth_offset_[laser_id] + (static_cast<float>(azimuth)) / 100.0)));
  point.z = static_cast<float>(block.distance * DISTANCE_UNIT * sin_elev_angle_[laser_id]);

  point.intensity = block.reflectivity;
  point.distance = xyDistance;
  point.ring = laser_id;
  point.azimuth = static_cast<float>(azimuth/100.0f) + azimuth_offset_[laser_id];
  point.return_type = 0; // TODO
  point.time_stamp = unix_second + packet_.tail.timestamp_us * 1e-6;

  return point;
}

PointcloudXYZIRADT Pandar128E4XDecoder::convert()
{
  PointcloudXYZIRADT block_pc(new pcl::PointCloud<PointXYZIRADT>);
  block_pc->reserve(LASER_COUNT*2);
  struct tm t = {};
  t.tm_year = packet_.tail.date_time.year;
  t.tm_mon = packet_.tail.date_time.month - 1;
  t.tm_mday = packet_.tail.date_time.day;
  t.tm_hour = packet_.tail.date_time.hour;
  t.tm_min = packet_.tail.date_time.minute;
  t.tm_sec = packet_.tail.date_time.second;
  t.tm_isdst = 0;
  auto unix_second = static_cast<double>(timegm(&t));

  for(size_t i= 0; i < LASER_COUNT; i++) {
    auto block1_pt = build_point(packet_.body.block_01[i],
                                 i,
                                 packet_.body.azimuth_1,
                                 unix_second);

    auto block2_pt = build_point(packet_.body.block_02[i],
                                 i,
                                 packet_.body.azimuth_2,
                                 unix_second);
    if (block1_pt.distance >= MIN_RANGE && block1_pt.distance <= MAX_RANGE) {
      block_pc->points.emplace_back(block1_pt);
    }
    if (block2_pt.distance >= MIN_RANGE && block2_pt.distance <= MAX_RANGE) {
      block_pc->points.emplace_back(block2_pt);
    }
  }
  return block_pc;
}

PointcloudXYZIRADT Pandar128E4XDecoder::convert_dual()
{
  PointcloudXYZIRADT block_pc(new pcl::PointCloud<PointXYZIRADT>);
  struct tm t = {};
  t.tm_year = packet_.tail.date_time.year;
  t.tm_mon = packet_.tail.date_time.month - 1;
  t.tm_mday = packet_.tail.date_time.day;
  t.tm_hour = packet_.tail.date_time.hour;
  t.tm_min = packet_.tail.date_time.minute;
  t.tm_sec = packet_.tail.date_time.second;
  t.tm_isdst = 0;

  for(size_t i= 0; i < LASER_COUNT; i++) {
    block_pc->points.emplace_back(
        build_point(packet_.body.block_01[i],
                    i,
                    packet_.body.azimuth_1,
                    static_cast<double>(timegm(&t)))
    );
    // TODO check the second block and compare with first
  }

  return block_pc;
}



}  // namespace pandar40
}  // namespace pandar_pointcloud