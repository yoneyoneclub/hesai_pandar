#pragma once
#include <map>

struct Calibration
{
  std::map<int, float> elev_angle_map;
  std::map<int, float> azimuth_offset_map;

  Calibration();
  int loadFile(const std::string & calibration_file);
  int loadContent(const std::string & calibration_content);
  int saveFile(const std::string & calibration_file);
};