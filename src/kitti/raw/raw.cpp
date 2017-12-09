#include "gvio/kitti/raw.hpp"

namespace gvio {

int RawDataset::loadCalibrations() {
  const std::string base_dir = this->raw_dir + "/" + this->date;
  this->calib_cam_to_cam.load(base_dir + "/calib_cam_to_cam.txt");
  this->calib_imu_to_cam.load(base_dir + "/calib_imu_to_cam.txt");
  this->calib_velo_to_cam.load(base_dir + "/calib_velo_to_cam.txt");

  return 0;
}

int RawDataset::load() {
  // Pre-check
  const std::string dataset_path = this->raw_dir + "/" + this->date;
  if (dir_exists(dataset_path) == false) {
    LOG_ERROR("Raw dataset path not found! [%s]", this->raw_dir.c_str());
    return -1;
  }

  // Load calibrations
  if (this->loadCalibrations() != 0) {
    LOG_ERROR("Failed to load calibrations!");
    return -2;
  }

  return 0;
}

} // namespace gvio
