#include "gvio/dataset/kitti/raw/raw.hpp"

namespace gvio {

int RawDataset::loadCalibrations() {
  int retval;
  std::string calib_path;

  // Camera to camera calibration
  calib_path = this->date_dir + "/calib_cam_to_cam.txt";
  retval = this->calib_cam_to_cam.load(calib_path);
  CHECK(retval == 0, "Failed to load camera to camera calibration!");

  // IMU to Velo calibration
  calib_path = this->date_dir + "/calib_imu_to_velo.txt";
  retval = this->calib_imu_to_velo.load(calib_path);
  CHECK(retval == 0, "Failed to load imu to velocalibration!");

  // Velo to camera calibration
  calib_path = this->date_dir + "/calib_velo_to_cam.txt";
  retval = this->calib_velo_to_cam.load(calib_path);
  CHECK(retval == 0, "Failed to load velo to camera calibration!");

  return 0;
error:
  return -1;
}

int RawDataset::loadImagePaths() {
  // Get list of image paths
  if (list_dir(this->drive_dir + "/image_00/data", this->cam0) != 0) {
    return -1;
  }
  if (list_dir(this->drive_dir + "/image_01/data", this->cam1) != 0) {
    return -1;
  }
  if (list_dir(this->drive_dir + "/image_02/data", this->cam2) != 0) {
    return -1;
  }
  if (list_dir(this->drive_dir + "/image_03/data", this->cam3) != 0) {
    return -1;
  }

  // Sort image paths
  std::sort(this->cam0.begin(), this->cam0.end());
  std::sort(this->cam1.begin(), this->cam1.end());
  std::sort(this->cam2.begin(), this->cam2.end());
  std::sort(this->cam3.begin(), this->cam3.end());

  // Add full path to files
  for (size_t i = 0; i < this->cam0.size(); i++) {
    this->cam0[i] = this->drive_dir + "/image_00/data/" + this->cam0[i];
    this->cam1[i] = this->drive_dir + "/image_01/data/" + this->cam1[i];
    this->cam2[i] = this->drive_dir + "/image_02/data/" + this->cam2[i];
    this->cam3[i] = this->drive_dir + "/image_03/data/" + this->cam3[i];
  }

  return 0;
}

int RawDataset::loadOXTS() {
  const std::string oxts_dir = this->drive_dir + "/oxts";
  if (this->oxts.load(oxts_dir) != 0) {
    LOG_ERROR("Failed to load OXTS data!");
    return -1;
  }

  return 0;
}

int RawDataset::load() {
  // Pre-check
  if (dir_exists(this->date_dir) == false) {
    LOG_ERROR("Raw dataset path not found! [%s]", this->date_dir.c_str());
    return -1;
  }

  // Load
  if (this->loadCalibrations() != 0) {
    LOG_ERROR("Failed to load calibrations!");
    return -2;
  }
  if (this->loadImagePaths() != 0) {
    LOG_ERROR("Failed to load image paths!");
    return -3;
  }
  if (this->loadOXTS() != 0) {
    LOG_ERROR("Failed to load OXTS data!");
    return -4;
  }

  this->ok = true;
  return 0;
}

} // namespace gvio
