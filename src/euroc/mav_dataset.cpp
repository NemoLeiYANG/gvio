#include "gvio/euroc/mav_dataset.hpp"

namespace gvio {

int IMUData::load(const std::string &data_dir) {
  const std::string imu_data_path = data_dir + "/data.csv";
  const std::string imu_calib_path = data_dir + "/sensor.yaml";

  // Load IMU data
  MatX data;
  if (csv2mat(imu_data_path, true, data) != 0) {
    LOG_ERROR("Failed to load IMU data [%s]!", imu_data_path.c_str());
    return -1;
  }

  const double t0 = data(0, 0);
  for (long i = 0; i < data.rows(); i++) {
    this->timestamps.emplace_back(data(i, 0));
    this->time.emplace_back((data(i, 0) - t0) * 1e-9);
    this->w_B.emplace_back(data(i, 1), data(i, 2), data(i, 3));
    this->a_B.emplace_back(data(i, 4), data(i, 5), data(i, 6));
  }

  // Load calibration data
  ConfigParser parser;
  parser.addParam("sensor_type", &this->sensor_type);
  parser.addParam("comment", &this->comment);
  parser.addParam("T_BS", &this->T_BS);
  parser.addParam("rate_hz", &this->rate_hz);
  parser.addParam("gyroscope_noise_density", &this->gyro_noise_density);
  parser.addParam("gyroscope_random_walk", &this->gyro_random_walk);
  parser.addParam("accelerometer_noise_density", &this->accel_noise_density);
  parser.addParam("accelerometer_random_walk", &this->accel_random_walk);
  if (parser.load(imu_calib_path) != 0) {
    LOG_ERROR("Failed to load sensor file [%s]!", imu_calib_path.c_str());
    return -1;
  }

  return 0;
}

std::ostream &operator<<(std::ostream &os, const IMUData &data) {
  // clang-format off
  os << "sensor_type: " << data.sensor_type << std::endl;
  os << "comment: " << data.comment << std::endl;
  os << "T_BS:\n" << data.T_BS << std::endl;
  os << "rate_hz: " << data.rate_hz << std::endl;
  os << "gyroscope_noise_density: " << data.gyro_noise_density << std::endl;
  os << "gyroscope_random_walk: " << data.gyro_random_walk << std::endl;
  os << "accelerometer_noise_density: " << data.accel_noise_density << std::endl;
  os << "accelerometer_random_walk: " << data.accel_random_walk << std::endl;
  // clang-format on

  return os;
}

int CameraData::load(const std::string &data_dir) {
  const std::string cam_data_path = data_dir + "/data.csv";
  const std::string cam_calib_path = data_dir + "/sensor.yaml";

  // Load camera data
  MatX data;
  if (csv2mat(cam_data_path, true, data) != 0) {
    LOG_ERROR("Failed to load camera data [%s]!", cam_data_path.c_str());
    return -1;
  }
  const double t0 = data(0, 0);
  for (long i = 0; i < data.rows(); i++) {
    const std::string image_file = std::to_string((long) data(i, 0)) + ".png";
    this->timestamps.emplace_back(data(i, 0));
    this->time.emplace_back((data(i, 0) - t0) * 1e-9);
    this->image_paths.emplace_back(data_dir + "/data/" + image_file);
  }

  // Load calibration data
  ConfigParser parser;
  parser.addParam("sensor_type", &this->sensor_type);
  parser.addParam("comment", &this->comment);
  parser.addParam("T_BS", &this->T_BS);
  parser.addParam("rate_hz", &this->rate_hz);
  parser.addParam("resolution", &this->resolution);
  parser.addParam("camera_model", &this->camera_model);
  parser.addParam("intrinsics", &this->intrinsics);
  parser.addParam("distortion_model", &this->distortion_model);
  parser.addParam("distortion_coefficients", &this->distortion_coefficients);
  if (parser.load(cam_calib_path) != 0) {
    LOG_ERROR("Failed to load senor file [%s]!", cam_calib_path.c_str());
    return -1;
  }

  return 0;
}

std::ostream &operator<<(std::ostream &os, const CameraData &data) {
  // clang-format off
  os << "sensor_type: " << data.sensor_type << std::endl;
  os << "comment: " << data.comment << std::endl;
  os << "T_BS:\n" << data.T_BS << std::endl;
  os << "rate_hz: " << data.rate_hz << std::endl;
  os << "resolution: " << data.resolution.transpose() << std::endl;
  os << "camera_model: " << data.camera_model << std::endl;
  os << "intrinsics: " << data.intrinsics.transpose() << std::endl;
  os << "distortion_model: " << data.distortion_model << std::endl;
  os << "distortion_coefficients: " << data.distortion_coefficients.transpose() << std::endl;
  // clang-format on

  return os;
}

int GroundTruthData::load(const std::string &data_dir) {
  // Load ground truth data
  const std::string gnd_data_path = data_dir + "/data.csv";
  MatX data;
  if (csv2mat(gnd_data_path, true, data) != 0) {
    LOG_ERROR("Failed to load ground truth data [%s]!", gnd_data_path.c_str());
    return -1;
  }

  const double t0 = data(0, 0);
  for (long i = 0; i < data.rows(); i++) {
    this->timestamps.emplace_back(data(i, 0));
    this->time.emplace_back((data(i, 0) - t0) * 1e-9);
    this->p_RS_R.emplace_back(data(i, 1), data(i, 2), data(i, 3));
    this->q_RS.emplace_back(data(i, 4), data(i, 5), data(i, 6), data(i, 7));
    this->v_RS_R.emplace_back(data(i, 8), data(i, 9), data(i, 10));
    this->b_w_RS_S.emplace_back(data(i, 11), data(i, 12), data(i, 13));
    this->b_a_RS_S.emplace_back(data(i, 14), data(i, 15), data(i, 16));
  }

  return 0;
}

int MAVDataset::loadIMUData() {
  const std::string imu_data_dir = this->data_path + "/imu0";
  if (this->imu_data.load(imu_data_dir) != 0) {
    LOG_ERROR("Failed to load IMU data [%s]!", imu_data_dir.c_str());
    return -1;
  }

  return 0;
}

int MAVDataset::loadCameraData() {
  const std::string cam0_dir = this->data_path + "/cam0";
  if (this->cam0_data.load(cam0_dir) != 0) {
    LOG_ERROR("Failed to load cam0 data [%s]!", cam0_dir.c_str());
    return -1;
  }

  const std::string cam1_dir = this->data_path + "/cam1";
  if (this->cam1_data.load(cam1_dir) != 0) {
    LOG_ERROR("Failed to load cam1 data [%s]!", cam1_dir.c_str());
    return -1;
  }

  return 0;
}

int MAVDataset::loadGroundTruthData() {
  const std::string gnd_dir = this->data_path + "/state_groundtruth_estimate0";
  if (this->ground_truth.load(gnd_dir) != 0) {
    LOG_ERROR("Failed to load ground truth data !");
    return -1;
  }

  return 0;
}

int MAVDataset::load() {
  if (this->loadIMUData() != 0) {
    LOG_ERROR("Failed to load imu data!");
    return -1;
  }

  if (this->loadCameraData() != 0) {
    LOG_ERROR("Failed to load camera data!");
    return -1;
  }

  if (this->loadGroundTruthData() != 0) {
    LOG_ERROR("Failed to load ground truth data!");
    return -1;
  }

  this->ok = true;
  return 0;
}

} // namespace gvio
