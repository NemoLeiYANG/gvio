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

  for (int i = 0; i < data.rows(); i++) {
    this->timestamps.emplace_back(data(i, 0));
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
    LOG_ERROR("Failed to load configure file [%s]!", imu_calib_path.c_str());
    return -1;
  }

  return 0;
}

int MAVDataset::loadIMUData() {
  // Load IMU data
  const std::string imu_data_dir = this->data_path + "/imu0";
  if (this->imu_data.load(imu_data_dir) != 0) {
    LOG_ERROR("Failed to load IMU data [%s]!", imu_data_dir.c_str());
    return -1;
  }

  // clang-format off
  std::cout << "sensor_type: " << this->imu_data.sensor_type << std::endl;
  std::cout << "comment: " << this->imu_data.comment << std::endl;
  std::cout << "T_BS:\n" << this->imu_data.T_BS << std::endl;
  std::cout << "rate_hz: " << this->imu_data.rate_hz << std::endl;
  std::cout << "gyroscope_noise_density: " << this->imu_data.gyro_noise_density << std::endl;
  std::cout << "gyroscope_random_walk: " << this->imu_data.gyro_random_walk << std::endl;
  std::cout << "accelerometer_noise_density: " << this->imu_data.accel_noise_density << std::endl;
  std::cout << "accelerometer_random_walk: " << this->imu_data.accel_random_walk << std::endl;
  // clang-format on

  return 0;
}

int MAVDataset::loadCameraData() {
  // Get list of image paths for cam0
  if (list_dir(this->data_path + "/cam0/data", this->cam0) != 0) {
    return -1;
  }

  // Get list of image paths for cam1
  if (list_dir(this->data_path + "/cam1/data", this->cam1) != 0) {
    return -1;
  }

  // Sort image paths
  std::sort(this->cam0.begin(), this->cam0.end());
  std::sort(this->cam1.begin(), this->cam1.end());

  return 0;
}

int MAVDataset::load() { return 0; }

} // namespace gvio
