#include "gvio/dataset/euroc/imu_data.hpp"

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

  const long t0 = data(0, 0);
  for (long i = 0; i < data.rows(); i++) {
    const long ts = data(i, 0);
    this->timestamps.push_back(ts);
    this->time.push_back((ts - t0) * 1e-9);
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

} // namespace gvio
