#include "gvio/dataset/euroc/camera_data.hpp"

namespace gvio {

int CameraData::load(const std::string &data_dir) {
  const std::string cam_data_path = data_dir + "/data.csv";
  const std::string cam_calib_path = data_dir + "/sensor.yaml";

  // Load camera data
  MatX data;
  if (csv2mat(cam_data_path, true, data) != 0) {
    LOG_ERROR("Failed to load camera data [%s]!", cam_data_path.c_str());
    return -1;
  }

  const long t0 = data(0, 0);
  for (long i = 0; i < data.rows(); i++) {
    const std::string image_file = std::to_string((long) data(i, 0)) + ".png";
    const std::string image_path = data_dir + "/data/" + image_file;
    const long ts = data(i, 0);

    if (file_exists(image_path) == false) {
      LOG_ERROR("File [%s] does not exist!", image_path.c_str());
      return -1;
    }

    this->timestamps.emplace_back(ts);
    this->time.emplace_back((ts - t0) * 1e-9);
    this->image_paths.emplace_back(image_path);
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

} // namespace gvio
