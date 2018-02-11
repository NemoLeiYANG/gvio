#include "gvio/dataset/euroc/mav_dataset.hpp"

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
    this->timestamps.emplace_back(data(i, 0));
    this->time.emplace_back((ts - t0) * 1e-9);
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
    this->q_RS.emplace_back(data(i, 5), data(i, 6), data(i, 7), data(i, 4));
    this->v_RS_R.emplace_back(data(i, 8), data(i, 9), data(i, 10));
    this->b_w_RS_S.emplace_back(data(i, 11), data(i, 12), data(i, 13));
    this->b_a_RS_S.emplace_back(data(i, 14), data(i, 15), data(i, 16));
  }

  return 0;
}

std::ostream &operator<<(std::ostream &os, const DatasetEvent &data) {
  if (data.type == IMU_EVENT) {
    os << "event_type: imu" << std::endl;
    os << "a_m: " << data.a_m.transpose() << std::endl;
    os << "w_m: " << data.w_m.transpose() << std::endl;
  } else if (data.type == CAMERA_EVENT) {
    os << "event_type: camera" << std::endl;
    os << "camera_index: " << data.camera_index << std::endl;
    os << "image_path: " << data.image_path << std::endl;
  }

  return os;
}

int MAVDataset::loadIMUData() {
  const std::string imu_data_dir = this->data_path + "/imu0";
  if (this->imu_data.load(imu_data_dir) != 0) {
    LOG_ERROR("Failed to load IMU data [%s]!", imu_data_dir.c_str());
    return -1;
  }

  for (size_t i = 0; i < this->imu_data.timestamps.size(); i++) {
    const long ts = this->imu_data.timestamps[i];
    const auto imu_event =
        DatasetEvent(this->imu_data.a_B[i], this->imu_data.w_B[i]);
    this->timeline.insert({ts, imu_event});
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

  for (size_t i = 0; i < this->cam0_data.timestamps.size(); i++) {
    const long ts = this->cam0_data.timestamps[i];
    const auto cam0_event = DatasetEvent(0, this->cam0_data.image_paths[i]);
    this->timeline.insert({ts, cam0_event});
  }

  for (size_t i = 0; i < this->cam1_data.timestamps.size(); i++) {
    const long ts = this->cam1_data.timestamps[i];
    const auto cam1_event = DatasetEvent(1, this->cam1_data.image_paths[i]);
    this->timeline.insert({ts, cam1_event});
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

long MAVDataset::minTimestamp() {
  const long cam0_first_ts = this->cam0_data.timestamps.front();
  const long imu_first_ts = this->imu_data.timestamps.front();
  const long gnd_first_ts = this->ground_truth.timestamps.front();

  std::vector<long> first_ts{cam0_first_ts, imu_first_ts, gnd_first_ts};
  auto first_result = std::min_element(first_ts.begin(), first_ts.end());
  const long first_ts_index = std::distance(first_ts.begin(), first_result);
  const long min_ts = first_ts[first_ts_index];

  return min_ts;
}

long MAVDataset::maxTimestamp() {
  const long cam0_last_ts = this->cam0_data.timestamps.back();
  const long imu_last_ts = this->imu_data.timestamps.back();
  const long gnd_last_ts = this->ground_truth.timestamps.back();

  std::vector<long> last_ts{cam0_last_ts, imu_last_ts, gnd_last_ts};
  auto last_result = std::max_element(last_ts.begin(), last_ts.end());
  const long last_ts_index = std::distance(last_ts.begin(), last_result);
  const long max_ts = last_ts[last_ts_index];

  return max_ts;
}

int MAVDataset::load() {
  // Load data
  if (this->loadCameraData() != 0) {
    LOG_ERROR("Failed to load camera data!");
    return -1;
  }
  if (this->loadIMUData() != 0) {
    LOG_ERROR("Failed to load imu data!");
    return -1;
  }
  if (this->loadGroundTruthData() != 0) {
    LOG_ERROR("Failed to load ground truth data!");
    return -1;
  }

  // Timestamp
  this->ts_start = this->minTimestamp();
  this->ts_end = this->maxTimestamp();
  this->ts_now = this->ts_start;

  // Get timestamps
  auto it = this->timeline.begin();
  auto it_end = this->timeline.end();
  while (it != it_end) {
    this->timestamps.push_back(it->first);
    it = this->timeline.upper_bound(it->first);
  }

  this->ok = true;
  return 0;
}

void MAVDataset::reset() {
  this->ts_start = this->minTimestamp();
  this->ts_end = this->maxTimestamp();
  this->ts_now = this->ts_start;
  this->time_index = 0;
  this->imu_index = 0;
  this->frame_index = 0;
}

int MAVDataset::step() {
  bool imu_event = false;
  bool cam0_event = false;
  bool cam1_event = false;

  Vec3 a_m{0.0, 0.0, 0.0};
  Vec3 w_m{0.0, 0.0, 0.0};
  std::string cam0_image_path;
  std::string cam1_image_path;

  // Loop through events at a specific timestamp
  // and get the imu or camera data
  auto it = this->timeline.lower_bound(this->ts_now);
  auto it_end = this->timeline.upper_bound(this->ts_now);
  while (it != it_end) {
    DatasetEvent event = it->second;
    if (event.type == IMU_EVENT) {
      imu_event = true;
      const Mat3 R = euler321ToRot(Vec3{0.0, deg2rad(-90.0), deg2rad(180)});
      a_m = R * event.a_m;
      w_m = R * event.w_m;
    } else if (event.camera_index == 0) {
      cam0_event = true;
      cam0_image_path = event.image_path;
    } else if (event.camera_index == 1) {
      cam1_event = true;
      cam1_image_path = event.image_path;
    }

    it++;
  }

  // Trigger imu callback
  if (imu_event && this->imu_cb != nullptr) {
    if (this->imu_cb(a_m, w_m, this->ts_now) != 0) {
      LOG_ERROR("IMU callback failed! Stopping MAVDataset!");
      return -2;
    }
    this->imu_index++;
  }

  // // Trigger camera callback
  // if (cam0_event && cam1_event) {
  //   if (this->mono_camera_cb != nullptr) {
  //     const cv::Mat frame = cv::imread(cam0_image_path);
  //     if (this->mono_camera_cb(frame, this->ts_now) != 0) {
  //       LOG_ERROR("Mono camera callback failed! Stopping MAVDataset!");
  //       return -3;
  //     }
  //   } else if (this->stereo_camera_cb != nullptr) {
  //     const cv::Mat frame0 = cv::imread(cam0_image_path);
  //     const cv::Mat frame1 = cv::imread(cam1_image_path);
  //     if (this->stereo_camera_cb(frame0, frame1, this->ts_now) != 0) {
  //       LOG_ERROR("Stereo camera callback failed! Stopping MAVDataset!");
  //       return -3;
  //     }
  //   }
  //
  //   this->frame_index++;
  // }

  if (this->record_est_cb != nullptr && this->get_state != nullptr) {
    const VecX state = this->get_state();
    this->record_est_cb(this->ts_now,
                        state.segment(0, 3),
                        state.segment(3, 3),
                        state.segment(6, 3));
  }

  // Update timestamp
  this->ts_now = this->timestamps[this->time_index++];

  return 0;
}

int MAVDataset::run() {
  // for (size_t i = 0; i < this->timestamps.size(); i++) {
  for (size_t i = 0; i < 20; i++) {
    const int retval = this->step();
    if (retval != 0) {
      return retval;
    }
  }

  return 0;
}

} // namespace gvio
