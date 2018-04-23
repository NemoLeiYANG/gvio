#include "gvio/dataset/euroc/mav_dataset.hpp"

namespace gvio {

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

MAVDataset::MAVDataset(const std::string &data_path)
    : data_path{strip_end(data_path, "/")} {}

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

int MAVDataset::loadGroundTruth() {
  const std::string gnd_dir = this->data_path + "/state_groundtruth_estimate0";
  if (this->ground_truth.load(gnd_dir) != 0) {
    LOG_ERROR("Failed to load ground truth!");
    return -1;
  }

  return 0;
}

long MAVDataset::minTimestamp() {
  const long cam0_first_ts = this->cam0_data.timestamps.front();
  const long imu_first_ts = this->imu_data.timestamps.front();

  std::vector<long> first_ts{cam0_first_ts, imu_first_ts};
  auto first_result = std::min_element(first_ts.begin(), first_ts.end());
  const long first_ts_index = std::distance(first_ts.begin(), first_result);
  const long min_ts = first_ts[first_ts_index];

  return min_ts;
}

long MAVDataset::maxTimestamp() {
  const long cam0_last_ts = this->cam0_data.timestamps.back();
  const long imu_last_ts = this->imu_data.timestamps.back();

  std::vector<long> last_ts{cam0_last_ts, imu_last_ts};
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
  if (this->loadGroundTruth() != 0) {
    LOG_ERROR("Failed to load ground truth data!");
    return -1;
  }

  // Timestamp
  this->ts_start = this->minTimestamp();
  this->ts_end = this->maxTimestamp();
  this->ts_now = this->ts_start;

  // Get timestamps and calculate relative time
  auto it = this->timeline.begin();
  auto it_end = this->timeline.end();
  while (it != it_end) {
    const long ts = it->first;
    this->timestamps.push_back(ts);
    this->time[ts] = ((double) ts - this->ts_start) * 1e-9;

    // Advance to next non-duplicate entry.
    do {
      ++it;
    } while (ts == it->first);
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
      a_m = event.a_m;
      w_m = event.w_m;
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
    // clang-format off
    Mat3 R_body_imu;
    R_body_imu << 0.0, 0.0, 1.0,
                  0.0, -1.0, 0.0,
                  1.0, 0.0, 0.0;
    // clang-format on
    a_m = R_body_imu * a_m;
    w_m = R_body_imu * w_m;
    // std::cout << a_m.transpose() << std::endl;

    if (this->imu_cb(a_m, w_m, this->ts_now) != 0) {
      LOG_ERROR("IMU callback failed! Stopping MAVDataset!");
      return -2;
    }
    this->imu_index++;
  }

  // Trigger camera and measurement callback
  if (cam0_event && cam1_event) {
    // Camera callback
    if (this->mono_camera_cb != nullptr) {
      const cv::Mat frame = cv::imread(cam0_image_path);
      if (this->mono_camera_cb(frame) != 0) {
        LOG_ERROR("Mono camera callback failed! Stopping MAVDataset!");
        return -3;
      }
    } else if (this->stereo_camera_cb != nullptr) {
      const cv::Mat frame0 = cv::imread(cam0_image_path);
      const cv::Mat frame1 = cv::imread(cam1_image_path);
      if (this->stereo_camera_cb(frame0, frame1) != 0) {
        LOG_ERROR("Stereo camera callback failed! Stopping MAVDataset!");
        return -3;
      }
    }

    // Measurement callback
    if (this->get_tracks_cb != nullptr) {
      FeatureTracks tracks = this->get_tracks_cb();
      save_feature_tracks(tracks, "./stereo_orb_tracker");

      if (this->mea_cb != nullptr) {
        if (this->mea_cb(tracks) != 0) {
          LOG_ERROR("Measurement callback failed! Stopping MAVDataset!");
          return -2;
        }
      }
    }

    this->frame_index++;
  }

  // Trigger record estimate callback
  if (this->record_cb != nullptr && this->get_state != nullptr) {
    const VecX state = this->get_state();
    this->record_cb(this->time[this->ts_now],
                    state.segment(0, 3),
                    state.segment(3, 3),
                    state.segment(6, 3));
  }

  // Update timestamp
  this->time_index++;
  this->ts_now = this->timestamps[this->time_index];

  return 0;
}

int MAVDataset::run() {
  for (size_t i = 0; i < this->timestamps.size(); i++) {
    // for (size_t i = 0; i < 600; i++) {
    const int retval = this->step();
    if (retval != 0) {
      return retval;
    }
  }
  return 0;
}

} // namespace gvio
