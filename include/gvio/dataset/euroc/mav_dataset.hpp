/**
 * @file
 * @defgroup euroc euroc
 * @ingroup dataset
 */
#ifndef GVIO_DATASET_EUROC_MAV_DATASET_HPP
#define GVIO_DATASET_EUROC_MAV_DATASET_HPP

#include <iostream>
#include <functional>
#include <map>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include "gvio/util/util.hpp"
#include "gvio/msckf/msckf.hpp"
#include "gvio/dataset/euroc/imu_data.hpp"
#include "gvio/dataset/euroc/camera_data.hpp"
#include "gvio/dataset/euroc/ground_truth.hpp"

namespace gvio {
/**
 * @addtogroup euroc
 * @{
 */

#define BIND_IMU_CALLBACK(CLASS_METHOD, INSTANCE)                              \
  std::bind(&CLASS_METHOD,                                                     \
            &INSTANCE,                                                         \
            std::placeholders::_1,                                             \
            std::placeholders::_2,                                             \
            std::placeholders::_3);

#define BIND_MONO_CAMERA_CALLBACK(CLASS_METHOD, INSTANCE)                      \
  std::bind(&CLASS_METHOD, &INSTANCE, std::placeholders::_1);

#define BIND_STEREO_CAMERA_CALLBACK(CLASS_METHOD, INSTANCE)                    \
  std::bind(&CLASS_METHOD,                                                     \
            &INSTANCE,                                                         \
            std::placeholders::_1,                                             \
            std::placeholders::_2)

#define BIND_GET_TRACKS_CALLBACK(CLASS_METHOD, INSTANCE)                       \
  std::bind(&CLASS_METHOD, &INSTANCE);

#define BIND_MEASUREMENT_CALLBACK(CLASS_METHOD, INSTANCE)                      \
  std::bind(&CLASS_METHOD, &INSTANCE, std::placeholders::_1);

#define BIND_GET_STATE_CALLBACK(CLASS_METHOD, INSTANCE)                        \
  std::bind(&CLASS_METHOD, &INSTANCE);

#define BIND_RECORD_CALLBACK(CLASS_METHOD, INSTANCE)                           \
  std::bind(&CLASS_METHOD,                                                     \
            &INSTANCE,                                                         \
            std::placeholders::_1,                                             \
            std::placeholders::_2,                                             \
            std::placeholders::_3,                                             \
            std::placeholders::_4);

/**
 * Dataset Event Type
 */
enum DatasetEventType { NOT_SET, IMU_EVENT, CAMERA_EVENT };

/**
 * Dataset Event
 */
struct DatasetEvent {
  // Event Type
  DatasetEventType type;

  // IMU data
  Vec3 a_m;
  Vec3 w_m;

  // Camera data
  int camera_index;
  std::string image_path;

  DatasetEvent(const Vec3 &a_m, const Vec3 &w_m)
      : type{IMU_EVENT}, a_m{a_m}, w_m{w_m} {}
  DatasetEvent(const int camera_index, const std::string &image_path)
      : type{CAMERA_EVENT}, camera_index{camera_index}, image_path{image_path} {
  }
};

/**
 * DatasetEvent to output stream
 */
std::ostream &operator<<(std::ostream &os, const DatasetEvent &data);

/**
 * EuRoC MAV Dataset
 */
class MAVDataset {
public:
  bool ok = false;
  std::string data_path;

  IMUData imu_data;
  CameraData cam0_data;
  CameraData cam1_data;
  GroundTruth ground_truth;

  long ts_start = 0;
  long ts_end = 0;
  long ts_now = 0;
  long time_index = 0;
  long imu_index = 0;
  long frame_index = 0;

  std::vector<long> timestamps;
  std::map<long, double> time;
  std::multimap<long, DatasetEvent> timeline;
  FeatureTracks feature_tracks;

  // clang-format off
  std::function<VecX()> get_state;
  std::function<int(const cv::Mat &frame)> mono_camera_cb;
  std::function<int(const cv::Mat &frame0, const cv::Mat &frame1)> stereo_camera_cb;
  std::function<std::vector<FeatureTrack>()> get_tracks_cb;
  std::function<int(const Vec3 &a_m, const Vec3 &w_m, const long ts)> imu_cb;
  std::function<int(const FeatureTracks &tracks)> mea_cb;
  std::function<int(const double time, const Vec3 &p_G, const Vec3 &v_G, const Vec3 &rpy_G)> record_cb;
  // clang-format on

  MAVDataset(const std::string &data_path);

  /**
   * Load imu data
   * @returns 0 for success, -1 for failure
   */
  int loadIMUData();

  /**
   * Load camera data
   * @returns 0 for success, -1 for failure
   */
  int loadCameraData();

  /**
   * Load ground truth
   * @returns 0 for success, -1 for failure
   */
  int loadGroundTruth();

  /**
   * Return min timestamp
   */
  long minTimestamp();

  /**
   * Return max timestamp
   */
  long maxTimestamp();

  /**
   * Load data
   * @returns 0 for success, -1 for failure
   */
  int load();

  /**
   * Reset
   */
  void reset();

  /**
   * Step
   *
   * @returns
   * - 0 for success
   * - -2 for imu callback failure
   * - -3 for camera callback failure
   */
  int step();

  /**
   * Run
   *
   * @returns
   * - 0 for success
   * - -2 for imu callback failure
   * - -3 for camera callback failure
   */
  int run();
};

/** @} group euroc */
} // namespace gvio
#endif // GVIO_DATASET_EUROC_MAV_DATASET_HPP
