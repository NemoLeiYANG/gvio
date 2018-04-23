#include <iostream>

#include "gvio/dataset/euroc/euroc.hpp"
#include "gvio/feature2d/stereo_klt_tracker.hpp"
#include "gvio/feature2d/stereo_orb_tracker.hpp"
#include "gvio/feature2d/orb_tracker.hpp"

using namespace gvio;

void print_usage() {
  // Usage
  std::cout << "Usage: feature_tracker_benchmarker ";
  std::cout << "<dataset_path> ";

  // Example
  std::cout << "Example: feature_tracker_benchmarker ";
  std::cout << "/data/euroc/raw/V1_01_easy";
}

int main(const int argc, const char *argv[]) {
  // Parse cli args
  if (argc != 2) {
    print_usage();
    return -1;
  }
  const std::string dataset_path(argv[1]);

  // Load dataset
  MAVDataset mav_data(dataset_path);
  if (mav_data.load() != 0) {
    LOG_ERROR("Failed to load EuRoC mav dataset [%s]!", dataset_path.c_str());
    return -1;
  }

  // Setup camera model and feature tracker
  const int image_width = mav_data.cam0_data.resolution(0);
  const int image_height = mav_data.cam0_data.resolution(1);
  // -- Setup cam0 property
  const double cam0_fx = mav_data.cam0_data.intrinsics(0);
  const double cam0_fy = mav_data.cam0_data.intrinsics(1);
  const double cam0_cx = mav_data.cam0_data.intrinsics(2);
  const double cam0_cy = mav_data.cam0_data.intrinsics(3);
  CameraProperty camprop0{0,
                          cam0_fx,
                          cam0_fy,
                          cam0_cx,
                          cam0_cy,
                          image_width,
                          image_height};
  // -- Setup cam1 property
  const double cam1_fx = mav_data.cam1_data.intrinsics(0);
  const double cam1_fy = mav_data.cam1_data.intrinsics(1);
  const double cam1_cx = mav_data.cam1_data.intrinsics(2);
  const double cam1_cy = mav_data.cam1_data.intrinsics(3);
  CameraProperty camprop1{1,
                          cam1_fx,
                          cam1_fy,
                          cam1_cx,
                          cam1_cy,
                          image_width,
                          image_height};
  // -- Setup feature tracker
  // StereoORBTracker tracker(camprop0, camprop1, 2, 100);

  // const Mat4 T_imu_cam0 = mav_data.cam0_data.T_BS;
  // const Mat4 T_imu_cam1 = mav_data.cam1_data.T_BS;
  // const Mat4 T_cam1_cam0 = T_imu_cam1.inverse() * T_imu_cam0;
  // StereoKLTTracker tracker(camprop0, camprop1, T_cam1_cam0, 1, 100);

  // Stereo ORB tracker
  // clang-format off
  StereoORBTracker tracker(camprop0, camprop1, 2, 100);
  tracker.show_matches = true;
  mav_data.stereo_camera_cb = BIND_STEREO_CAMERA_CALLBACK(StereoORBTracker::update, tracker);
  mav_data.get_tracks_cb = BIND_GET_TRACKS_CALLBACK(StereoORBTracker::getLostTracks, tracker);
  mav_data.run();
  // clang-format on

  // // Mono ORB tracker
  // // clang-format off
  // ORBTracker tracker(camprop0, 2, 100);
  // tracker.show_matches = true;
  // mav_data.mono_camera_cb = BIND_MONO_CAMERA_CALLBACK(ORBTracker::update,
  // tracker);
  // mav_data.get_tracks_cb =
  // BIND_GET_TRACKS_CALLBACK(ORBTracker::getLostTracks, tracker);
  // mav_data.run();
  // // clang-format on

  const long nb_images = mav_data.frame_index;
  const long nb_imu_measurements = mav_data.imu_index;
  LOG_INFO("EuroC MAV dataset summary");
  LOG_INFO("-----------------------------");
  LOG_INFO("Processed %ld images", nb_images);
  LOG_INFO("Processed %ld measurements", nb_imu_measurements);

  return 0;
}
