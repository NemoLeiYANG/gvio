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

  // Setup camera property
  const Vec2 image_size = mav_data.cam0_data.resolution;
  // -- Setup cam0 property
  const Mat3 cam0_K = pinhole_K(mav_data.cam0_data.intrinsics);
  const VecX cam0_D = mav_data.cam0_data.distortion_coefficients;
  CameraProperty camprop0{0, "pinhole", cam0_K, "radtan", cam0_D, image_size};
  // -- Setup cam1 property
  const Mat3 cam1_K = pinhole_K(mav_data.cam0_data.intrinsics);
  const VecX cam1_D = mav_data.cam0_data.distortion_coefficients;
  CameraProperty camprop1{1, "pinhole", cam1_K, "radtan", cam1_D, image_size};

  // // Mono ORB tracker
  // // clang-format off
  // ORBTracker orb_tracker(camprop0, 2, 100);
  // orb_tracker.show_matches = true;
  // mav_data.mono_camera_cb = BIND_MONO_CAMERA_CALLBACK(ORBTracker::update,
  // orb_tracker);
  // mav_data.get_tracks_cb =
  // BIND_GET_TRACKS_CALLBACK(ORBTracker::getLostTracks, orb_tracker);
  // mav_data.run();
  // save_feature_tracks(mav_data.feature_tracks, "/tmp/orb_feature_tracks");
  // // clang-format on

  // Mono ORB tracker
  // clang-format off
  KLTTracker klt_tracker(camprop0, 2, 100);
  klt_tracker.show_matches = true;
  mav_data.mono_camera_cb = BIND_MONO_CAMERA_CALLBACK(KLTTracker::update, klt_tracker);
  mav_data.get_tracks_cb = BIND_GET_TRACKS_CALLBACK(KLTTracker::getLostTracks, klt_tracker);
  mav_data.run();
  save_feature_tracks(mav_data.feature_tracks, "/tmp/klt_feature_tracks");
  // clang-format on

  // // Stereo KLT tracker
  // // clang-format off
  // const Mat4 T_imu_cam0 = mav_data.cam0_data.T_BS;
  // const Mat4 T_imu_cam1 = mav_data.cam1_data.T_BS;
  // const Mat4 T_cam1_cam0 = T_imu_cam1.inverse() * T_imu_cam0;
  // StereoKLTTracker stereo_klt_tracker(camprop0, camprop1, T_cam1_cam0, 2,
  // 100);
  // mav_data.stereo_camera_cb =
  // BIND_STEREO_CAMERA_CALLBACK(StereoKLTTracker::update,
  //                                                         stereo_klt_tracker);
  // mav_data.get_tracks_cb =
  // BIND_GET_TRACKS_CALLBACK(StereoKLTTracker::getLostTracks,
  //                                                   stereo_klt_tracker);
  // // save_feature_tracks(mav_data.feature_tracks, "/tmp/feature_tracks");
  // // mav_data.run();
  // // clang-format on

  // Summary
  const long nb_images = mav_data.frame_index;
  const long nb_imu_measurements = mav_data.imu_index;
  LOG_INFO("EuroC MAV dataset summary");
  LOG_INFO("-----------------------------");
  LOG_INFO("Processed %ld images", nb_images);
  LOG_INFO("Processed %ld measurements", nb_imu_measurements);

  return 0;
}
