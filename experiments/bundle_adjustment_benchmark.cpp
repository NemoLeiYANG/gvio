#include <iostream>

#include "gvio/dataset/kitti/kitti.hpp"
#include "gvio/msckf/feature_estimator.hpp"
#include "gvio/feature2d/stereo_klt_tracker.hpp"

using namespace gvio;

void print_usage() {
  // Usage
  std::cout << "Usage: bundle_adjustment_benchmark ";
  std::cout << "<dataset_path> <date> <seq> ";

  // Example
  std::cout << "Example: bundle_adjustment_benchmark ";
  std::cout << "/data/kitti/raw 2011_09_26 0005 ";
}

int main(const int argc, const char *argv[]) {
  // Parse cli args
  if (argc != 4) {
    print_usage();
    return -1;
  }
  const std::string dataset_path(argv[1]);
  const std::string dataset_date(argv[2]);
  const std::string dataset_seq(argv[3]);

  // Load raw dataset
  RawDataset raw_dataset(dataset_path, dataset_date, dataset_seq);
  if (raw_dataset.load() != 0) {
    LOG_ERROR("Failed to load KITTI raw dataset [%s]!", dataset_path.c_str());
    return -1;
  }

  // Setup front-end
  cv::Mat cam0_img = cv::imread(raw_dataset.cam0[0]);
  const Vec2 image_size{cam0_img.size().width, cam0_img.size().height};
  // -- Setup cam0 property
  const Mat3 cam0_K = raw_dataset.calib_cam_to_cam.K[0];
  const VecX cam0_D = Vec4::Zero();
  CameraProperty camprop0{0, "pinhole", cam0_K, "radtan", cam0_D, image_size};
  // -- Setup cam1 property
  const Mat3 cam1_K = raw_dataset.calib_cam_to_cam.K[1];
  const VecX cam1_D = Vec4::Zero();
  CameraProperty camprop1{1, "pinhole", cam1_K, "radtan", cam1_D, image_size};
  // -- cam0 to cam1 extrinsics
  const Mat3 R_C1C0 = raw_dataset.calib_cam_to_cam.R[1];
  const Vec3 t_C1C0 = raw_dataset.calib_cam_to_cam.T[1];
  const Mat4 T_C1_C0 = transformation_matrix(R_C1C0, t_C1C0);
  // -- Setup feature tracker
  StereoKLTTracker tracker{camprop0, camprop1, T_C1_C0, 10, 20};
  tracker.show_matches = false;

  CameraStates camera_states;
  std::vector<Vec3> landmarks;
  const Vec3 ext_p_IC{0.0, 0.0, 0.0};
  const Vec4 ext_q_CI{0.50243, -0.491157, 0.504585, -0.50172};

  for (size_t i = 0; i < raw_dataset.cam0.size(); i++) {
    // Feature tracker
    const std::string cam0_img_path = raw_dataset.cam0[i];
    const std::string cam1_img_path = raw_dataset.cam1[i];
    const cv::Mat cam0_img = cv::imread(cam0_img_path);
    const cv::Mat cam1_img = cv::imread(cam1_img_path);
    tracker.update(cam0_img, cam1_img);
    FeatureTracks tracks = tracker.getLostTracks();

    // Add camera state
    const Vec3 imu_p_G = raw_dataset.oxts.p_G[i];
    const Vec4 imu_q_IG = euler2quat(raw_dataset.oxts.rpy[i]);
    const Vec4 cam_q_CG = quatlcomp(ext_q_CI) * imu_q_IG;
    const Vec3 cam_p_G = imu_p_G + C(imu_q_IG).transpose() * ext_p_IC;
    camera_states.emplace_back(i, cam_p_G, cam_q_CG);

    // Estimate features
    int landmarks_added = 0;
    for (auto track : tracks) {
      auto track_cam_states = get_track_camera_states(camera_states, track);
      auto T_cam1_cam0 = tracker.T_cam1_cam0;
      assert(track_cam_states.size() == track.trackedLength());
      CeresFeatureEstimator estimator{track, track_cam_states, T_cam1_cam0};

      Vec3 p_G_f;
      if (estimator.estimate(p_G_f) == 0) {
        landmarks.emplace_back(p_G_f);
        landmarks_added++;
      }
    }
    LOG_INFO("frame: %d, landmarks added: [%d / %d]",
             (int) i,
             landmarks_added,
             (int) tracks.size());

    // Break loop if 'q' was pressed
    if (tracker.show_matches && cv::waitKey(0) == 113) {
      break;
    }
  }
  LOG_INFO("Dataset done!");

  // Output landmarks to file
  LOG_INFO("Outputting landmarks to file!");
  std::ofstream landmarks_file("/tmp/landmarks.csv");
  for (auto landmark : landmarks) {
    landmarks_file << landmark(0) << ",";
    landmarks_file << landmark(1) << ",";
    landmarks_file << landmark(2) << std::endl;
  }
  landmarks_file.close();

  // Output ground truth to file
  LOG_INFO("Outputting ground truth to file!");
  std::ofstream pose_file("/tmp/pose.csv");
  pose_file << "x,y,z,roll,pitch,yaw" << std::endl;
  for (auto camera_state : camera_states) {
    // Position
    pose_file << camera_state.p_G(0) << ",";
    pose_file << camera_state.p_G(1) << ",";
    pose_file << camera_state.p_G(2) << ",";

    // Roll, pitch and yaw
    const Vec3 rpy_G = quat2euler(camera_state.q_CG);
    pose_file << rpy_G(0) << ",";
    pose_file << rpy_G(1) << ",";
    pose_file << rpy_G(2) << std::endl;
  }
  pose_file.close();

  return 0;
}
