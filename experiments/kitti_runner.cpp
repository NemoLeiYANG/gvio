#include "gvio/dataset/kitti/kitti.hpp"
#include "gvio/msckf/msckf.hpp"
#include "gvio/msckf/blackbox.hpp"
#include "gvio/feature2d/klt_tracker.hpp"

using namespace gvio;

void print_usage() {
  // Usage
  std::cout << "Usage: kitti_runner ";
  std::cout << "<dataset_path> <date> <seq> ";
  std::cout << "<msckf_config_path> <output_path>" << std::endl;

  // Example
  std::cout << "Example: kitti_runner ";
  std::cout << "/data/kitti/raw 2011_09_26 0005 ";
  std::cout << "/tmp/msckf.yaml /tmp/msckf" << std::endl;
}

void get_imu_cam_extrinsics(const RawDataset &dataset, Vec4 &q_CI, Vec3 &p_IC) {
  const Mat4 T_cam_velo = dataset.calib_velo_to_cam.T_cam_velo;
  const Mat4 T_velo_imu = dataset.calib_imu_to_velo.T_velo_imu;
  const Mat4 T_cam_imu = T_cam_velo * T_velo_imu;
  const Mat4 T_imu_cam = T_cam_imu.inverse();

  q_CI = rot2quat(T_cam_imu.block(0, 0, 3, 3));
  p_IC = T_imu_cam.block(0, 3, 3, 1);

  std::cout << "Camera-IMU Extrinsics:" << std::endl;
  std::cout << "q_CI: " << q_CI.transpose() << std::endl;
  std::cout << "p_IC: " << p_IC.transpose() << std::endl;
}

int main(const int argc, const char *argv[]) {
  // Seed random
  std::srand(1);

  // Parse cli args
  if (argc != 6) {
    print_usage();
    return -1;
  }
  const std::string dataset_path(argv[1]);
  const std::string dataset_date(argv[2]);
  const std::string dataset_seq(argv[3]);
  const std::string msckf_config_path(argv[4]);
  const std::string msckf_output_path(argv[5]);

  // Load raw dataset
  RawDataset raw_dataset(dataset_path, dataset_date, dataset_seq);
  if (raw_dataset.load() != 0) {
    LOG_ERROR("Failed to load KITTI raw dataset [%s]!", dataset_path.c_str());
    return -1;
  }
  Vec4 q_CI;
  Vec3 p_IC;
  get_imu_cam_extrinsics(raw_dataset, q_CI, p_IC);

  // Setup blackbox
  BlackBox blackbox;
  if (blackbox.configure(msckf_output_path, "msckf") != 0) {
    LOG_ERROR("Failed to configure MSCKF blackbox!");
    return -1;
  }

  // Load first image
  cv::Mat img_ref = cv::imread(raw_dataset.cam0[0]);

  // Setup camera model
  const int image_width = img_ref.cols;
  const int image_height = img_ref.rows;
  const double fx = raw_dataset.calib_cam_to_cam.K[0](0, 0);
  const double fy = raw_dataset.calib_cam_to_cam.K[0](1, 1);
  const double cx = raw_dataset.calib_cam_to_cam.K[0](0, 2);
  const double cy = raw_dataset.calib_cam_to_cam.K[0](1, 2);
  PinholeModel pinhole_model{image_width, image_height, fx, fy, cx, cy};

  // Setup feature tracker
  KLTTracker tracker{&pinhole_model};
  tracker.initialize(img_ref);

  // Setup MSCKF
  MSCKF msckf;
  if (msckf.configure(msckf_config_path) != 0) {
    LOG_ERROR("Failed to configure MSCKF!");
    return -1;
  }
  msckf.initialize(raw_dataset.oxts.timestamps[0],
                   euler2quat(raw_dataset.oxts.rpy[0]),
                   raw_dataset.oxts.v_G[0],
                   Vec3{0.0, 0.0, 0.0});

  // Record initial conditions
  blackbox.recordTimeStep(raw_dataset.oxts.time[0],
                          msckf,
                          raw_dataset.oxts.a_B[0],
                          raw_dataset.oxts.w_B[0],
                          raw_dataset.oxts.p_G[0],
                          raw_dataset.oxts.v_G[0],
                          raw_dataset.oxts.rpy[0]);

  // Loop through data and do prediction update
  struct timespec msckf_start = tic();
  for (size_t i = 1; i < raw_dataset.oxts.time.size(); i++) {
    // Feature tracker
    const std::string img_path = raw_dataset.cam0[i];
    const cv::Mat img_cur = cv::imread(img_path);
    tracker.update(img_cur);
    FeatureTracks tracks = tracker.getLostTracks();

    // MSCKF
    const Vec3 a_B = raw_dataset.oxts.a_B[i];
    const Vec3 w_B = raw_dataset.oxts.w_B[i];
    const long ts = raw_dataset.oxts.timestamps[i];
    msckf.predictionUpdate(a_B, w_B, ts);
    msckf.measurementUpdate(tracks);

    // Record
    blackbox.recordTimeStep(raw_dataset.oxts.time[i],
                            msckf,
                            a_B,
                            w_B,
                            raw_dataset.oxts.p_G[i],
                            raw_dataset.oxts.v_G[i],
                            raw_dataset.oxts.rpy[i]);

    printf("frame: %zu, nb_tracks: %ld\n", i, tracks.size());
  }
  printf("-- total elasped: %fs --\n", toc(&msckf_start));
  blackbox.recordCameraStates(msckf);

  return 0;
}
