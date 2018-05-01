#include "gvio/msckf/msckf.hpp"
#include "gvio/msckf/blackbox.hpp"
#include "gvio/dataset/kitti/kitti.hpp"
#include "gvio/feature2d/klt_tracker.hpp"
#include "gvio/feature2d/stereo_klt_tracker.hpp"
#include "gvio/quaternion/quaternion.hpp"

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

  // Setup blackbox
  BlackBox blackbox;
  if (blackbox.configure(msckf_output_path, "msckf") != 0) {
    LOG_ERROR("Failed to configure MSCKF blackbox!");
    return -1;
  }

  // Setup mono front-end
  // -- Load first image to obtain image size
  const cv::Mat img = cv::imread(raw_dataset.cam0[0]);
  // -- Setup camera model
  const double fx = raw_dataset.calib_cam_to_cam.K[0](0, 0);
  const double fy = raw_dataset.calib_cam_to_cam.K[0](1, 1);
  const double cx = raw_dataset.calib_cam_to_cam.K[0](0, 2);
  const double cy = raw_dataset.calib_cam_to_cam.K[0](1, 2);
  const int image_width = img.cols;
  const int image_height = img.rows;
  CameraProperty camera_property{0, fx, fy, cx, cy, image_width, image_height};
  // -- Setup feature tracker
  KLTTracker mono_tracker{camera_property, 10, 20};
  mono_tracker.initialize(cv::imread(raw_dataset.cam0[0]));

  // Setup stereo front-end
  // -- Load first image from cam0 and cam1
  const cv::Mat cam0_img = cv::imread(raw_dataset.cam0[0]);
  const cv::Mat cam1_img = cv::imread(raw_dataset.cam1[0]);
  // -- Obtain image size
  const Vec2 image_size{img.size().width, img.size().height};
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
  StereoKLTTracker stereo_tracker{camprop0, camprop1, T_C1_C0, 10, 20};
  stereo_tracker.initialize(cam0_img, cam1_img);

  // Setup MSCKF
  MSCKF msckf;
  if (msckf.configure(msckf_config_path) != 0) {
    LOG_ERROR("Failed to configure MSCKF!");
    return -1;
  }
  msckf.T_C1_C0 = T_C1_C0;
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
  for (int i = 1; i < (int) raw_dataset.oxts.time.size(); i++) {
    // Feature tracker
    const std::string cam0_img_path = raw_dataset.cam0[i];
    const std::string cam1_img_path = raw_dataset.cam1[i];
    const cv::Mat cam0_img = cv::imread(cam0_img_path);
    const cv::Mat cam1_img = cv::imread(cam1_img_path);

    // // Mono front-end
    // mono_tracker.update(cam0_img);
    // FeatureTracks tracks = mono_tracker.getLostTracks();

    // Stereo front-end
    stereo_tracker.update(cam0_img, cam1_img);
    FeatureTracks tracks = stereo_tracker.getLostTracks();

    // Triangulate tracks
    auto T_cam1_cam0 = stereo_tracker.T_cam1_cam0;
    triangulate_stereo_tracks(T_cam1_cam0, tracks);

    // MSCKF
    const Vec3 a_B = raw_dataset.oxts.a_B[i];
    const Vec3 w_B = raw_dataset.oxts.w_B[i];
    const size_t ts = raw_dataset.oxts.timestamps[i];
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

    // cv::imshow("Image", cam0_img);
    // cv::waitKey(1);

    LOG_INFO("frame: %d, nb_tracks: %ld\n", i, tracks.size());
  }
  LOG_INFO("-- total elasped: %fs --\n", toc(&msckf_start));
  blackbox.recordCameraStates(msckf);

  return 0;
}
