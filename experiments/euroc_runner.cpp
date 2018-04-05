#include "gvio/msckf/msckf.hpp"
#include "gvio/msckf/blackbox.hpp"
#include "gvio/dataset/euroc/euroc.hpp"
#include "gvio/feature2d/klt_tracker.hpp"
#include "gvio/quaternion/quaternion.hpp"

using namespace gvio;

void print_usage() {
  // Usage
  std::cout << "Usage: euroc_runner ";
  std::cout << "<dataset_path> ";
  std::cout << "<msckf_config_path> <output_path>" << std::endl;

  // Example
  std::cout << "Example: euroc_runner ";
  std::cout << "/data/euroc/raw/V1_01_easy";
  std::cout << "/tmp/msckf.yaml /tmp/msckf" << std::endl;
}

int mono_camera_cb(const cv::Mat &frame, const long ts) {
  cv::imshow("Image", frame);
  cv::waitKey(1);
  UNUSED(ts);

  return 0;
}

int main(const int argc, const char *argv[]) {
  // Parse cli args
  if (argc != 4) {
    print_usage();
    return -1;
  }
  const std::string dataset_path(argv[1]);
  const std::string msckf_config_path(argv[2]);
  const std::string msckf_output_path(argv[3]);

  // Load raw dataset
  MAVDataset mav_data(dataset_path);
  if (mav_data.load() != 0) {
    LOG_ERROR("Failed to load EuRoC mav dataset [%s]!", dataset_path.c_str());
    return -1;
  }

  // Setup blackbox
  BlackBox blackbox;
  if (blackbox.configure(msckf_output_path, "msckf") != 0) {
    LOG_ERROR("Failed to configure MSCKF blackbox!");
    return -1;
  }

  // Setup MSCKF
  MSCKF msckf;
  if (msckf.configure(msckf_config_path) != 0) {
    LOG_ERROR("Failed to configure MSCKF!");
    return -1;
  }

  // Initialize MSCKF
  msckf.initialize(mav_data.minTimestamp(),
                   mav_data.ground_truth.q_RS[0],
                   mav_data.ground_truth.v_RS_R[0],
                   mav_data.ground_truth.p_RS_R[0]);

  // Setup camera model and feature tracker
  // -- Setup camera model
  const int image_width = mav_data.cam0_data.resolution(0);
  const int image_height = mav_data.cam0_data.resolution(1);
  const double fx = mav_data.cam0_data.intrinsics(0);
  const double fy = mav_data.cam0_data.intrinsics(1);
  const double cx = mav_data.cam0_data.intrinsics(2);
  const double cy = mav_data.cam0_data.intrinsics(2);
  PinholeModel pinhole_model{image_width, image_height, fx, fy, cx, cy};
  // -- Setup feature tracker
  KLTTracker tracker{&pinhole_model};
  cv::Mat cam0_img0 = cv::imread(mav_data.cam0_data.image_paths[0]);
  tracker.initialize(cam0_img0);

  // Bind MSCKF to MAV dataset runner
  mav_data.imu_cb = std::bind(&MSCKF::predictionUpdate,
                              &msckf,
                              std::placeholders::_1,
                              std::placeholders::_2,
                              std::placeholders::_3);

  mav_data.mono_camera_cb = std::bind(&KLTTracker::update2,
                                      &tracker,
                                      std::placeholders::_1,
                                      std::placeholders::_2);

  mav_data.get_tracks_cb = std::bind(&KLTTracker::getLostTracks, &tracker);

  mav_data.mea_cb =
      std::bind(&MSCKF::measurementUpdate, &msckf, std::placeholders::_1);

  mav_data.get_state = std::bind(&MSCKF::getState, &msckf);
  mav_data.record_cb = std::bind(&BlackBox::recordEstimate,
                                 &blackbox,
                                 std::placeholders::_1,
                                 std::placeholders::_2,
                                 std::placeholders::_3,
                                 std::placeholders::_4);
  mav_data.run();

  return 0;
}
