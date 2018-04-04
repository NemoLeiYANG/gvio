#include "gvio/msckf/msckf.hpp"
#include "gvio/msckf/blackbox.hpp"
#include "gvio/dataset/euroc/euroc.hpp"
#include "gvio/feature2d/klt_tracker.hpp"

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
  int retval = mav_data.load();
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

  mav_data.mono_camera_cb = mono_camera_cb;
  mav_data.run();

  // Load first image
  // cv::Mat img_ref = cv::imread(mav_data.cam0_data.image_paths[0]);
  // cv::imshow("Image", img_ref);
  // cv::waitKey();

  // // Get timestamps
  // std::vector<long> timestamps;
  // auto it = mav_data.timeline.begin();
  // auto it_end = mav_data.timeline.end();
  // while (it != it_end) {
  //   timestamps.push_back(it->first);
  //   it = mav_data.timeline.upper_bound(it->first);
  // }

  // // Iterate through timestamps
  // long time_index = 0;
  // for (auto ts : timestamps) {
  //   std::cout << "time index: " << time_index << std::endl;
  //
  //   auto it = mav_data.timeline.lower_bound(ts);
  //   auto it_end = mav_data.timeline.upper_bound(ts);
  //   while (it != it_end) {
  //     std::cout << it->second << std::endl;
  //     it++;
  //   }
  //   std::cout << std::endl;
  //   time_index++;
  //   cv::waitKey();
  // }

  // // Setup camera model
  // const int image_width = img_ref.cols;
  // const int image_height = img_ref.rows;
  // const double fx = raw_dataset.calib_cam_to_cam.K[0](0, 0);
  // const double fy = raw_dataset.calib_cam_to_cam.K[0](1, 1);
  // const double cx = raw_dataset.calib_cam_to_cam.K[0](0, 2);
  // const double cy = raw_dataset.calib_cam_to_cam.K[0](1, 2);
  // PinholeModel pinhole_model{image_width, image_height, fx, fy, cx, cy};
  //
  // // Setup feature tracker
  // KLTTracker tracker{&pinhole_model};
  // tracker.initialize(img_ref);
  //
  // // Setup MSCKF
  // MSCKF msckf;
  // if (msckf.configure(msckf_config_path) != 0) {
  //   LOG_ERROR("Failed to configure MSCKF!");
  //   return -1;
  // }
  // msckf.initialize(raw_dataset.oxts.timestamps[0],
  //                  euler2quat(raw_dataset.oxts.rpy[0]),
  //                  raw_dataset.oxts.v_G[0],
  //                  Vec3{0.0, 0.0, 0.0});
  //
  // // Record initial conditions
  // blackbox.recordTimeStep(raw_dataset.oxts.time[0],
  //                         msckf,
  //                         raw_dataset.oxts.a_B[0],
  //                         raw_dataset.oxts.w_B[0],
  //                         raw_dataset.oxts.p_G[0],
  //                         raw_dataset.oxts.v_G[0],
  //                         raw_dataset.oxts.rpy[0]);
  //
  // // Loop through data and do prediction update
  // struct timespec msckf_start = tic();
  // for (size_t i = 1; i < raw_dataset.oxts.time.size(); i++) {
  //   // Feature tracker
  //   const std::string img_path = raw_dataset.cam0[i];
  //   const cv::Mat img_cur = cv::imread(img_path);
  //   tracker.update(img_cur);
  //   FeatureTracks tracks = tracker.getLostTracks();
  //
  //   // MSCKF
  //   const Vec3 a_B = raw_dataset.oxts.a_B[i];
  //   const Vec3 w_B = raw_dataset.oxts.w_B[i];
  //   const long ts = raw_dataset.oxts.timestamps[i];
  //   msckf.predictionUpdate(a_B, w_B, ts);
  //   msckf.measurementUpdate(tracks);
  //
  //   // Record
  //   blackbox.recordTimeStep(raw_dataset.oxts.time[i],
  //                           msckf,
  //                           a_B,
  //                           w_B,
  //                           raw_dataset.oxts.p_G[i],
  //                           raw_dataset.oxts.v_G[i],
  //                           raw_dataset.oxts.rpy[i]);
  //
  //   printf("frame: %zu, nb_tracks: %ld\n", i, tracks.size());
  // }
  // printf("-- total elasped: %fs --\n", toc(&msckf_start));
  // blackbox.recordCameraStates(msckf);

  return 0;
}
