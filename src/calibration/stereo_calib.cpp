#include "gvio/calibration/stereo_calib.hpp"

namespace gvio {

StereoCalib::StereoCalib() {}

StereoCalib::~StereoCalib() {}

int StereoCalib::preprocessData(const std::string &data_path) {
  // Get cam0 images
  const std::string cam0_dir = data_path + "/cam0";
  std::vector<std::string> cam0_files;
  if (list_dir(cam0_dir, cam0_files) != 0) {
    LOG_ERROR("Failed to walk [%s]!", cam0_dir.c_str());
    return -1;
  }

  // Get cam1 images
  const std::string cam1_dir = data_path + "/cam0";
  std::vector<std::string> cam1_files;
  if (list_dir(cam1_dir, cam1_files) != 0) {
    LOG_ERROR("Failed to walk [%s]!", cam1_dir.c_str());
    return -1;
  }

  // Sort files and check number of files
  std::sort(cam0_files.begin(), cam0_files.end(), std::less<std::string>());
  std::sort(cam1_files.begin(), cam1_files.end(), std::less<std::string>());
  if (cam0_files.size() != cam1_files.size()) {
    LOG_ERROR("Number of cam0 / cam1 files mismatch!");
    return -1;
  }

  // Camchain
  const std::string camchain_file = data_path + "/gvio_camchain2.yaml";
  if (this->camchain.load(2, camchain_file) != 0) {
    LOG_ERROR("Failed to load camchain file [%s]!", camchain_file.c_str());
    return -1;
  }

  // Chessborad
  this->chessboard.nb_rows = 6;
  this->chessboard.nb_cols = 7;
  this->chessboard.square_size = 0.0285;
  this->chessboard.object_points = this->chessboard.createObjectPoints();

  // Detect chessboard corners
  for (size_t i = 0; i < cam0_files.size(); i++) {
    // Load images
    const std::string img0_file = data_path + "/cam0/" + cam0_files[i];
    const std::string img1_file = data_path + "/cam1/" + cam1_files[i];
    const cv::Mat img0 = cv::imread(img0_file);
    const cv::Mat img1 = cv::imread(img1_file);
    this->image_size = cv::Size(img0.cols, img0.rows);
    std::cout << "Loading file [" << img0_file.c_str() << "]" << std::endl;
    std::cout << "Loading file [" << img1_file.c_str() << "]" << std::endl;

    // Detect chessboard corners from cam0 and cam1
    int retval = 0;
    std::vector<cv::Point2f> corners0, corners1;
    retval = this->chessboard.detect(img1, corners1);
    retval = this->chessboard.detect(img0, corners0);
    if (retval != 0) {
      LOG_ERROR("Failed to detect chessboard in both images [%d]!", (int) i);
      continue;
    }

    // Update book keeping
    this->object_points.push_back(this->chessboard.object_points);
    this->imgpts0.push_back(corners0);
    this->imgpts1.push_back(corners1);
  }

  // cv::Mat image_rgb = image.clone();
  // this->chessboard.drawCorners(image_rgb);
  // cv::imshow("Image", image_rgb);
  // cv::waitKey();

  return 0;
}

// int StereoCalib::calibrateIntrinsics() {
//   cv::Mat K0;
//   cv::Mat D0;
//   std::vector<cv::Mat> rvecs;
//   std::vector<cv::Mat> tvecs;
//
//   cv::calibrateCamera(this->object_points,
//                       this->imgpts0,
//                       this->image_size,
//                       K0,
//                       D0,
//                       rvecs,
//                       tvecs);
//
//   std::cout << K0 << std::endl;
//   std::cout << D0 << std::endl;
//
//   return 0;
// }

int StereoCalib::calibrateExtrinsics() {
  std::cout << "Calibrating stereo extrinsics!" << std::endl;
  std::cout << "This can take a while ..." << std::endl;

  cv::Mat K0 = convert(this->camchain.cam[0].K());
  cv::Mat K1 = convert(this->camchain.cam[1].K());
  cv::Mat D0 = convert(this->camchain.cam[0].D());
  cv::Mat D1 = convert(this->camchain.cam[1].D());
  cv::Mat R, T, E, F;
  cv::TermCriteria criteria =
      cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 1000, 1e-5);
  int flags = 0;
  // int flags = CV_CALIB_FIX_ASPECT_RATIO;
  // flags += CV_CALIB_ZERO_TANGENT_DIST;
  // flags += CV_CALIB_SAME_FOCAL_LENGTH;
  // flags += CV_CALIB_RATIONAL_MODEL;
  // flags += CV_CALIB_FIX_K3;
  // flags += CV_CALIB_FIX_K4;
  // flags += CV_CALIB_FIX_K5;

  double rms = cv::stereoCalibrate(object_points,    // Object points
                                   imgpts0,          // Cam0 image points
                                   imgpts1,          // Cam1 image points
                                   K0,               // Cam0 intrinsic matrix K
                                   D0,               // Cam0 distortion coeffs D
                                   K1,               // Cam1 intrinsic matrix K
                                   D1,               // Cam1 distortion coeffs D
                                   this->image_size, // Image size
                                   R,                // Rotation
                                   T,                // Translation
                                   E,                // Essential matrix
                                   F,                // Foundamental matrix
                                   flags,            // Flags
                                   criteria);        // Criteria
  std::cout << "done with RMS error=" << rms << std::endl;

  std::cout << "K0: " << K0 << std::endl;
  std::cout << "D0: " << D0 << std::endl;
  std::cout << "K1: " << K1 << std::endl;
  std::cout << "D1: " << D1 << std::endl;
  std::cout << "R: " << R << std::endl;
  std::cout << "T: " << T << std::endl;

  return 0;
}

} // namespace gvio
