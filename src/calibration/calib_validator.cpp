#include "gvio/calibration/calib_validator.hpp"

namespace gvio {

CalibValidator::CalibValidator() {}

CalibValidator::~CalibValidator() {}

int CalibValidator::load(const int nb_cameras,
                         const std::string &camchain_file,
                         const std::string &target_file) {
  assert(nb_cameras > 0);
  assert(camchain_file.empty() == false);
  assert(target_file.empty() == false);

  // Parse camchain file
  if (this->camchain.load(nb_cameras, camchain_file) != 0) {
    LOG_ERROR("Failed to load camchain file [%s]!", camchain_file.c_str());
    return -1;
  }

  // Chessboard
  if (this->chessboard.load(target_file) != 0) {
    LOG_ERROR("Failed to load chessboard config!");
    return -1;
  }

  // Gimbal model
  this->gimbal_model.tau_s = this->camchain.tau_s;
  this->gimbal_model.tau_d = this->camchain.tau_d;
  this->gimbal_model.w1 = this->camchain.w1;
  this->gimbal_model.w2 = this->camchain.w2;
  this->gimbal_model.theta1_offset = this->camchain.theta1_offset;
  this->gimbal_model.theta2_offset = this->camchain.theta2_offset;

  return 0;
}

int CalibValidator::detect(const int camera_index,
                           const cv::Mat &image,
                           MatX &X) {
  // Find chessboard corners
  std::vector<cv::Point2f> corners;
  if (this->chessboard.detect(image, corners) != 0) {
    return 0;
  }

  // Undistort points
  auto camera = this->camchain.cam[camera_index];
  const std::vector<cv::Point2f> corners_ud = camera.undistortPoints(corners);

  // Calculate corner positions in 3D
  this->chessboard.calcCornerPositions(corners_ud, I(3), X);

  return 1;
}

cv::Mat CalibValidator::drawDetected(const cv::Mat &image,
                                     const cv::Scalar &color) {
  // Find chessboard corners
  std::vector<cv::Point2f> corners;
  if (this->chessboard.detect(image, corners) != 0) {
    return image;
  }

  // Make an RGB version of the input image
  cv::Mat image_rgb;
  if (image.channels() == 1) {
    image_rgb = cv::Mat(image.size(), CV_8UC3);
    image_rgb = image.clone();
    cv::cvtColor(image, image_rgb, CV_GRAY2RGB);
  } else {
    image_rgb = image.clone();
  }

  // Draw detected points
  for (size_t i = 0; i < corners.size(); i++) {
    cv::circle(image_rgb,  // Target image
               corners[i], // Center
               1,          // Radius
               color,      // Colour
               CV_FILLED,  // Thickness
               8);         // Line type
  }

  return image_rgb;
}

double CalibValidator::reprojectionError(
    const cv::Mat &image, const std::vector<cv::Point2f> &image_points) {
  // Find chessboard corners
  std::vector<cv::Point2f> corners;
  if (this->chessboard.detect(image, corners) != 0) {
    return -1;
  }

  // Calculate RMSE reprojection errors
  double sse = 0.0;
  for (size_t i = 0; i < image_points.size(); i++) {
    sse += cv::norm(corners[i] - image_points[i]);
  }
  const double n = image_points.size();
  const double rmse = sqrt(sse / n);

  return rmse;
}

cv::Mat CalibValidator::project(const int camera_index,
                                const cv::Mat &image,
                                const MatX &X,
                                const cv::Scalar &color) {
  // Distort points
  MatX pixels = this->camchain.cam[camera_index].project(X);

  // Make an RGB version of the input image
  cv::Mat image_rgb = image.clone();
  if (image.channels() == 1) {
    image_rgb = cv::Mat(image.size(), CV_8UC3);
    image_rgb = image.clone();
    cv::cvtColor(image, image_rgb, CV_GRAY2RGB);
  }

  // Draw projected points
  std::vector<cv::Point2f> points;
  for (long i = 0; i < pixels.cols(); i++) {
    const Vec2 pixel = pixels.col(i);
    points.emplace_back(pixel(0), pixel(1));

    cv::circle(image_rgb,                       // Target image
               cv::Point2d{pixel(0), pixel(1)}, // Center
               1,                               // Radius
               color,                           // Colour
               CV_FILLED,                       // Thickness
               8);                              // Line type
  }

  // Calculate reprojection error and show in image
  const double rmse = this->reprojectionError(image_rgb, points);
  if (rmse > 0.0) {
    // Convert rmse to string
    std::stringstream stream;
    stream << fixed << setprecision(2) << rmse;
    const std::string rmse_str = stream.str();

    // Draw on image
    cv::putText(image_rgb,
                "RMSE Reprojection Error: " + rmse_str,
                cv::Point(0, 18),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(0, 0, 255),
                2);
  }

  return image_rgb;
}

cv::Mat CalibValidator::validate(const int camera_index, cv::Mat &image) {
  // Pre-check
  assert(camera_index >= 0);
  assert(camera_index < this->camchain.cam.size());
  assert(image.empty() == false);

  // Colors
  const cv::Scalar red{0, 0, 255};
  const cv::Scalar green{0, 255, 0};

  // Detect chessboard corners and output 3d positions
  const cv::Mat D;
  MatX X;
  if (this->detect(camera_index, image, X) != 1) {
    return image;
  }

  // Draw detected chessboard corners
  cv::Mat result = this->drawDetected(image, red);

  // Project 3D point to undistorted image
  return this->project(camera_index, result, X, green);
}

cv::Mat CalibValidator::validateStereo(const cv::Mat &img0,
                                       const cv::Mat &img1) {
  // Pre-check
  assert(img0.empty() == false);
  assert(img1.empty() == false);

  // Setup
  const cv::Scalar red{0, 0, 255};
  const cv::Scalar green{0, 255, 0};

  // Detect chessboard corners and output 3d positions
  MatX X0, X1;
  int retval = 0;
  retval += this->detect(0, img0, X0);
  retval += this->detect(1, img1, X1);
  if (retval != 2) {
    cv::Mat result;
    cv::vconcat(img0, img1, result);
    return result;
  }
  const cv::Mat img0_det = this->drawDetected(img0, red);
  const cv::Mat img1_det = this->drawDetected(img1, green);

  // Project points observed from cam1 to cam0 image
  // -- Make points homogeneous by adding 1's in last row
  X1.conservativeResize(X1.rows() + 1, X1.cols());
  X1.row(X1.rows() - 1) = ones(1, X1.cols());
  // -- Project and draw
  const Mat4 T_C1_C0 = this->camchain.T_C1_C0;
  const MatX X0_cal = (T_C1_C0.inverse() * X1).block(0, 0, 3, X1.cols());
  const cv::Mat img0_cb = this->project(0, img0_det, X0_cal, green);

  // Project points observed from cam0 to cam1 image
  // -- Make points homogeneous by adding 1's in last row
  X0.conservativeResize(X0.rows() + 1, X0.cols());
  X0.row(X0.rows() - 1) = ones(1, X0.cols());
  // -- Project and draw
  const MatX X1_cal = (T_C1_C0 * X0).block(0, 0, 3, X0.cols());
  const cv::Mat img1_cb = this->project(1, img1_det, X1_cal, red);

  // Combine cam0 and cam1 images
  cv::Mat result;
  cv::vconcat(img0_cb, img1_cb, result);
  return result;
}

cv::Mat CalibValidator::validateTriclops(const cv::Mat &img0,
                                         const cv::Mat &img1,
                                         const cv::Mat &img2,
                                         const double joint_roll,
                                         const double joint_pitch) {
  // Pre-check
  assert(img0.empty() == false);
  assert(img1.empty() == false);
  assert(img2.empty() == false);

  // Colors
  const cv::Scalar red{0, 0, 255};
  const cv::Scalar green{0, 255, 0};
  const cv::Scalar blue{255, 0, 0};

  // Detect chessboard corners, and return:
  // - Undistorted image
  // - Knew
  // - 3d corner positions
  MatX X0, X1, X2;
  const std::string dist_model0 = this->camchain.cam[0].distortion_model;
  const std::string dist_model1 = this->camchain.cam[1].distortion_model;
  const std::string dist_model2 = this->camchain.cam[2].distortion_model;
  int retval = 0;
  retval = this->detect(0, img0, X0);
  retval += this->detect(1, img1, X1);
  retval += this->detect(2, img2, X2);
  if (retval != 0) {
    cv::Mat img02;
    cv::hconcat(img0, img2, img02);
    cv::Mat img021;
    cv::hconcat(img02, img1, img021);
    return img021;
  }

  // Visualize cam0 and cam1 intrinsics
  cv::Mat img0_cb = this->project(0, img0, X0, red);
  cv::Mat img1_cb = this->project(1, img1, X1, green);

  // Get gimbal roll and pitch angles then form T_ds transform
  this->gimbal_model.Lambda1 = joint_roll;
  this->gimbal_model.Lambda2 = joint_pitch;
  const Mat4 T_ds = this->gimbal_model.T_ds();

  // Project points observed from cam0 to cam2 image
  // -- Make points homogeneous by adding 1's in last row
  X0.conservativeResize(X0.rows() + 1, X0.cols());
  X0.row(X0.rows() - 1) = ones(1, X0.cols());
  // -- Project and draw
  MatX X2_cal = (T_ds * X0).block(0, 0, 3, X0.cols());
  cv::Mat img2_cb = this->project(2, img2, X2_cal, red);

  // Project points observed from cam1 to cam2 image
  // -- Make points homogeneous by adding 1's in last row
  X1.conservativeResize(X1.rows() + 1, X1.cols());
  X1.row(X1.rows() - 1) = ones(1, X1.cols());
  // -- Project and draw
  const Mat4 T_C1_C0 = this->camchain.T_C1_C0;
  X2_cal = (T_ds * T_C1_C0.inverse() * X1).block(0, 0, 3, X1.cols());
  img2_cb = this->project(2, img2_cb, X2_cal, green);

  // Combine images
  cv::Mat img02;
  cv::hconcat(img0_cb, img2_cb, img02);
  cv::Mat img021;
  cv::hconcat(img02, img1_cb, img021);
  return img021;
}

} // namespace gvio
