#include "gvio/gimbal/calibration/calib_validator.hpp"

namespace gvio {

std::ostream &operator<<(std::ostream &os, const CameraProperty &cam) {
  os << "camera_model: " << cam.camera_model << std::endl;
  os << "distortion_model: " << cam.distortion_model << std::endl;
  os << "distortion_coeffs: " << cam.distortion_coeffs.transpose() << std::endl;
  os << "intrinsics: " << cam.intrinsics.transpose() << std::endl;
  os << "resolution: " << cam.resolution.transpose() << std::endl;
  return os;
}

CalibValidator::CalibValidator() {}

CalibValidator::~CalibValidator() {}

Mat3 CalibValidator::K(const int cam_id) {
  assert(cam_id >= 0);
  assert(cam_id < this->cam.size());

  Mat3 K;
  // First row
  K(0, 0) = this->cam[cam_id].intrinsics[0];
  K(0, 1) = 0.0;
  K(0, 2) = this->cam[cam_id].intrinsics[1];

  // Second row
  K(1, 0) = 0.0;
  K(1, 1) = this->cam[cam_id].intrinsics[2];
  K(1, 2) = this->cam[cam_id].intrinsics[3];

  // Thrid row
  K(2, 0) = 0.0;
  K(2, 1) = 0.0;
  K(2, 2) = 1.0;

  return K;
}

VecX CalibValidator::D(const int cam_id) {
  assert(cam_id >= 0);
  assert(cam_id < this->cam.size());

  VecX D;
  D.resize(4);

  // clang-format off
  D << this->cam[cam_id].distortion_coeffs(0),
       this->cam[cam_id].distortion_coeffs(1),
       this->cam[cam_id].distortion_coeffs(2),
       this->cam[cam_id].distortion_coeffs(3);
  // clang-format on

  return D;
}

int CalibValidator::load(const int nb_cameras,
                         const std::string &calib_file,
                         const std::string &target_file) {
  assert(nb_cameras > 0);
  assert(calib_file.empty() == false);
  assert(target_file.empty() == false);

  // Parse calib file
  ConfigParser parser;
  CameraProperty cam0, cam1, cam2;
  for (int i = 0; i < nb_cameras; i++) {
    switch (i) {
      case 0:
        parser.addParam("cam0.camera_model", &cam0.camera_model);
        parser.addParam("cam0.distortion_model", &cam0.distortion_model);
        parser.addParam("cam0.distortion_coeffs", &cam0.distortion_coeffs);
        parser.addParam("cam0.intrinsics", &cam0.intrinsics);
        parser.addParam("cam0.resolution", &cam0.resolution);
        break;
      case 1:
        parser.addParam("cam1.camera_model", &cam1.camera_model);
        parser.addParam("cam1.distortion_model", &cam1.distortion_model);
        parser.addParam("cam1.distortion_coeffs", &cam1.distortion_coeffs);
        parser.addParam("cam1.intrinsics", &cam1.intrinsics);
        parser.addParam("cam1.resolution", &cam1.resolution);
        break;
      case 2:
        parser.addParam("cam2.camera_model", &cam2.camera_model);
        parser.addParam("cam2.distortion_model", &cam2.distortion_model);
        parser.addParam("cam2.distortion_coeffs", &cam2.distortion_coeffs);
        parser.addParam("cam2.intrinsics", &cam2.intrinsics);
        parser.addParam("cam2.resolution", &cam2.resolution);
        break;
    }
  }
  parser.addParam("T_C1_C0", &this->T_C1_C0);
  parser.addParam("T_C2_C0.tau_s", &this->gimbal_model.tau_s);
  parser.addParam("T_C2_C0.tau_d", &this->gimbal_model.tau_d);
  parser.addParam("T_C2_C0.w1", &this->gimbal_model.w1);
  parser.addParam("T_C2_C0.w2", &this->gimbal_model.w2);
  parser.addParam("T_C2_C0.link1_offset", &this->gimbal_model.link1_offset);
  parser.addParam("T_C2_C0.link2_offset", &this->gimbal_model.link2_offset);
  if (parser.load(calib_file) != 0) {
    LOG_ERROR("Failed to load calib file [%s]!", calib_file.c_str());
    return -1;
  }
  this->cam = {cam0, cam1, cam2};

  // Chessboard
  if (this->chessboard.load(target_file) != 0) {
    LOG_ERROR("Failed to load chessboard config!");
    return -1;
  }

  return 0;
}

int CalibValidator::detect(const cv::Mat &image,
                           const Mat3 &K,
                           const VecX &D,
                           cv::Mat &image_ud,
                           cv::Mat &Knew,
                           MatX &X) {
  // Undistort image
  image_ud = pinhole_equi_undistort_image(K, D, image, 0.5, Knew);

  // Find chessboard corners (with distorted image)
  std::vector<cv::Point2f> corners;
  if (this->chessboard.detect(image, corners) != 0) {
    return -1;
  }

  // Undistort points
  std::vector<cv::Point2f> corners_ud;
  // -- Note: we are using the original K to undistort the points
  cv::fisheye::undistortPoints(corners, corners_ud, convert(K), convert(D));
  // -- Convert undistorted points from ideal to pixel coordinates to the
  // undistorted image with Knew (calculated when undistorting the image*)
  for (size_t i = 0; i < corners_ud.size(); i++) {
    const Vec3 p{corners_ud[i].x, corners_ud[i].y, 1.0};
    const Vec3 x = K * p;
    corners_ud[i].x = x(0);
    corners_ud[i].y = x(1);
  }

  // Solve PnP (with undistorted points)
  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);
  cv::Mat D_empty;
  cv::solvePnP(chessboard.object_points,
               corners_ud,
               convert(K),
               D_empty, // D is empty because corners are undistorted
               rvec,
               tvec,
               false,
               cv::SOLVEPNP_ITERATIVE);

  // Calculate corner positions in 3D
  this->chessboard.calcCornerPositions(corners_ud, K, X);

  return 0;
}

cv::Mat CalibValidator::projectAndDraw(const cv::Mat &image,
                                       const Mat3 &K,
                                       const MatX &X,
                                       const cv::Scalar &color) {
  // Project 3D point to image plane
  std::vector<cv::Point2f> image_points;
  for (int i = 0; i < X.cols(); i++) {
    Vec3 x = K * X.col(i);
    image_points.emplace_back(x(0) / x(2), x(1) / x(2));
  }

  // Make an RGB version of the input image
  cv::Mat result(image.size(), CV_8UC3);
  result = image.clone();
  cv::cvtColor(image, result, CV_GRAY2RGB);

  // Draw projected points
  for (size_t i = 0; i < image_points.size(); i++) {
    cv::circle(result,          // Target image
               image_points[i], // Center
               2.0,             // Radius
               color,           // Colour
               CV_FILLED,       // Thickness
               8);              // Line type
  }

  return result;
}

cv::Mat CalibValidator::projectAndDraw(const cv::Mat &image,
                                       const Mat3 &K,
                                       const VecX &D,
                                       const MatX &X,
                                       const cv::Scalar &color) {
  // Convert points from Eigen::Matrix to vector of cv::Point2f
  std::vector<cv::Point2f> points;
  for (long i = 0; i < X.cols(); i++) {
    const Vec3 p = X.col(i);
    points.emplace_back(p(0) / p(2), p(1) / p(2));
  }

  // Undistort points
  std::vector<cv::Point2f> distorted_points;
  cv::fisheye::distortPoints(points, distorted_points, convert(K), convert(D));

  // Make an RGB version of the input image
  cv::Mat image_rgb(image.size(), CV_8UC3);
  image_rgb = image.clone();
  cv::cvtColor(image, image_rgb, CV_GRAY2RGB);

  // Draw projected points
  for (size_t i = 0; i < distorted_points.size(); i++) {
    cv::circle(image_rgb,              // Target image
               distorted_points[i], // Center
               2.0,                 // Radius
               color,               // Colour
               CV_FILLED,           // Thickness
               8);                  // Line type
  }

  return image_rgb;
}

cv::Mat CalibValidator::validate(const int cam_id, const cv::Mat &image) {
  // Pre-check
  assert(cam_id >= 0);
  assert(cam_id < this->cam.size());
  assert(image.empty() == false);

  // Detect chessboard corners, and return:
  // - Undistorted image
  // - Knew
  // - 3d corner positions
  cv::Mat image_ud;
  cv::Mat Knew;
  MatX X;
  if (this->detect(image,
                   this->K(cam_id),
                   this->D(cam_id),
                   image_ud,
                   Knew,
                   X) != 0) {
    return image_ud;
  }

  return this->projectAndDraw(image_ud, convert(Knew), X);
}

cv::Mat CalibValidator::validateStereo(const cv::Mat &img0,
                                       const cv::Mat &img1) {
  // Pre-check
  assert(img0.empty() == false);
  assert(img1.empty() == false);

  // Detect chessboard corners, and return:
  // - Undistorted image
  // - Knew
  // - 3d corner positions
  cv::Mat img0_ud, img1_ud;
  cv::Mat K0_new, K1_new;
  MatX X0, X1;
  int retval = 0;
  retval = this->detect(img0, this->K(0), this->D(0), img0_ud, K0_new, X0);
  retval += this->detect(img1, this->K(1), this->D(1), img1_ud, K1_new, X1);
  if (retval != 0) {
    cv::Mat result;
    cv::vconcat(img0, img1, result);
    return result;
  }

  // Project points observed from cam1 to cam0 image
  // -- Make points homogeneous by adding 1's in last row
  X1.conservativeResize(X1.rows() + 1, X1.cols());
  X1.row(X1.rows() - 1) = ones(1, X1.cols());
  // -- Project and draw
  MatX X0_cal = (this->T_C1_C0.inverse() * X1).block(0, 0, 3, X1.cols());
  cv::Mat img0_cb = this->projectAndDraw(img0, this->K(0), this->D(0), X0_cal);

  // Project points observed from cam0 to cam1 image
  // -- Make points homogeneous by adding 1's in last row
  X0.conservativeResize(X0.rows() + 1, X0.cols());
  X0.row(X0.rows() - 1) = ones(1, X0.cols());
  // -- Project and draw
  MatX X1_cal = (this->T_C1_C0 * X0).block(0, 0, 3, X0.cols());
  cv::Mat img1_cb = this->projectAndDraw(img1, this->K(1), this->D(1), X1_cal);

  // Combine cam0 and cam1 images
  cv::Mat result;
  cv::vconcat(img0_cb, img1_cb, result);
  return result;
}

cv::Mat CalibValidator::validateTriclops(const cv::Mat &img0,
                                         const cv::Mat &img1,
                                         const cv::Mat &img2,
                                         const double roll,
                                         const double pitch) {
  // Pre-check
  assert(img0.empty() == false);
  assert(img1.empty() == false);
  assert(img2.empty() == false);

  // Detect chessboard corners, and return:
  // - Undistorted image
  // - Knew
  // - 3d corner positions
  cv::Mat img0_ud, img1_ud, img2_ud;
  cv::Mat K0_new, K1_new, K2_new;
  MatX X0, X1, X2;
  int retval = 0;
  retval = this->detect(img0, this->K(0), this->D(0), img0_ud, K0_new, X0);
  retval += this->detect(img1, this->K(1), this->D(1), img1_ud, K1_new, X1);
  retval += this->detect(img2, this->K(2), this->D(2), img2_ud, K2_new, X2);
  if (retval != 0) {
    return img2;
  }

  // Get gimbal roll and pitch agnles then form T_ds transform
  this->gimbal_model.Lambda1 = roll + this->gimbal_model.link1_offset;
  this->gimbal_model.Lambda2 = pitch;
  const Mat4 T_ds = this->gimbal_model.T_ds();

  // Project points observed from cam0 to cam2 image
  // -- Make points homogeneous by adding 1's in last row
  X0.conservativeResize(X0.rows() + 1, X0.cols());
  X0.row(X0.rows() - 1) = ones(1, X0.cols());
  // -- Project and draw
  MatX X2_cal = (T_ds * X0).block(0, 0, 3, X0.cols());
  cv::Mat img2_cb = this->projectAndDraw(img2, this->K(2), this->D(2), X2_cal);

  // Combine images
  return img2_cb;
}

} // namespace gvio
