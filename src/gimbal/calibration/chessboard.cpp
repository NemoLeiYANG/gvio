#include "gvio/gimbal/calibration/chessboard.hpp"

namespace gvio {

Chessboard::Chessboard() {}

Chessboard::~Chessboard() {}

int Chessboard::load(const std::string &config_file) {
  // Parse config file
  ConfigParser parser;
  parser.addParam("nb_rows", &this->nb_rows);
  parser.addParam("nb_cols", &this->nb_cols);
  parser.addParam("square_size", &this->square_size);
  if (parser.load(config_file) != 0) {
    LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
    return -1;
  }

  // Create object points
  this->object_points = this->createObjectPoints();

  return 0;
}

std::vector<Vec2> Chessboard::createGridPoints2d() {
  std::vector<Vec2> grid_points2d;
  for (int i = 0; this->nb_rows; i++) {
    for (int j = 0; this->nb_cols; j++) {
      auto p = Vec2{i, j} * this->square_size;
      grid_points2d.push_back(p);
    }
  }

  return grid_points2d;
}

std::vector<cv::Point3f> Chessboard::createObjectPoints() {
  std::vector<cv::Point3f> object_points;

  for (int i = 0; i < this->nb_rows; i++) {
    for (int j = 0; j < this->nb_cols; j++) {
      object_points.emplace_back(j * this->square_size,
                                 i * this->square_size,
                                 0.0);
    }
  }

  return object_points;
}

int Chessboard::detect(const cv::Mat &image,
                       std::vector<cv::Point2f> &corners) {
  // Find the chessboard corners
  // -- Convert image to grayscale
  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
  // -- Setup
  cv::Size size(this->nb_cols, this->nb_rows);
  // -- Detect chessboard corners
  if (cv::findChessboardCorners(image_gray, size, corners) == false) {
    return -1;
  }

  return 0;
}

int Chessboard::drawCorners(cv::Mat &image) {
  // Detect corners
  std::vector<cv::Point2f> corners;
  if (this->detect(image, corners) != 0) {
    return -1;
  }

  // Draw
  cv::Size size(this->nb_cols, this->nb_rows);
  cv::drawChessboardCorners(image, size, corners, true);

  return 0;
}

int Chessboard::solvePnP(const std::vector<cv::Point2f> corners,
                         const cv::Mat &K,
                         Mat4 &T_c_t) {

  // Calculate transformation matrix
  cv::Mat D;
  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);
  cv::solvePnP(this->object_points,
               corners,
               K,
               D,
               rvec,
               tvec,
               false,
               cv::SOLVEPNP_ITERATIVE);

  // Convert rotation vector to matrix
  cv::Mat R(3, 3, cv::DataType<double>::type);
  cv::Rodrigues(rvec, R);

  // Form transformation matrix
  // clang-format off
  T_c_t <<
    R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), tvec.at<double>(0),
    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), tvec.at<double>(1),
    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), tvec.at<double>(2),
    0.0, 0.0, 0.0, 1.0;
  // clang-format on

  return 0;
}

int Chessboard::calcCornerPositions(const std::vector<cv::Point2f> corners,
                                    const cv::Mat &K,
                                    MatX &X) {
  // Get transform from camera to chessboard
  Mat4 T_c_t;
  if (this->solvePnP(corners, K, T_c_t) != 0) {
    return -1;
  }

  // Convert object points from cv::Point3f to Eigen::Matrix
  const int nb_pts = this->nb_rows * this->nb_cols;
  MatX obj_pts_homo;
  obj_pts_homo.resize(4, nb_pts);
  for (int i = 0; i < nb_pts; i++) {
    obj_pts_homo(0, i) = this->object_points[i].x;
    obj_pts_homo(1, i) = this->object_points[i].y;
    obj_pts_homo(2, i) = this->object_points[i].z;
    obj_pts_homo(3, i) = 1.0;
  }

  // Calculate chessboard corners
  MatX X_homo;
  X_homo = T_c_t * obj_pts_homo;
  X = X_homo.block(0, 0, 3, nb_pts);

  return 0;
}

int Chessboard::calcCornerPositions(const std::vector<cv::Point2f> corners,
                                    const Mat3 &K,
                                    MatX &X) {
  const cv::Mat K_cam = convert(K);
  return this->calcCornerPositions(corners, K_cam, X);
}

void Chessboard::project3DPoints(const MatX &X, const Mat3 &K, cv::Mat &image) {
  // Project 3d point to image plane
  MatX x = K * X;

  for (int i = 0; i < x.cols(); i++) {
    const Vec3 p = x.col(i);
    const double px = p(0) / p(2);
    const double py = p(1) / p(2);

    cv::circle(image,                 // Target image
               cv::Point(px, py),     // Center
               3.0,                   // Radius
               cv::Scalar(0, 0, 255), // Colour
               CV_FILLED,             // Thickness
               8);                    // Line type
  }
}

void Chessboard::project3DPoints(const MatX &X,
                                 const cv::Mat &K,
                                 cv::Mat &image) {
  // clang-format off
  Mat3 K_cam;
  K_cam << K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
           K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
           K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);
  // clang-format on

  this->project3DPoints(X, K_cam, image);
}

} // namespace gvio
