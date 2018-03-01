#include "gvio/munit.hpp"
#include "gvio/gimbal/calibration/chessboard.hpp"

namespace gvio {

#define TEST_CONFIG "test_configs/gimbal/calibration/chessboard.yaml"
#define TEST_IMAGE "test_data/calibration2/img_0.jpg"

int test_Chessboard_constructor() {
  Chessboard cb;

  MU_CHECK(cb.nb_rows == 10);
  MU_CHECK(cb.nb_cols == 10);
  MU_CHECK_FLOAT(cb.square_size, 0.1);

  return 0;
}

int test_Chessboard_load() {
  Chessboard cb;

  cb.load(TEST_CONFIG);
  MU_CHECK(cb.nb_rows == 6);
  MU_CHECK(cb.nb_cols == 7);
  MU_CHECK_FLOAT(cb.square_size, 0.0285);

  return 0;
}

int test_Chessboard_detect() {
  Chessboard cb;

  cb.load(TEST_CONFIG);
  const cv::Mat image = cv::imread(TEST_IMAGE);
  const std::vector<cv::Point2f> corners = cb.detect(image);
  MU_CHECK_EQ((int) corners.size(), cb.nb_rows * cb.nb_cols);

  // cv::imshow("Image", image);
  // cv::waitKey();

  return 0;
}

int test_Chessboard_drawCorners() {
  Chessboard cb;

  cb.load(TEST_CONFIG);
  cv::Mat image = cv::imread(TEST_IMAGE);
  const int retval = cb.drawCorners(image);
  MU_CHECK_EQ(retval, 0);

  // cv::imshow("Image", image);
  // cv::waitKey();

  return 0;
}

int test_Chessboard_solvePnP() {
  Chessboard cb;

  cb.load(TEST_CONFIG);
  const cv::Mat image = cv::imread(TEST_IMAGE);
  const std::vector<cv::Point2f> corners = cb.detect(image);

  // Solve PnP
  // -- Form camera intrinsics K
  cv::Mat K(3, 3, cv::DataType<double>::type);
  K.at<double>(0, 0) = 359.8796;
  K.at<double>(0, 1) = 0.0;
  K.at<double>(0, 2) = 341.8768;
  K.at<double>(1, 0) = 0.0;
  K.at<double>(1, 1) = 361.4580;
  K.at<double>(1, 2) = 255.9160;
  K.at<double>(2, 0) = 0.0;
  K.at<double>(2, 1) = 0.0;
  K.at<double>(2, 2) = 1.0;
  // -- Form camera distortion coefficients D
  cv::Mat D;
  // -- Transform
  Mat4 T_c_t;
  // -- Solve
  const int retval = cb.solvePnP(corners, K, D, T_c_t);
  std::cout << T_c_t << std::endl;

  MU_CHECK_EQ(retval, 0);

  return 0;
}

int test_Chessboard_calcCornerPositions() {
  Chessboard cb;

  cb.load(TEST_CONFIG);
  const cv::Mat image = cv::imread(TEST_IMAGE);
  const std::vector<cv::Point2f> corners = cb.detect(image);

  // Calculate corner positions
  // -- Form camera intrinsics K
  cv::Mat K(3, 3, cv::DataType<double>::type);
  K.at<double>(0, 0) = 359.8796;
  K.at<double>(0, 1) = 0.0;
  K.at<double>(0, 2) = 341.8768;
  K.at<double>(1, 0) = 0.0;
  K.at<double>(1, 1) = 361.4580;
  K.at<double>(1, 2) = 255.9160;
  K.at<double>(2, 0) = 0.0;
  K.at<double>(2, 1) = 0.0;
  K.at<double>(2, 2) = 1.0;
  // -- Form camera distortion coefficients D
  cv::Mat D;
  // -- Calculate
  MatX X;
  const int retval = cb.calcCornerPositions(corners, K, D, X);
  std::cout << X.transpose() << std::endl;

  MU_CHECK_EQ(retval, 0);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_Chessboard_constructor);
  MU_ADD_TEST(test_Chessboard_load);
  MU_ADD_TEST(test_Chessboard_detect);
  MU_ADD_TEST(test_Chessboard_drawCorners);
  MU_ADD_TEST(test_Chessboard_solvePnP);
  MU_ADD_TEST(test_Chessboard_calcCornerPositions);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
