#include "gvio/munit.hpp"
#include "gvio/calibration/aprilgrid.hpp"

#define TEST_IMAGE "test_data/calibration/cam0/0.jpg"

namespace gvio {

int test_AprilGrid_constructor() {
  AprilGrid grid;

  // auto capture = cv::VideoCapture(0);
  // // const cv::Mat image = cv::imread(TEST_IMAGE);
  // if (capture.isOpened() == false) {
  //   return -1;
  // }
  //
  // while (true) {
  //   cv::Mat image;
  //   capture.read(image);
  //
  //   grid.detect(image);
  //   cv::imshow("Image", image);
  //   if (cv::waitKey(1) == 113) {
  //     break;
  //   }
  // }

  // cv::imshow("Image", image);
  // cv::waitKey(1);

  return 0;
}

int test_AprilGrid_detect() {
  AprilGrid grid;

  cv::Mat image = cv::imread(TEST_IMAGE);
  int retval = grid.detect(image);

  MU_CHECK_EQ(retval, 0);

  // cv::imshow("Image", image);
  // cv::waitKey();

  return 0;
}

int test_AprilGrid_extractTags() {
  AprilGrid grid;

  cv::Mat image = cv::imread(TEST_IMAGE);

  std::map<int, std::vector<cv::Point2f>> tags;
  int retval = grid.extractTags(image, tags);

  MU_CHECK_EQ(tags.size(), 36);
  MU_CHECK_EQ(retval, 0);

  // cv::imshow("Image", image);
  // cv::waitKey();

  return 0;
}

int test_AprilGrid_formObjectPoints() {
  AprilGrid grid(6, 6, 0.088, 0.3);

  cv::Mat image = cv::imread(TEST_IMAGE);

  std::map<int, std::vector<cv::Point2f>> tags;
  int retval = grid.extractTags(image, tags);
  std::vector<cv::Point3f> object_points = grid.formObjectPoints(tags);

  MU_CHECK_EQ(retval, 0);

  // cv::imshow("Image", image);
  // cv::waitKey();

  return 0;
}

int test_AprilGrid_formImagePoints() {
  AprilGrid grid(6, 6, 0.088, 0.3);

  cv::Mat image = cv::imread(TEST_IMAGE);

  std::map<int, std::vector<cv::Point2f>> tags;
  int retval = grid.extractTags(image, tags);
  std::vector<cv::Point3f> object_points = grid.formObjectPoints(tags);
  std::vector<cv::Point2f> image_points = grid.formImagePoints(tags);

  MU_CHECK_EQ(object_points.size(), image_points.size());
  MU_CHECK_EQ(retval, 0);

  return 0;
}

int test_AprilGrid_solvePnP() {
  AprilGrid grid(6, 6, 0.088, 0.3);

  cv::Mat image = cv::imread(TEST_IMAGE);

  std::map<int, std::vector<cv::Point2f>> tags;
  int retval = grid.extractTags(image, tags);

  const double fx = 393.3799;
  const double fy = 395.3197;
  const double cx = 370.0501;
  const double cy = 239.3825;
  Mat3 K;
  K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
  Vec4 D;
  D << -0.0678890809361792, 0.007709008000713503, -0.010307689056822753,
      0.003975705353253985;

  MatX grid_points;
  grid.solvePnP(tags, convert(K), grid_points);

  // Draw projected points
  for (long i = 0; i < grid_points.rows(); i++) {
    const Vec3 X = grid_points.row(i);
    const Vec2 x = project_pinhole_equi(K, D, X);

    cv::circle(image,                   // Target image
               cv::Point2f(x(0), x(1)), // Center
               1.0,                     // Radius
               cv::Scalar(0, 0, 255),   // Colour
               CV_FILLED,               // Thickness
               8);                      // Line type
  }

  MU_CHECK_EQ(retval, 0);

  // cv::imshow("Image", image);
  // cv::waitKey();

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_AprilGrid_constructor);
  MU_ADD_TEST(test_AprilGrid_detect);
  MU_ADD_TEST(test_AprilGrid_extractTags);
  MU_ADD_TEST(test_AprilGrid_formObjectPoints);
  MU_ADD_TEST(test_AprilGrid_formImagePoints);
  MU_ADD_TEST(test_AprilGrid_solvePnP);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
