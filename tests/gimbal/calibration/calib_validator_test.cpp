#include "gvio/munit.hpp"
#include "gvio/gimbal/calibration/calib_validator.hpp"

namespace gvio {

#define TEST_IMAGE "test_data/calibration2/img_0.jpg"
#define TEST_CALIB_FILE "test_data/calibration2/intrinsics.yaml"
#define TEST_TARGET_FILE "test_configs/gimbal/calibration/chessboard.yaml"

int test_CalibValidator_constructor() {
  CalibValidator validator;

  return 0;
}

int test_CalibValidator_validate() {
  CalibValidator validator;

  // Load validator
  validator.load(TEST_CALIB_FILE, TEST_TARGET_FILE);

  const cv::Mat image = cv::imread(TEST_IMAGE);
  const cv::Mat result = validator.validate(image);
  cv::imshow("Image", result);
  cv::waitKey();

  // // Load webcam
  // auto capture = cv::VideoCapture(0);
  // if (capture.isOpened() == false) {
  //   return -1;
  // }
  //
  // // Loop webcam
  // while (true) {
  //   cv::Mat image;
  //   capture.read(image);
  //
  //   cv::Mat result = validator.validate(image);
  //   cv::imshow("Validation", result);
  //   if (cv::waitKey(1) == 113) {
  //     break;
  //   }
  // }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_CalibValidator_constructor);
  MU_ADD_TEST(test_CalibValidator_validate);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
