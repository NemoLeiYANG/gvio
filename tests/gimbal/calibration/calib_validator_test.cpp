#include "gvio/munit.hpp"
#include "gvio/camera/ids.hpp"
#include "gvio/gimbal/calibration/calib_validator.hpp"

namespace gvio {

#define TEST_CALIB_FILE "test_configs/gimbal/calibration/camchain.yaml"
#define TEST_TARGET_FILE "test_configs/gimbal/calibration/chessboard.yaml"
#define TEST_CAM0_CONFIG "test_configs/camera/ueye/cam0.yaml"
#define TEST_CAM1_CONFIG "test_configs/camera/ueye/cam1.yaml"
#define TEST_IMAGE "test_data/calibration2/img_0.jpg"

int test_CalibValidator_constructor() {
  CalibValidator validator;

  return 0;
}

int test_CalibValidator_load() {
  CalibValidator validator;

  // Load validator
  validator.load(3, TEST_CALIB_FILE, TEST_TARGET_FILE);
  std::cout << validator.cam[0] << std::endl;
  std::cout << validator.cam[1] << std::endl;
  std::cout << validator.cam[2] << std::endl;

  return 0;
}

int test_CalibValidator_validate() {
  CalibValidator validator;

  // Load validator
  validator.load(3, TEST_CALIB_FILE, TEST_TARGET_FILE);

  const cv::Mat image = cv::imread(TEST_IMAGE);
  const cv::Mat result = validator.validate(0, image);
  cv::imshow("Image", result);
  cv::waitKey();

  return 0;
}

int test_CalibValidator_validate_live() {
  CalibValidator validator;

  // Load validator
  validator.load(3, TEST_CALIB_FILE, TEST_TARGET_FILE);

  // Load camera
  IDSCamera camera;
  camera.configure(TEST_CAM0_CONFIG);

  // Loop webcam
  while (true) {
    cv::Mat image;
    if (camera.getFrame(image) != 0) {
      return -1;
    }

    cv::Mat result = validator.validate(0, image);
    cv::imshow("Validation", result);
    if (cv::waitKey(1) == 113) {
      break;
    }
  }

  return 0;
}

int test_CalibValidator_validateStereo_live() {
  CalibValidator validator;

  // Load validator
  validator.load(3, TEST_CALIB_FILE, TEST_TARGET_FILE);

  // Load camera
  IDSCamera cam0, cam1;
  cam0.configure(TEST_CAM0_CONFIG);
  cam1.configure(TEST_CAM1_CONFIG);

  // Loop webcam
  while (true) {
    cv::Mat img0, img1;
    if (cam0.getFrame(img0) != 0) {
      return -1;
    }
    if (cam1.getFrame(img1) != 0) {
      return -1;
    }

    cv::Mat result = validator.validateStereo(img0, img1);
    cv::imshow("Validation", result);
    if (cv::waitKey(1) == 113) {
      break;
    }
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_CalibValidator_constructor);
  MU_ADD_TEST(test_CalibValidator_load);
  // MU_ADD_TEST(test_CalibValidator_validate);
  MU_ADD_TEST(test_CalibValidator_validate_live);
  // MU_ADD_TEST(test_CalibValidator_validateStereo_live);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
