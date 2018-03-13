#include "gvio/munit.hpp"
#include "gvio/camera/ids.hpp"
#include "gvio/gimbal/calibration/calib_validator.hpp"
#include "gvio/gimbal/gimbal.hpp"

namespace gvio {

#define TEST_CALIB_FILE "test_configs/gimbal/calibration/camchain.yaml"
#define TEST_TARGET_FILE "test_configs/gimbal/calibration/chessboard.yaml"
#define TEST_CAM0_CONFIG "test_configs/camera/ueye/cam0.yaml"
#define TEST_CAM1_CONFIG "test_configs/camera/ueye/cam1.yaml"
#define TEST_CAM2_CONFIG "test_configs/camera/ueye/cam2.yaml"
#define TEST_IMAGE "test_data/calibration2/img_0.jpg"
#define TEST_GIMBAL_CONFIG "test_configs/gimbal/config2.yaml"

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
  std::cout << "T_C1_C0:\n" << validator.T_C1_C0 << std::endl << std::endl;
  std::cout << validator.gimbal_model << std::endl;

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
  // Load validator
  CalibValidator validator;
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

int test_CalibValidator_validateTriclops_live() {
  // Load validator
  CalibValidator validator;
  if (validator.load(3, TEST_CALIB_FILE, TEST_TARGET_FILE) != 0) {
    LOG_ERROR("Failed to load validation!");
    return -1;
  }

  // Load gimbal
  Gimbal gimbal;
  if (gimbal.configure(TEST_GIMBAL_CONFIG) != 0) {
    LOG_ERROR("Failed to load gimbal!");
    return -1;
  }

  // Load camera
  IDSCamera cam0, cam1, cam2;
  if (cam0.configure(TEST_CAM0_CONFIG) != 0) {
    LOG_ERROR("Failed to load cam0!");
    return -1;
  }
  if (cam1.configure(TEST_CAM1_CONFIG) != 0) {
    LOG_ERROR("Failed to load cam1!");
    return -1;
  }
  if (cam2.configure(TEST_CAM2_CONFIG) != 0) {
    LOG_ERROR("Failed to load cam2!");
    return -1;
  }

  // Loop webcam
  while (true) {
    cv::Mat img0, img1, img2;
    if (cam0.getFrame(img0) != 0) {
      return -1;
    }
    if (cam1.getFrame(img1) != 0) {
      return -1;
    }
    if (cam2.getFrame(img2) != 0) {
      return -1;
    }

    gimbal.update();
    const double joint_roll = gimbal.camera_angles(0) - gimbal.frame_angles(0);
    const double joint_pitch = gimbal.camera_angles(1) - gimbal.frame_angles(1);
    cv::Mat result =
        validator.validateTriclops(img0, img1, img2, joint_roll, joint_pitch);
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
  // MU_ADD_TEST(test_CalibValidator_validate_live);
  // MU_ADD_TEST(test_CalibValidator_validateStereo_live);
  // MU_ADD_TEST(test_CalibValidator_validateTriclops_live);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
