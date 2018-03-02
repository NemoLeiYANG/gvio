#include "gvio/munit.hpp"
#include "gvio/util/vision.hpp"
#include "gvio/camera/camera_config.hpp"

#define TEST_CONFIG "test_configs/camera/webcam/640x480.yaml"

namespace gvio {

int test_CameraConfig_constructor() {
  CameraConfig config;

  MU_FALSE(config.loaded);

  MU_CHECK(0 == config.index);
  MU_CHECK(0 == config.image_width);
  MU_CHECK(0 == config.image_height);

  MU_CHECK_FLOAT(0.0, config.exposure_value);
  MU_CHECK_FLOAT(0.0, config.gain_value);

  MU_CHECK_FLOAT(0.0, config.lambda(0));
  MU_CHECK_FLOAT(0.0, config.lambda(1));
  MU_CHECK_FLOAT(0.0, config.lambda(2));
  MU_CHECK_FLOAT(0.0, config.alpha);

  // config.camera_matrix;
  // config.rectification_matrix;
  // config.distortion_coefficients;
  // config.projection_matrix;

  MU_FALSE(config.imshow);
  MU_FALSE(config.snapshot);

  return 0;
}

int test_CameraConfig_load() {
  // clang-format off
  CameraConfig config;
  double camera_matrix[] = {1.0, 2.0, 3.0,
                            4.0, 5.0, 6.0,
                            7.0, 8.0, 9.0};
  double distortion_coefficients[] = {1.0, 2.0, 3.0, 4.0, 5.0};
  double rectification_matrix[] = {1.0, 2.0, 3.0,
                                   4.0, 5.0, 6.0,
                                   7.0, 8.0, 9.0};
  double projection_matrix[] = {1.0, 2.0, 3.0, 4.0,
                                5.0, 6.0, 7.0, 8.0,
                                9.0, 10.0, 11.0, 12.0};
  // clang-format on

  // test and assert
  config.load(TEST_CONFIG);

  MU_CHECK(config.loaded);

  MU_CHECK(0 == config.index);
  MU_CHECK(640 == config.image_width);
  MU_CHECK(480 == config.image_height);

  MU_CHECK_FLOAT(1.0, config.exposure_value);
  MU_CHECK_FLOAT(2.0, config.gain_value);
  MU_CHECK_FLOAT(1.0, config.lambda(0));
  MU_CHECK_FLOAT(2.0, config.lambda(1));
  MU_CHECK_FLOAT(3.0, config.lambda(2));
  MU_CHECK_FLOAT(4.0, config.alpha);

  // clang-format off
  cv::Mat expected(3, 3, CV_64F, camera_matrix);
  MU_CHECK(is_equal(expected, config.camera_matrix));

  expected = cv::Mat(1, 5, CV_64F, distortion_coefficients);
  MU_CHECK(is_equal(expected, config.distortion_coefficients));

  expected = cv::Mat(3, 3, CV_64F, rectification_matrix);
  MU_CHECK(is_equal(expected, config.rectification_matrix));

  expected = cv::Mat(3, 4, CV_64F, projection_matrix);
  MU_CHECK(is_equal(expected, config.projection_matrix));
  // clang-format on

  MU_CHECK(config.imshow);
  MU_CHECK(config.snapshot);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_CameraConfig_constructor);
  MU_ADD_TEST(test_CameraConfig_load);
}

} // namespace gvio
MU_RUN_TESTS(gvio::test_suite);
