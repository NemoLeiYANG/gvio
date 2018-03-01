#include "gvio/munit.hpp"
#include "gvio/gimbal/calibration/gimbal_calib.hpp"

namespace gvio {

// #define TEST_CONFIG_FILE "test_configs/gimbal/calibration/params.yaml"
// #define TEST_JOINT_DATA "test_data/calibration/joint.csv"
#define TEST_DATA "/home/chutsu/Dropbox/measurements"

int test_GimbalCalib_constructor() {
  GimbalCalib calib;
  return 0;
}

int test_GimbalCalib_load() {
  GimbalCalib calib;

  calib.load(TEST_DATA);
  calib.calibrate();

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_GimbalCalib_constructor);
  MU_ADD_TEST(test_GimbalCalib_load);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
