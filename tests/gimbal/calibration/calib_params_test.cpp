#include "gvio/munit.hpp"
#include "gvio/gimbal/calibration/calib_params.hpp"

namespace gvio {

#define TEST_CONFIG_FILE "test_configs/gimbal/calibration/params.yaml"
#define TEST_JOINT_DATA "test_data/calibration/joint.csv"

int test_CalibParmas_constructor() {
  CalibParams params;

  MU_CHECK(params.tau_s == nullptr);
  MU_CHECK(params.tau_d == nullptr);
  MU_CHECK(params.w1 == nullptr);
  MU_CHECK(params.w2 == nullptr);
  MU_CHECK(params.Lambda1 == nullptr);
  MU_CHECK(params.Lambda2 == nullptr);

  return 0;
}

int test_CalibParmas_load() {
  CalibParams params;
  params.load(TEST_CONFIG_FILE, TEST_JOINT_DATA);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_CalibParmas_constructor);
  MU_ADD_TEST(test_CalibParmas_load);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
