#include "gvio/munit.hpp"
#include "gvio/gimbal/calibration/residual.hpp"

namespace gvio {

int test_GimbalCalibResidual_dhTransform() {
  GimbalCalibResidual err;

  auto result = err.dhTransform(0.0, 0.0, 0.0, 0.0);
  std::cout << result << std::endl;

  return 0;
}

int test_GimbalCalibResidual_euler321ToRot() {
  GimbalCalibResidual err;

  double rpy[3] = {0.0, 0.0, 0.0};
  auto result = err.euler321ToRot(rpy);
  std::cout << result << std::endl;

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_GimbalCalibResidual_dhTransform);
  MU_ADD_TEST(test_GimbalCalibResidual_euler321ToRot);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
