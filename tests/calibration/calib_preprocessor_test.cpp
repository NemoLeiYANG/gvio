#include "gvio/munit.hpp"
#include "gvio/calibration/calib_preprocessor.hpp"

namespace gvio {

#define TEST_DATA "test_data/calibration"

int test_CalibPreprocessor_constructor() {
  CalibPreprocessor preprocessor;

  return 0;
}

int test_CalibPreprocessor_preprocess() {
  CalibPreprocessor preprocessor;
  preprocessor.preprocess(TEST_DATA);

  std::cout << preprocessor.target << std::endl;
  std::cout << preprocessor.camera_properties[0] << std::endl;
  std::cout << preprocessor.camera_properties[1] << std::endl;
  std::cout << preprocessor.camera_properties[2] << std::endl;

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_CalibPreprocessor_constructor);
  MU_ADD_TEST(test_CalibPreprocessor_preprocess);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
