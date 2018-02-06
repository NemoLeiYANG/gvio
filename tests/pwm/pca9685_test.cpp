#include "gvio/munit.hpp"
#include "gvio/pwm/pca9685.hpp"

namespace gvio {

int test_PCA9685_constructor() {
  PCA9685 pwm_driver;
  return 0;
}

int test_PCA9685_configure() {
  PCA9685 pwm_driver;
  pwm_driver.configure(100);
  return 0;
}

int test_PCA9685_setPWM() {
  PCA9685 pwm_driver;
  pwm_driver.configure(30);
  pwm_driver.setAllPWM(3685.5);
  sleep(100);
  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_PCA9685_constructor);
  MU_ADD_TEST(test_PCA9685_configure);
  MU_ADD_TEST(test_PCA9685_setPWM);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
