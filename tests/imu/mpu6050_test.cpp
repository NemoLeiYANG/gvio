#include "gvio/munit.h"
#include "gvio/imu/mpu6050.hpp"

namespace gvio {

int test_MPU6050_configure() {
  int retval;
  MPU6050 imu;

  retval = imu.configure();
  MU_CHECK_EQ(0.0, retval);

  return 0;
}

void test_suite() { MU_ADD_TEST(test_MPU6050_configure); }

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
