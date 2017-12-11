#include "gvio/gvio_test.hpp"
#include "gvio/imu/mpu6050.hpp"

namespace gvio {

TEST(MPU6050, configure) {
  int retval;
  MPU6050 imu;

  retval = imu.configure();
  EXPECT_EQ(0.0, retval);
}

} // namespace gvio
