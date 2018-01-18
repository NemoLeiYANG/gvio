#include "gvio/munit.h"
#include "gvio/sim/twowheel.hpp"

namespace gvio {

int test_TwoWheel_constructor() {
  TwoWheelRobot robot;

  MU_CHECK(robot.p_G.isApprox(Vec3::Zero()));
  MU_CHECK(robot.rpy_G.isApprox(Vec3::Zero()));
  MU_CHECK(robot.w_B.isApprox(Vec3::Zero()));
  MU_CHECK(robot.v_B.isApprox(Vec3::Zero()));
  MU_CHECK(robot.a_B.isApprox(Vec3::Zero()));

  return 0;
}

int test_TwoWheel_update() {
  // Setup output file
  std::ofstream output_file("/tmp/twowheel.dat");
  if (output_file.good() == false) {
    LOG_ERROR("Failed to open file for output!");
    return -1;
  }

  // Setup robot
  TwoWheelRobot robot;
  const double dt = 0.1;
  double ax_B = 0.01;
  double wz_B = 0.1;

  for (int i = 0; i < 100; i++) {
    if (robot.v_B(0) > 1.0) {
      ax_B = 0.0;
    }

    robot.update(ax_B, wz_B, dt);

    // Record robot state
    output_file << robot.p_G(0) << ",";
    output_file << robot.p_G(1) << ",";
    output_file << robot.p_G(2) << ",";
    output_file << robot.rpy_G(0) << ",";
    output_file << robot.rpy_G(1) << ",";
    output_file << robot.rpy_G(2) << std::endl;
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_TwoWheel_constructor);
  MU_ADD_TEST(test_TwoWheel_update);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
