#include "gvio/munit.hpp"
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

  // Write output file header
  const std::string header = "t,x,y,z,roll,pitch,yaw";
  output_file << header << std::endl;

  // Setup robot
  TwoWheelRobot robot;
  double t_end = 0.0;
  const double dt = 0.1;

  double wz_B = 0.0;
  double ax_B = 0.0;
  circle_trajectory(10.0, 1.0, &wz_B, &t_end);
  robot.v_B(0) = 1.0;

  // Record initial robot state
  output_file << 0.0 << ",";
  output_file << robot.p_G(0) << ",";
  output_file << robot.p_G(1) << ",";
  output_file << robot.p_G(2) << ",";
  output_file << robot.rpy_G(0) << ",";
  output_file << robot.rpy_G(1) << ",";
  output_file << robot.rpy_G(2) << std::endl;

  // Simulate robot motion
  for (double t = 0.0; t < t_end; t += dt) {
    // Update
    robot.update(ax_B, wz_B, dt);

    // Record robot state
    output_file << t << ",";
    output_file << robot.p_G(0) << ",";
    output_file << robot.p_G(1) << ",";
    output_file << robot.p_G(2) << ",";
    output_file << robot.rpy_G(0) << ",";
    output_file << robot.rpy_G(1) << ",";
    output_file << robot.rpy_G(2) << std::endl;
  }
  PYTHON_SCRIPT("scripts/plot_twowheel.py /tmp/twowheel.dat")

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_TwoWheel_constructor);
  MU_ADD_TEST(test_TwoWheel_update);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
