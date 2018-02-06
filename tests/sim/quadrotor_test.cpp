#include "gvio/munit.hpp"
#include "gvio/sim/quadrotor.hpp"
#include "gvio/sim/carrot_controller.hpp"

namespace gvio {

int test_QuadrotorModel_constructor() {
  QuadrotorModel quad;
  return 0;
}

int test_QuadrotorModel_update() {
  // Ground truth file
  const std::string gnd_file_path = "/tmp/quadrotor_gnd.dat";
  std::ofstream gnd_file(gnd_file_path);
  if (gnd_file.good() == false) {
    LOG_ERROR("Failed to open ground truth file for recording [%s]",
              gnd_file_path.c_str());
    return -1;
  }

  // Measurement file
  const std::string mea_file_path = "/tmp/quadrotor_mea.dat";
  std::ofstream mea_file(mea_file_path);
  if (mea_file.good() == false) {
    LOG_ERROR("Failed to open measurement file for recording [%s]",
              mea_file_path.c_str());
    return -1;
  }

  // Write header
  const std::string gnd_header = "t,x,y,z,vx,vy,vz,roll,pitch,yaw";
  gnd_file << gnd_header << std::endl;
  const std::string mea_header = "t,ax_B,ay_B,az_B,wx_B,wy_B,wz_B";
  mea_file << mea_header << std::endl;

  // Simulate
  QuadrotorModel quad(Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 0.0, 5.0});
  const double dt = 0.001;

  // Record initial quadrotor state
  gnd_file << 0.0 << ",";
  gnd_file << quad.p_G(0) << ",";
  gnd_file << quad.p_G(1) << ",";
  gnd_file << quad.p_G(2) << ",";
  gnd_file << quad.v_G(0) << ",";
  gnd_file << quad.v_G(1) << ",";
  gnd_file << quad.v_G(2) << ",";
  gnd_file << quad.rpy_G(0) << ",";
  gnd_file << quad.rpy_G(1) << ",";
  gnd_file << quad.rpy_G(2) << std::endl;

  const Vec3 a_B = quad.getBodyAcceleration();
  const Vec3 w_B = quad.getBodyAngularVelocity();
  mea_file << 0.0 << ",";
  mea_file << a_B(0) << ",";
  mea_file << a_B(1) << ",";
  mea_file << a_B(2) << ",";
  mea_file << w_B(0) << ",";
  mea_file << w_B(1) << ",";
  mea_file << w_B(2) << std::endl;

  CarrotController controller;
  std::vector<Vec3> waypoints;
  waypoints.emplace_back(0.0, 0.0, 5.0);
  waypoints.emplace_back(5.0, 0.0, 5.0);
  waypoints.emplace_back(5.0, 5.0, 5.0);
  waypoints.emplace_back(0.0, 5.0, 5.0);
  waypoints.emplace_back(0.0, 0.0, 5.0);
  controller.configure(waypoints, 0.1);

  quad.setPosition(1.0, 1.0, 5.0);
  for (double t = 0.0; t <= 25.0; t += dt) {
    // Update
    Vec3 carrot_pt;
    controller.update(quad.p_G, carrot_pt);
    quad.setPosition(carrot_pt(0), carrot_pt(1), carrot_pt(2));
    quad.update(dt);

    // Record quadrotor state
    gnd_file << t << ",";
    gnd_file << quad.p_G(0) << ",";
    gnd_file << quad.p_G(1) << ",";
    gnd_file << quad.p_G(2) << ",";
    gnd_file << quad.v_G(0) << ",";
    gnd_file << quad.v_G(1) << ",";
    gnd_file << quad.v_G(2) << ",";
    gnd_file << quad.rpy_G(0) << ",";
    gnd_file << quad.rpy_G(1) << ",";
    gnd_file << quad.rpy_G(2) << std::endl;

    const Vec3 a_B = quad.getBodyAcceleration();
    const Vec3 w_B = quad.getBodyAngularVelocity();
    mea_file << t << ",";
    mea_file << a_B(0) << ",";
    mea_file << a_B(1) << ",";
    mea_file << a_B(2) << ",";
    mea_file << w_B(0) << ",";
    mea_file << w_B(1) << ",";
    mea_file << w_B(2) << std::endl;
  }
  gnd_file.close();

  // Plot quadrotor trajectory
  PYTHON_SCRIPT("scripts/plot_quadrotor.py "
                "/tmp/quadrotor_gnd.dat "
                "/tmp/quadrotor_mea.dat");

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_QuadrotorModel_constructor);
  MU_ADD_TEST(test_QuadrotorModel_update);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
