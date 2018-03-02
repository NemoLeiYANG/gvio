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

  // const double roll = 0.3;
  // const double pitch = 0.2;
  // const double yaw = 0.1;
  //
  // Mat3 R = euler321ToRot(Vec3{roll, pitch, yaw});
  // std::cout << R << std::endl;
  //
  // Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  // Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  // Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  // Eigen::Quaternion<double> q = yaw_angle * pitch_angle * roll_angle;
  // Eigen::Matrix3d R = q.matrix();
  //
  // std::cout << rotationMatrix << std::endl;

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_GimbalCalib_constructor);
  MU_ADD_TEST(test_GimbalCalib_load);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
