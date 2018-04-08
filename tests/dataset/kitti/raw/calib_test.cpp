#include "gvio/munit.hpp"
#include "gvio/dataset/kitti/kitti.hpp"
#include "gvio/quaternion/jpl.hpp"

namespace gvio {

#define TEST_DATA "test_data/kitti/raw/2011_09_26"

int test_CalibCamToCam_load() {
  CalibCamToCam calib;

  int retval = calib.load(TEST_DATA "/calib_cam_to_cam.txt");
  MU_CHECK_EQ(retval, 0);

  const Vec2 S_00_exp{1.392000e+03, 5.120000e+02};
  MU_CHECK_EQ("09-Jan-2012 13:57:47", calib.calib_time);
  MU_CHECK_FLOAT(9.950000e-02, calib.corner_dist);

  return 0;
}

int test_CalibIMUToVelo_load() {
  CalibIMUToVelo calib;

  int retval = calib.load(TEST_DATA "/calib_imu_to_velo.txt");
  MU_CHECK_EQ(retval, 0);

  std::cout << calib.R << std::endl;

  // const Vec2 S_00_exp{1.392000e+03, 5.120000e+02};
  // MU_CHECK_EQ("09-Jan-2012 13:57:47", calib.calib_time);
  // MU_CHECK_FLOAT(9.950000e-02, calib.corner_dist);

  return 0;
}

int test_CalibVeloToCam_load() {
  CalibIMUToVelo calib;

  int retval = calib.load(TEST_DATA "/calib_velo_to_cam.txt");
  MU_CHECK_EQ(retval, 0);

  return 0;
}

int test_sandbox() {
  CalibIMUToVelo imu2velo;

  CalibVeloToCam velo2cam;

  imu2velo.load(TEST_DATA "/calib_imu_to_velo.txt");
  velo2cam.load(TEST_DATA "/calib_velo_to_cam.txt");

  // Create transform matrices
  const Mat4 T_cam_imu = velo2cam.T_cam_velo * imu2velo.T_velo_imu;
  const Mat4 T_imu_cam = T_cam_imu.inverse();
  const Vec3 X{0.0, 10.0, 0.0};
  // std::cout << T_cam_imu * X.homogeneous() << std::endl;

  const Vec4 q = rot2quat(T_imu_cam.block(0, 0, 3, 3).transpose());
  std::cout << q.transpose() << std::endl;
  // std::cout << C(q) * X << std::endl;

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_CalibCamToCam_load);
  MU_ADD_TEST(test_CalibIMUToVelo_load);
  MU_ADD_TEST(test_CalibVeloToCam_load);
  MU_ADD_TEST(test_sandbox);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
