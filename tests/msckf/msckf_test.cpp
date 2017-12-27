#include "gvio/munit.h"
#include "gvio/msckf/msckf.hpp"

namespace gvio {

int test_MSCKF_constructor() {
  MSCKF msckf;

  MU_CHECK_EQ(CameraState::size, msckf.P_cam.rows());
  MU_CHECK_EQ(CameraState::size, msckf.P_cam.cols());
  MU_CHECK_EQ(IMUState::size, msckf.P_imu_cam.rows());
  MU_CHECK_EQ(CameraState::size, msckf.P_imu_cam.cols());

  MU_CHECK_EQ(0, msckf.counter_frame_id);
  MU_CHECK(zeros(3, 1).isApprox(msckf.ext_p_IC));
  MU_CHECK(zeros(4, 1).isApprox(msckf.ext_q_CI));
  MU_CHECK_FLOAT(0.0, msckf.n_u);
  MU_CHECK_FLOAT(0.0, msckf.n_v);

  MU_CHECK(msckf.enable_ns_trick);
  MU_CHECK(msckf.enable_qr_trick);

  return 0;
}

int test_MSCKF_P() {
  MSCKF msckf;
  const MatX P = msckf.P();

  MU_CHECK_EQ(15, P.rows());
  MU_CHECK_EQ(15, P.cols());

  return 0;
}

int test_MSCKF_N() {
  MSCKF msckf;
  MU_CHECK_EQ(0, msckf.N());
  return 0;
}

int test_MSCKF_H() {
  MSCKF msckf;
  return 0;
}

int test_MSCKF_augmentState() {
  MSCKF msckf;

  // Augment state 1
  msckf.augmentState();
  MU_CHECK_EQ(1, msckf.N());
  std::cout << msckf.P_cam.rows() << std::endl;
  MU_CHECK_EQ(6, msckf.P_cam.rows());
  MU_CHECK_EQ(6, msckf.P_cam.cols());
  MU_CHECK_EQ(15, msckf.P_imu_cam.rows());
  MU_CHECK_EQ(6, msckf.P_imu_cam.cols());

  // Augment state 2
  msckf.augmentState();
  MU_CHECK_EQ(2, msckf.N());
  MU_CHECK_EQ(12, msckf.P_cam.rows());
  MU_CHECK_EQ(12, msckf.P_cam.cols());
  MU_CHECK_EQ(15, msckf.P_imu_cam.rows());
  MU_CHECK_EQ(6, msckf.P_imu_cam.cols());

  // print_shape(msckf.P_imu_cam);

  // Augment state 3
  msckf.augmentState();
  MU_CHECK_EQ(3, msckf.N());
  MU_CHECK_EQ(12, msckf.P_cam.rows());
  MU_CHECK_EQ(12, msckf.P_cam.cols());
  MU_CHECK_EQ(15, msckf.P_imu_cam.rows());
  MU_CHECK_EQ(12, msckf.P_imu_cam.cols());

  // MU_CHECK_EQ(2, msckf.N());
  // MU_CHECK_EQ(12, msckf.P_cam.rows());
  // MU_CHECK_EQ(12, msckf.P_cam.cols());
  // MU_CHECK_EQ(15, msckf.P_imu_cam.rows());
  // MU_CHECK_EQ(12, msckf.P_imu_cam.cols());

  return 0;
}

int test_MSCKF_getTrackCameraStates() {
  MSCKF msckf;

  // CameraStates msckf.getCameraStates();
  return 0;
}

int test_MSCKF_predictionUpdate() {
  MSCKF msckf;

  // CameraStates msckf.getCameraStates();
  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_MSCKF_constructor);
  MU_ADD_TEST(test_MSCKF_P);
  MU_ADD_TEST(test_MSCKF_N);
  MU_ADD_TEST(test_MSCKF_H);
  MU_ADD_TEST(test_MSCKF_augmentState);
  MU_ADD_TEST(test_MSCKF_getTrackCameraStates);
  MU_ADD_TEST(test_MSCKF_predictionUpdate);
  // MU_ADD_TEST(test_MSCKF_measurementUpdate);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
