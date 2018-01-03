#include "gvio/munit.h"
#include "gvio/msckf/msckf.hpp"
#include "gvio/msckf/blackbox.hpp"
#include "gvio/kitti/kitti.hpp"
#include "gvio/util/util.hpp"

namespace gvio {

static const std::string KITTI_RAW_DATASET = "/data/kitti/raw";

int test_MSCKF_constructor() {
  MSCKF msckf;

  // MU_CHECK_EQ(CameraState::size, msckf.P_cam.rows());
  // MU_CHECK_EQ(CameraState::size, msckf.P_cam.cols());
  // MU_CHECK_EQ(IMUState::size, msckf.P_imu_cam.rows());
  // MU_CHECK_EQ(CameraState::size, msckf.P_imu_cam.cols());

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
  MU_CHECK_EQ(6, msckf.P_cam.rows());
  MU_CHECK_EQ(6, msckf.P_cam.cols());
  MU_CHECK_EQ(15, msckf.P_imu_cam.rows());
  MU_CHECK_EQ(6, msckf.P_imu_cam.cols());
  MU_CHECK_EQ(1, msckf.N());
  MU_CHECK_EQ(1, msckf.counter_frame_id);

  // Augment state 2
  msckf.augmentState();
  MU_CHECK_EQ(12, msckf.P_cam.rows());
  MU_CHECK_EQ(12, msckf.P_cam.cols());
  MU_CHECK_EQ(15, msckf.P_imu_cam.rows());
  MU_CHECK_EQ(12, msckf.P_imu_cam.cols());
  MU_CHECK_EQ(2, msckf.N());
  MU_CHECK_EQ(2, msckf.counter_frame_id);

  // Augment state 3
  msckf.augmentState();
  MU_CHECK_EQ(18, msckf.P_cam.rows());
  MU_CHECK_EQ(18, msckf.P_cam.cols());
  MU_CHECK_EQ(15, msckf.P_imu_cam.rows());
  MU_CHECK_EQ(18, msckf.P_imu_cam.cols());
  MU_CHECK_EQ(3, msckf.N());
  MU_CHECK_EQ(3, msckf.counter_frame_id);

  return 0;
}

int test_MSCKF_getTrackCameraStates() {
  MSCKF msckf;
  msckf.augmentState();
  msckf.augmentState();

  Feature f1{Vec2{0.0, 0.0}};
  Feature f2{Vec2{1.0, 1.0}};
  FeatureTrack track{0, 1, f1, f2};

  CameraStates track_cam_states = msckf.getTrackCameraStates(track);

  MU_CHECK_EQ(2, msckf.cam_states.size());
  MU_CHECK_EQ(2, track_cam_states.size());
  MU_CHECK_EQ(0, track_cam_states[0].frame_id);
  MU_CHECK_EQ(1, track_cam_states[1].frame_id);

  return 0;
}

int test_MSCKF_predictionUpdate() {
  // Load raw dataset
  RawDataset raw_dataset(KITTI_RAW_DATASET, "2011_09_26", "0005");
  if (raw_dataset.load() != 0) {
    LOG_ERROR("Failed to load KITTI raw dataset [%s]!",
              KITTI_RAW_DATASET.c_str());
    return -1;
  }

  // Prep blackbox
  BlackBox blackbox;
  if (blackbox.configure("/tmp/test_msckf_predictionUpdate.dat") != 0) {
    LOG_ERROR("Failed to configure MSCKF blackbox!");
  }

  // Loop through data and do prediction update
  MSCKF msckf;

  for (int i = 1; i < 10; i++) {
    const Vec3 a_B = raw_dataset.oxts.a_B[i];
    const Vec3 w_B = raw_dataset.oxts.w_B[i];
    const double t_prev = raw_dataset.oxts.timestamps[i - 1];
    const double t_now = raw_dataset.oxts.timestamps[i];
    const double dt = t_now - t_prev;

    msckf.predictionUpdate(a_B, w_B, dt);

    // const std::string img_path = raw_dataset.cam0[i];
    // const cv::Mat image = cv::imread(img_path);
    // cv::imshow("Image", image);
    // cv::waitKey(0);
  }

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
  // TODO: MU_ADDTEST(test_MSCKF_calTrackResiduals);
  // TODO: MU_ADDTEST(test_MSCKF_calResiduals);
  // TODO: MU_ADDTEST(test_MSCKF_correctIMUState);
  // TODO: MU_ADDTEST(test_MSCKF_correctCameraStates);
  // MU_ADD_TEST(test_MSCKF_measurementUpdate);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
