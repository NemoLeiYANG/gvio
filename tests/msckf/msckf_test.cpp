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
  MU_CHECK(Vec4(0.0, 0.0, 0.0, 1.0).isApprox(msckf.ext_q_CI));
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

int test_MSCKF_R() {
  MSCKF msckf;

  MatX R;
  msckf.R(1.0, 2.0, 4, R);

  Mat4 expected;
  // clang-format off
  expected << 1.0, 0.0, 0.0, 0.0,
              0.0, 2.0, 0.0, 0.0,
              0.0, 0.0, 1.0, 0.0,
              0.0, 0.0, 0.0, 2.0;
  // clang-format on

  MU_CHECK_EQ(4, R.rows());
  MU_CHECK_EQ(4, R.cols());
  MU_CHECK(expected.isApprox(R));

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
  if (blackbox.configure("/tmp", "test_msckf_predictionUpdate") != 0) {
    LOG_ERROR("Failed to configure MSCKF blackbox!");
  }

  // Setup MSCKF
  MSCKF msckf;
  msckf.initialize(euler2quat(raw_dataset.oxts.rpy[0]),
                   raw_dataset.oxts.v_G[0],
                   Vec3{0.0, 0.0, 0.0});

  // Record initial conditions
  blackbox.record(raw_dataset.oxts.timestamps[0],
                  msckf,
                  raw_dataset.oxts.a_B[0],
                  raw_dataset.oxts.w_B[0],
                  raw_dataset.oxts.p_G[0],
                  raw_dataset.oxts.v_G[0],
                  raw_dataset.oxts.rpy[0]);

  // Loop through data and do prediction update
  for (int i = 1; i < (int) raw_dataset.oxts.timestamps.size() - 1; i++) {
    const Vec3 a_B = raw_dataset.oxts.a_B[i];
    const Vec3 w_B = raw_dataset.oxts.w_B[i];
    const double t_prev = raw_dataset.oxts.timestamps[i - 1];
    const double t_now = raw_dataset.oxts.timestamps[i];
    const double dt = t_now - t_prev;

    msckf.predictionUpdate(a_B, w_B, dt);
    blackbox.record(raw_dataset.oxts.timestamps[i],
                    msckf,
                    a_B,
                    w_B,
                    raw_dataset.oxts.p_G[i],
                    raw_dataset.oxts.v_G[i],
                    raw_dataset.oxts.rpy[i]);

    // const std::string img_path = raw_dataset.cam0[i];
    // const cv::Mat image = cv::imread(img_path);
    // cv::imshow("Image", image);
    // cv::waitKey(0);
  }

  // system("python3 scripts/plot_msckf_blackbox.py");

  return 0;
}

int test_MSCKF_calTrackResiduals() {
  // Camera model
  const int image_width = 640;
  const int image_height = 640;
  const double fov = 60.0;
  const double fx = PinholeModel::focalLengthX(image_width, fov);
  const double fy = PinholeModel::focalLengthY(image_height, fov);
  const double cx = image_width / 2.0;
  const double cy = image_height / 2.0;
  PinholeModel pinhole_model{image_width, image_height, fx, fy, cx, cy};

  // Setup MSCKF
  MSCKF msckf;
  // -- Modify default settings for test
  msckf.min_track_length = 2;
  msckf.camera_model = &pinhole_model;
  // -- Add first camera state
  msckf.initialize();
  // -- Add second camera state
  msckf.imu_state.p_G = Vec3{0.0, 1.0, 0.0};
  msckf.augmentState();

  // Prepare features and feature track
  // -- Create 2 features
  const Vec3 p_G_f{0.0, 0.0, 10.0};
  const Vec3 pt0 = pinhole_model.project(p_G_f,
                                         C(msckf.cam_states[0].q_CG),
                                         msckf.cam_states[0].p_G);
  const Vec3 pt1 = pinhole_model.project(p_G_f,
                                         C(msckf.cam_states[1].q_CG),
                                         msckf.cam_states[1].p_G);
  Feature f0{Vec2{pt0(0), pt0(1)}};
  Feature f1{Vec2{pt1(0), pt1(1)}};
  // -- Create a feature track based on two features
  FeatureTrack track{0, 1, f0, f1};

  // Calculate track residual
  MatX H_j;
  VecX r_j;
  MatX R_j;
  int retval = msckf.calTrackResiduals(track, H_j, r_j, R_j);

  // Assert
  MU_CHECK_EQ(0, retval);
  for (int i = 0; i < r_j.rows(); i++) {
    MU_CHECK_NEAR(r_j(i), 0.0, 1e-5);
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_MSCKF_constructor);
  MU_ADD_TEST(test_MSCKF_P);
  MU_ADD_TEST(test_MSCKF_N);
  MU_ADD_TEST(test_MSCKF_H);
  MU_ADD_TEST(test_MSCKF_R);
  MU_ADD_TEST(test_MSCKF_augmentState);
  MU_ADD_TEST(test_MSCKF_getTrackCameraStates);
  MU_ADD_TEST(test_MSCKF_predictionUpdate);
  MU_ADD_TEST(test_MSCKF_calTrackResiduals);
  // TODO: MU_ADD_TEST(test_MSCKF_calResiduals);
  // TODO: MU_ADD_TEST(test_MSCKF_correctIMUState);
  // TODO: MU_ADD_TEST(test_MSCKF_correctCameraStates);
  // MU_ADD_TEST(test_MSCKF_measurementUpdate);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
