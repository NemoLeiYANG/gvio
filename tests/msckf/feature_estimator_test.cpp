#include "gvio/munit.h"
#include "gvio/msckf/feature_estimator.hpp"
#include "gvio/camera/pinhole_model.hpp"

namespace gvio {

struct test_config {
  const int image_width = 640;
  const int image_height = 640;
  const double fov = 60.0;

  const double fx = PinholeModel::focalLengthX(image_width, fov);
  const double fy = PinholeModel::focalLengthY(image_height, fov);
  const double cx = image_width / 2.0;
  const double cy = image_height / 2.0;

  const Vec3 landmark{0.0, 0.0, 10.0};
};

void setup_test(const struct test_config &config,
                PinholeModel &cam_model,
                CameraStates &track_cam_states,
                FeatureTrack &track) {
  // Camera model
  cam_model = PinholeModel{config.image_width,
                           config.image_height,
                           config.fx,
                           config.fy,
                           config.cx,
                           config.cy};

  // -- Camera state 0
  const Vec3 p_G_C0{0.0, 0.0, 0.0};
  const Vec3 rpy_C0G{deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  const Vec4 q_C0G = euler2quat(rpy_C0G);
  const Mat3 C_C0G = C(q_C0G);
  const CameraState cam_state0{p_G_C0, q_C0G};
  // -- Camera state 1
  const Vec3 p_G_C1{1.0, 1.0, 0.0};
  const Vec3 rpy_C1G{deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  const Vec4 q_C1G = euler2quat(rpy_C1G);
  const Mat3 C_C1G = C(q_C1G);
  const CameraState cam_state1{p_G_C1, q_C1G};
  // -- Add to track camera states
  track_cam_states.push_back(cam_state0);
  track_cam_states.push_back(cam_state1);

  // Feature track
  const Vec3 landmark{config.landmark};
  const Vec2 kp1 = cam_model.project(landmark, C_C0G, p_G_C0).block(0, 0, 2, 1);
  const Vec2 kp2 = cam_model.project(landmark, C_C1G, p_G_C1).block(0, 0, 2, 1);
  // -- Add to feature track
  track = FeatureTrack{0, 1, Feature{kp1}, Feature{kp2}};
}

int test_FeatureEstimator_triangulate() {
  // Camera model
  struct test_config config;
  PinholeModel cam_model(config.image_width,
                         config.image_height,
                         config.fx,
                         config.fy,
                         config.cx,
                         config.cy);

  // Camera states
  // -- Camera state 0
  const Vec3 p_G_C0{0.0, 0.0, 0.0};
  const Vec3 rpy_C0G{deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  const Vec4 q_C0G = euler2quat(rpy_C0G);
  const Mat3 C_C0G = C(q_C0G);
  // -- Camera state 1
  const Vec3 p_G_C1{1.0, 1.0, 0.0};
  const Vec3 rpy_C1G{deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  const Vec4 q_C1G = euler2quat(rpy_C1G);
  const Mat3 C_C1G = C(q_C1G);

  // Features
  const Vec3 landmark{0.0, 0.0, 10.0};
  const Vec2 kp1 = cam_model.project(landmark, C_C0G, p_G_C0).block(0, 0, 2, 1);
  const Vec2 kp2 = cam_model.project(landmark, C_C1G, p_G_C1).block(0, 0, 2, 1);

  // Calculate rotation and translation of first and last camera states
  // -- Obtain rotation and translation from camera 0 to camera 1
  const Mat3 C_C0C1 = C_C0G * C_C1G.transpose();
  const Vec3 t_C0_C1C0 = C_C0G * (p_G_C1 - p_G_C0);
  // -- Convert from pixel coordinates to image coordinates
  const Vec2 pt1 = cam_model.pixel2image(kp1);
  const Vec2 pt2 = cam_model.pixel2image(kp2);

  // Triangulate
  Vec3 p_C0_f;
  const int retval =
      FeatureEstimator::triangulate(pt1, pt2, C_C0C1, t_C0_C1C0, p_C0_f);

  // Assert
  MU_CHECK(landmark.isApprox(p_C0_f));
  MU_CHECK_EQ(0, retval);

  return 0;
}

int test_FeatureEstimator_initialEstimate() {
  // Setup test
  const struct test_config config;
  PinholeModel cam_model;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_test(config, cam_model, track_cam_states, track);

  // Initial estimate
  Vec3 p_C0_f;
  FeatureEstimator estimator(&cam_model, track, track_cam_states);
  int retval = estimator.initialEstimate(p_C0_f);

  // Assert
  MU_CHECK(((config.landmark - p_C0_f).norm() < 1e-6));
  MU_CHECK_EQ(0, retval);

  return 0;
}

int test_FeatureEstimator_jacobian() {
  // Setup test
  const struct test_config config;
  PinholeModel cam_model;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_test(config, cam_model, track_cam_states, track);

  // Test jacobian
  Vec3 p_C0_f;
  Vec3 x{0.0, 0.0, 0.1};
  FeatureEstimator estimator(&cam_model, track, track_cam_states);
  estimator.jacobian(x);

  return 0;
}

int test_FeatureEstimator_reprojectionError() {
  // Setup test
  const struct test_config config;
  PinholeModel cam_model;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_test(config, cam_model, track_cam_states, track);

  // Test jacobian
  Vec3 p_C0_f;
  Vec3 x{0.0, 0.0, 0.1};
  FeatureEstimator estimator(&cam_model, track, track_cam_states);
  estimator.reprojectionError(x);

  return 0;
}

int test_FeatureEstimator_estimate() {
  // Setup test
  const struct test_config config;
  PinholeModel cam_model;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_test(config, cam_model, track_cam_states, track);

  // Test jacobian
  Vec3 p_G_f;
  FeatureEstimator estimator(&cam_model, track, track_cam_states);
  estimator.debug_mode = true;

  struct timespec start;
  tic(&start);
  int retval = estimator.estimate(p_G_f);
  printf("elasped: %fs\n", toc(&start));

  MU_CHECK_EQ(0, retval);
  MU_CHECK(((config.landmark - p_G_f).norm() < 1e-6));

  return 0;
}

int test_CeresReprojectionError_constructor() {
  // Setup test
  const struct test_config config;
  PinholeModel cam_model;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_test(config, cam_model, track_cam_states, track);

  // Ceres reprojection error
  CeresReprojectionError error{cam_model.K, track.track[1].kp};

  return 0;
}

int test_CeresReprojectionError_formK() {
  // Setup test
  const struct test_config config;
  PinholeModel cam_model;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_test(config, cam_model, track_cam_states, track);

  // Ceres reprojection error
  CeresReprojectionError error{cam_model.K, track.track[1].kp};
  MU_CHECK(error.formK<double>().isApprox(cam_model.K));

  return 0;
}

int test_CeresReprojectionError_quatToRot() {
  // Setup test
  const struct test_config config;
  PinholeModel cam_model;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_test(config, cam_model, track_cam_states, track);

  // Ceres reprojection error
  CeresReprojectionError error{cam_model.K, track.track[1].kp};

  const Vec4 q = euler2quat(Vec3{0.0, 0.0, 0.0});
  const Mat3 R = error.quatToRot<double>(q);

  MU_CHECK(R.isApprox(quat2rot(q)));

  return 0;
}

int test_CeresReprojectionError_error() {
  // Setup test
  const struct test_config config;
  PinholeModel cam_model;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_test(config, cam_model, track_cam_states, track);

  // Calculate reprojection error for first measurement
  double r1[2] = {1.0, 1.0};
  CeresReprojectionError error1{cam_model.K, track.track[0].kp};
  error1(track_cam_states[0].q_CG.data(),
         track_cam_states[0].p_G.data(),
         config.landmark.data(),
         r1);
  MU_CHECK_FLOAT(0.0, r1[0]);
  MU_CHECK_FLOAT(0.0, r1[1]);

  // Calculate reprojection error for second measurement
  double r2[2] = {1.0, 1.0};
  CeresReprojectionError error2{cam_model.K, track.track[1].kp};
  error2(track_cam_states[1].q_CG.data(),
         track_cam_states[1].p_G.data(),
         config.landmark.data(),
         r2);
  MU_CHECK_NEAR(0.0, r2[0], 1e-5);
  MU_CHECK_NEAR(0.0, r2[1], 1e-5);

  return 0;
}

void test_suite() {
  // FeatureEstimator
  MU_ADD_TEST(test_FeatureEstimator_triangulate);
  MU_ADD_TEST(test_FeatureEstimator_initialEstimate);
  MU_ADD_TEST(test_FeatureEstimator_jacobian);
  MU_ADD_TEST(test_FeatureEstimator_reprojectionError);
  MU_ADD_TEST(test_FeatureEstimator_estimate);

  // CeresReprojectionError
  MU_ADD_TEST(test_CeresReprojectionError_constructor);
  MU_ADD_TEST(test_CeresReprojectionError_formK);
  MU_ADD_TEST(test_CeresReprojectionError_quatToRot);
  MU_ADD_TEST(test_CeresReprojectionError_error);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
