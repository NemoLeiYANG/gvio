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

  // Get camera 0 rotation and translation
  const Mat3 C_C0G = C(track_cam_states[0].q_CG);
  const Vec3 p_G_C0 = track_cam_states[0].p_G;
  // Get camera i rotation and translation
  const int camera_index = 0;
  const Mat3 C_CiG = C(track_cam_states[camera_index].q_CG);
  const Vec3 p_G_Ci = track_cam_states[camera_index].p_G;
  // Set camera 0 as origin, work out rotation and translation
  // between camera i to to camera 0
  const Mat3 C_CiC0 = C_CiG * C_C0G.transpose();
  const Vec3 t_Ci_CiC0 = C_CiG * (p_G_C0 - p_G_Ci);

  // Ceres reprojection error
  CeresReprojectionError error{&cam_model,
                               C_CiC0,
                               t_Ci_CiC0,
                               track.track[camera_index].kp};

  return 0;
}

int test_CeresReprojectionError_evaluate() {
  // Setup test
  const struct test_config config;
  PinholeModel cam_model;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_test(config, cam_model, track_cam_states, track);

  // Get camera 0 rotation and translation
  const Mat3 C_C0G = C(track_cam_states[0].q_CG);
  const Vec3 p_G_C0 = track_cam_states[0].p_G;

  for (int i = 0; i < 2; i++) {
    // Get camera 0 rotation and translation
    const Mat3 C_CiG = C(track_cam_states[i].q_CG);
    const Vec3 p_G_Ci = track_cam_states[i].p_G;
    // Set camera 0 as origin, work out rotation and translation
    // between camera 0 to to camera 0
    const Mat3 C_CiC0 = C_CiG * C_C0G.transpose();
    const Vec3 t_Ci_CiC0 = C_CiG * (p_G_C0 - p_G_Ci);

    // Calculate reprojection error for first measurement
    double r[2] = {1.0, 1.0};
    CeresReprojectionError error{&cam_model,
                                 C_CiC0,
                                 t_Ci_CiC0,
                                 track.track[i].kp};

    // Create inverse depth params (these are to be optimized)
    const double alpha = config.landmark(0) / config.landmark(2);
    const double beta = config.landmark(1) / config.landmark(2);
    const double rho = 1.0 / config.landmark(2);
    double *x = (double *) malloc(sizeof(double) * 3);
    x[0] = alpha;
    x[1] = beta;
    x[2] = rho;

    // Test and assert
    error.Evaluate(&x, r, NULL);
    std::cout << "residuals: " << r[0] << ", " << r[1] << std::endl;

    MU_CHECK_NEAR(0.0, r[0], 1e-5);
    MU_CHECK_NEAR(0.0, r[1], 1e-5);
  }

  return 0;
}

int test_CeresFeatureEstimator_constructor() {
  // Setup test
  const struct test_config config;
  PinholeModel cam_model;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_test(config, cam_model, track_cam_states, track);

  // Setup CeresFeatureEstimator
  CeresFeatureEstimator estimator{&cam_model, track, track_cam_states};

  return 0;
}

int test_CeresFeatureEstimator_setupProblem() {
  // Setup test
  const struct test_config config;
  PinholeModel cam_model;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_test(config, cam_model, track_cam_states, track);

  // Setup CeresFeatureEstimator
  CeresFeatureEstimator estimator{&cam_model, track, track_cam_states};
  estimator.setupProblem();

  return 0;
}

int test_CeresFeatureEstimator_estimate() {
  // Setup test
  const struct test_config config;
  PinholeModel cam_model;
  CameraStates track_cam_states;
  FeatureTrack track;
  setup_test(config, cam_model, track_cam_states, track);

  // Setup CeresFeatureEstimator
  CeresFeatureEstimator estimator{&cam_model, track, track_cam_states};

  struct timespec start;
  tic(&start);
  Vec3 p_G_f;
  estimator.estimate(p_G_f);
  printf("elasped: %fs\n", toc(&start));

  // std::cout << p_G_f.transpose() << std::endl;

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
  MU_ADD_TEST(test_CeresReprojectionError_evaluate);

  // CeresFeatureEstimator
  MU_ADD_TEST(test_CeresFeatureEstimator_constructor);
  MU_ADD_TEST(test_CeresFeatureEstimator_setupProblem);
  MU_ADD_TEST(test_CeresFeatureEstimator_estimate);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
