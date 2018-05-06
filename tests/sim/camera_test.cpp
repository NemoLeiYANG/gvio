#include "gvio/munit.hpp"
#include "gvio/camera/pinhole_model.hpp"
#include "gvio/sim/camera.hpp"

namespace gvio {

struct test_config {
  const int image_width = 640;
  const int image_height = 640;
  const double fov = 60.0;

  const double fx = pinhole_focal_length(image_width, fov);
  const double fy = pinhole_focal_length(image_height, fov);
  const double cx = image_width / 2.0;
  const double cy = image_height / 2.0;
};

int test_VirtualCamera_constructor() {
  VirtualCamera camera;
  return 0;
}

int test_VirtualCamera_observedFeatures() {
  struct test_config config;
  VirtualCamera cam_model(config.image_width,
                          config.image_height,
                          config.fx,
                          config.fy,
                          config.cx,
                          config.cy);

  MatX features;
  features.resize(1, 3);
  features << 10.0, 0.0, 0.0;

  // Test no change in orientation
  Vec3 rpy{deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  Vec3 t{0.0, 0.0, 0.0};
  MatX observed;
  std::vector<int> feature_ids;
  observed = cam_model.observedFeatures(features, rpy, t, feature_ids);

  MU_CHECK_EQ(1, feature_ids.size());
  MU_CHECK_EQ(0, feature_ids[0]);
  MU_CHECK_EQ(1, observed.rows());
  MU_CHECK_EQ(2, observed.cols());
  MU_CHECK_FLOAT(320.0, observed(0, 0));
  MU_CHECK_FLOAT(320.0, observed(0, 1));

  // Test change in roll
  features.row(0) = Vec3{10.0, 1.0, 0.0};
  rpy = Vec3{deg2rad(10.0), deg2rad(0.0), deg2rad(0.0)};
  t = Vec3{0.0, 0.0, 0.0};
  feature_ids.clear();
  observed = cam_model.observedFeatures(features, rpy, t, feature_ids);

  MU_CHECK_EQ(1, feature_ids.size());
  MU_CHECK_EQ(0, feature_ids[0]);
  MU_CHECK_EQ(1, observed.rows());
  MU_CHECK_EQ(2, observed.cols());
  MU_CHECK(320.0 > observed(0, 0));
  MU_CHECK(320.0 < observed(0, 1));

  // Test change in pitch
  features.row(0) = Vec3{10.0, 0.0, 0.0};
  rpy = Vec3{deg2rad(0.0), deg2rad(10.0), deg2rad(0.0)};
  t = Vec3{0.0, 0.0, 0.0};
  feature_ids.clear();
  observed = cam_model.observedFeatures(features, rpy, t, feature_ids);

  MU_CHECK_EQ(1, feature_ids.size());
  MU_CHECK_EQ(0, feature_ids[0]);
  MU_CHECK_EQ(1, observed.rows());
  MU_CHECK_EQ(2, observed.cols());
  MU_CHECK_FLOAT(320.0, observed(0, 0));
  MU_CHECK(320.0 > observed(0, 1));

  // Test change in yaw
  features.row(0) = Vec3{10.0, 0.0, 0.0};
  rpy = Vec3{deg2rad(0.0), deg2rad(0.0), deg2rad(10.0)};
  t = Vec3{0.0, 0.0, 0.0};
  feature_ids.clear();
  observed = cam_model.observedFeatures(features, rpy, t, feature_ids);

  MU_CHECK_EQ(1, feature_ids.size());
  MU_CHECK_EQ(0, feature_ids[0]);
  MU_CHECK_EQ(1, observed.rows());
  MU_CHECK_EQ(2, observed.cols());
  MU_CHECK(320.0 < observed(0, 0));
  MU_CHECK_FLOAT(320.0, observed(0, 1));

  // Test change in translation x in global frame
  features.row(0) = Vec3{10.0, 0.0, 0.0};
  rpy = Vec3{deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  t = Vec3{1.0, 0.0, 0.0};
  feature_ids.clear();
  observed = cam_model.observedFeatures(features, rpy, t, feature_ids);

  MU_CHECK_EQ(1, feature_ids.size());
  MU_CHECK_EQ(0, feature_ids[0]);
  MU_CHECK_EQ(1, observed.rows());
  MU_CHECK_EQ(2, observed.cols());
  MU_CHECK_FLOAT(320.0, observed(0, 0));
  MU_CHECK_FLOAT(320.0, observed(0, 1));

  // Test change in translation y in global frame
  features.row(0) = Vec3{10.0, 0.0, 0.0};
  rpy = Vec3{deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  t = Vec3{0.0, 1.0, 0.0};
  feature_ids.clear();
  observed = cam_model.observedFeatures(features, rpy, t, feature_ids);

  MU_CHECK_EQ(1, feature_ids.size());
  MU_CHECK_EQ(0, feature_ids[0]);
  MU_CHECK_EQ(1, observed.rows());
  MU_CHECK_EQ(2, observed.cols());
  MU_CHECK(320.0 < observed(0, 0));
  MU_CHECK_FLOAT(320.0, observed(0, 1));

  // Test change in translation z in global frame
  features.row(0) = Vec3{10.0, 0.0, 0.0};
  rpy = Vec3{deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  t = Vec3{0.0, 0.0, 1.0};
  feature_ids.clear();
  observed = cam_model.observedFeatures(features, rpy, t, feature_ids);

  MU_CHECK_EQ(1, feature_ids.size());
  MU_CHECK_EQ(0, feature_ids[0]);
  MU_CHECK_EQ(1, observed.rows());
  MU_CHECK_EQ(2, observed.cols());
  MU_CHECK_FLOAT(320.0, observed(0, 0));
  MU_CHECK(320.0 < observed(0, 1));

  // Test point is behind camera
  features.row(0) = Vec3{-10.0, 0.0, 0.0};
  rpy = Vec3{deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  t = Vec3{0.0, 0.0, 0.0};
  feature_ids.clear();
  observed = cam_model.observedFeatures(features, rpy, t, feature_ids);

  MU_CHECK_EQ(0, feature_ids.size());

  return 0;
}

int test_VirtualStereoCamera_observedFeatures_sandbox() {
  // Convert from Global frame NWU to Camera frame EDN
  // NWU: (x - forward, y - left, z - up)
  // EDN: (x - right, y - down, z - forward)
  const Vec3 rpy_G{deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  const Vec3 t_G{1.0, 0.0, 0.0};
  const Vec3 rpy_C0{-rpy_G(1), -rpy_G(2), rpy_G(0)};
  const Mat3 R_C0G = euler123ToRot(rpy_C0);
  const Vec3 t_C0 = rotx(-M_PI / 2.0) * rotz(-M_PI / 2.0) * t_G;

  // Do the same for the second camera
  const Vec3 rpy_C1{deg2rad(10), deg2rad(0.0), deg2rad(0.0)};
  const Mat3 R_C1C0 = euler123ToRot(rpy_C1);
  const Vec3 t_C0C1{-0.5, 0.0, 0.0};
  const Mat4 T_cam1_cam0 = transformation_matrix(R_C1C0, t_C0C1);
  const Mat4 T_cam0_cam1 = T_cam1_cam0.inverse();

  const Mat3 R_C1G = T_cam0_cam1.block(0, 0, 3, 3) * R_C0G;
  const Vec3 t_C1 = (T_cam0_cam1 * t_C0.homogeneous()).head<3>();

  // Project point
  struct test_config config;
  PinholeModel camera_model(config.image_width,
                            config.image_height,
                            config.fx,
                            config.fy,
                            config.cx,
                            config.cy);

  Vec3 p{0.0, 0.0, 10.0}; // Point in camera frame
  Vec3 p0 = camera_model.project(homogeneous(p), R_C0G, t_C0);
  Vec3 p1 = camera_model.project(homogeneous(p), R_C1G, t_C1);

  // Normalize
  p0(0) = p0(0) / p0(2);
  p0(1) = p0(1) / p0(2);
  p1(0) = p1(0) / p1(2);
  p1(1) = p1(1) / p1(2);

  // Assert
  // std::cout << p0.head<2>().transpose() << std::endl;
  // std::cout << p1.head<2>().transpose() << std::endl;
  MU_CHECK(p0(0) > p1(0));
  MU_CHECK(p0(1) > p1(1));

  return 0;
}

int test_VirtualStereoCamera_observedFeatures() {
  struct test_config config;
  const Mat3 R_C1C0 = I(3);
  const Vec3 t_C1C0{0.5, 0.0, 0.0};
  const Mat4 T_cam1_cam0 = transformation_matrix(R_C1C0, t_C1C0);
  VirtualStereoCamera cam_model(config.image_width,
                                config.image_height,
                                config.fx,
                                config.fy,
                                config.cx,
                                config.cy,
                                T_cam1_cam0);

  MatX features;
  features.resize(1, 3);
  features << 10.0, 0.0, 0.0;

  // Test no change in orientation
  Vec3 rpy{deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  Vec3 t{0.0, 0.0, 0.0};
  MatX observed;
  std::vector<int> feature_ids;
  observed = cam_model.observedFeatures(features, rpy, t, feature_ids);

  MU_CHECK_EQ(1, feature_ids.size());
  MU_CHECK_EQ(0, feature_ids[0]);
  MU_CHECK_EQ(2, observed.rows());
  MU_CHECK_EQ(2, observed.cols());
  MU_CHECK_FLOAT(320.0, observed(0, 0));
  MU_CHECK_FLOAT(320.0, observed(0, 1));

  // Test change in roll
  features.row(0) = Vec3{10.0, 1.0, 0.0};
  rpy = Vec3{deg2rad(10.0), deg2rad(0.0), deg2rad(0.0)};
  t = Vec3{0.0, 0.0, 0.0};
  feature_ids.clear();
  observed = cam_model.observedFeatures(features, rpy, t, feature_ids);

  MU_CHECK_EQ(1, feature_ids.size());
  MU_CHECK_EQ(0, feature_ids[0]);
  MU_CHECK_EQ(2, observed.rows());
  MU_CHECK_EQ(2, observed.cols());
  MU_CHECK(320.0 > observed(0, 0));
  MU_CHECK(320.0 < observed(0, 1));

  // Test change in pitch
  features.row(0) = Vec3{10.0, 0.0, 0.0};
  rpy = Vec3{deg2rad(0.0), deg2rad(10.0), deg2rad(0.0)};
  t = Vec3{0.0, 0.0, 0.0};
  feature_ids.clear();
  observed = cam_model.observedFeatures(features, rpy, t, feature_ids);

  MU_CHECK_EQ(1, feature_ids.size());
  MU_CHECK_EQ(0, feature_ids[0]);
  MU_CHECK_EQ(2, observed.rows());
  MU_CHECK_EQ(2, observed.cols());
  MU_CHECK_FLOAT(320.0, observed(0, 0));
  MU_CHECK(320.0 > observed(0, 1));

  // Test change in yaw
  features.row(0) = Vec3{10.0, 0.0, 0.0};
  rpy = Vec3{deg2rad(0.0), deg2rad(0.0), deg2rad(10.0)};
  t = Vec3{0.0, 0.0, 0.0};
  feature_ids.clear();
  observed = cam_model.observedFeatures(features, rpy, t, feature_ids);

  MU_CHECK_EQ(1, feature_ids.size());
  MU_CHECK_EQ(0, feature_ids[0]);
  MU_CHECK_EQ(2, observed.rows());
  MU_CHECK_EQ(2, observed.cols());
  MU_CHECK(320.0 < observed(0, 0));
  MU_CHECK_FLOAT(320.0, observed(0, 1));

  // Test change in translation x in global frame
  features.row(0) = Vec3{10.0, 0.0, 0.0};
  rpy = Vec3{deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  t = Vec3{1.0, 0.0, 0.0};
  feature_ids.clear();
  observed = cam_model.observedFeatures(features, rpy, t, feature_ids);

  MU_CHECK_EQ(1, feature_ids.size());
  MU_CHECK_EQ(0, feature_ids[0]);
  MU_CHECK_EQ(2, observed.rows());
  MU_CHECK_EQ(2, observed.cols());
  MU_CHECK_FLOAT(320.0, observed(0, 0));
  MU_CHECK_FLOAT(320.0, observed(0, 1));

  // Test change in translation y in global frame
  features.row(0) = Vec3{10.0, 0.0, 0.0};
  rpy = Vec3{deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  t = Vec3{0.0, 1.0, 0.0};
  feature_ids.clear();
  observed = cam_model.observedFeatures(features, rpy, t, feature_ids);

  MU_CHECK_EQ(1, feature_ids.size());
  MU_CHECK_EQ(0, feature_ids[0]);
  MU_CHECK_EQ(2, observed.rows());
  MU_CHECK_EQ(2, observed.cols());
  MU_CHECK(320.0 < observed(0, 0));
  MU_CHECK_FLOAT(320.0, observed(0, 1));

  // Test change in translation z in global frame
  features.row(0) = Vec3{10.0, 0.0, 0.0};
  rpy = Vec3{deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  t = Vec3{0.0, 0.0, 1.0};
  feature_ids.clear();
  observed = cam_model.observedFeatures(features, rpy, t, feature_ids);

  MU_CHECK_EQ(1, feature_ids.size());
  MU_CHECK_EQ(0, feature_ids[0]);
  MU_CHECK_EQ(2, observed.rows());
  MU_CHECK_EQ(2, observed.cols());
  MU_CHECK_FLOAT(320.0, observed(0, 0));
  MU_CHECK(320.0 < observed(0, 1));

  // Test point is behind camera
  features.row(0) = Vec3{-10.0, 0.0, 0.0};
  rpy = Vec3{deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  t = Vec3{0.0, 0.0, 0.0};
  feature_ids.clear();
  observed = cam_model.observedFeatures(features, rpy, t, feature_ids);

  MU_CHECK_EQ(0, feature_ids.size());

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_VirtualCamera_constructor);
  MU_ADD_TEST(test_VirtualCamera_observedFeatures);
  MU_ADD_TEST(test_VirtualStereoCamera_observedFeatures_sandbox);
  MU_ADD_TEST(test_VirtualStereoCamera_observedFeatures);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
