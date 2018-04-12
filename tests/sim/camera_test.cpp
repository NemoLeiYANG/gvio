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

void test_suite() {
  MU_ADD_TEST(test_VirtualCamera_constructor);
  MU_ADD_TEST(test_VirtualCamera_observedFeatures);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
