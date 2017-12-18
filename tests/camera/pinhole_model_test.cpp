#include "gvio/munit.h"
#include "gvio/camera/pinhole_model.hpp"

namespace gvio {

struct test_config {
  const int image_width = 640;
  const int image_height = 640;
  const double fov = 60.0;

  const double fx = PinHoleModel::focalLengthX(image_width, fov);
  const double fy = PinHoleModel::focalLengthY(image_height, fov);
  const double cx = image_width / 2.0;
  const double cy = image_height / 2.0;
};

PinHoleModel setup_pinhole_model() {
  struct test_config config;

  PinHoleModel cam_model(config.image_width,
                         config.image_height,
                         config.fx,
                         config.fy,
                         config.cx,
                         config.cy);
  return cam_model;
}

int test_PinHoleModel_constructor() {
  PinHoleModel cam_model;

  MU_CHECK_EQ(0, cam_model.image_width);
  MU_CHECK_EQ(0, cam_model.image_height);
  MU_CHECK_FLOAT(0.0, cam_model.fx);
  MU_CHECK_FLOAT(0.0, cam_model.fy);
  MU_CHECK_FLOAT(0.0, cam_model.cx);
  MU_CHECK_FLOAT(0.0, cam_model.cy);

  return 0;
}

int test_PinHoleModel_constructor2() {
  const int image_width = 600;
  const int image_height = 400;

  const double fx = 1.0;
  const double fy = 2.0;
  const double cx = 3.0;
  const double cy = 4.0;

  MatX K = Mat3::Zero();
  K(0, 0) = fx;
  K(1, 1) = fy;
  K(0, 2) = cx;
  K(1, 2) = cy;

  PinHoleModel cam_model(image_width, image_height, K);

  MU_CHECK_EQ(image_width, cam_model.image_width);
  MU_CHECK_EQ(image_height, cam_model.image_height);
  MU_CHECK_FLOAT(fx, cam_model.fx);
  MU_CHECK_FLOAT(fy, cam_model.fy);
  MU_CHECK_FLOAT(cx, cam_model.cx);
  MU_CHECK_FLOAT(cy, cam_model.cy);

  return 0;
}

int test_PinHoleModel_constructor3() {
  const int image_width = 600;
  const int image_height = 400;

  const double fx = 1.0;
  const double fy = 2.0;
  const double cx = 3.0;
  const double cy = 4.0;

  PinHoleModel cam_model(image_width, image_height, fx, fy, cx, cy);

  MU_CHECK_EQ(image_width, cam_model.image_width);
  MU_CHECK_EQ(image_height, cam_model.image_height);
  MU_CHECK_FLOAT(fx, cam_model.fx);
  MU_CHECK_FLOAT(fy, cam_model.fy);
  MU_CHECK_FLOAT(cx, cam_model.cx);
  MU_CHECK_FLOAT(cy, cam_model.cy);

  return 0;
}

int test_PinHoleModel_focalLength() {
  const double fx = PinHoleModel::focalLengthX(600, 90.0);
  const double fy = PinHoleModel::focalLengthY(600, 90.0);
  const Vec2 focal_length = PinHoleModel::focalLength(600, 600, 90.0);

  MU_CHECK_FLOAT(300.0, fy);
  MU_CHECK_FLOAT(fx, fy);
  MU_CHECK_FLOAT(fx, focal_length(0));
  MU_CHECK_FLOAT(fy, focal_length(1));

  return 0;
}

int test_PinHoleModel_P() {
  struct test_config config;
  PinHoleModel cam_model = setup_pinhole_model();
  Mat3 R = euler321ToRot(Vec3{0.0, 0.0, 0.0});
  Vec3 t{1.0, 2.0, 3.0};
  Mat34 P = cam_model.P(R, t);

  Mat34 P_expected;
  // clang-format off
  P_expected << config.fx, 0.0, config.cx, -1514.26,
                0.0, config.fy, config.cy, -2068.51,
                0.0, 0.0, 1.0, -3.0;
  // clang-format on

  // std::cout << P << std::endl;
  // std::cout << P_expected << std::endl;

  MU_CHECK(((P - P_expected).norm() < 0.01));

  return 0;
}

int test_PinHoleModel_project() {
  PinHoleModel cam_model = setup_pinhole_model();
  Mat3 R = euler321ToRot(Vec3{0.0, 0.0, 0.0});
  Vec3 t{0.0, 0.0, 0.0};
  Vec3 X{0.0, 0.0, 10.0};
  Vec3 x = cam_model.project(X, R, t);

  MU_CHECK_FLOAT(320.0, x(0));
  MU_CHECK_FLOAT(320.0, x(1));
  MU_CHECK_FLOAT(1.0, x(2));

  return 0;
}

int test_PinHoleModel_pixel2image() {
  PinHoleModel cam_model = setup_pinhole_model();
  Vec2 point = cam_model.pixel2image(Vec2{320, 320});

  MU_CHECK_FLOAT(0.0, point(0));
  MU_CHECK_FLOAT(0.0, point(1));

  return 0;
}

int test_PinHoleModel_observedFeatures() {
  struct test_config config;
  PinHoleModel cam_model = setup_pinhole_model();

  MatX features;
  features.resize(3, 1);
  features << 0.0, 0.0, 10.0;

  Vec3 rpy{0.0, 0.0, 0.0};
  Vec3 t{0.0, 0.0, 0.0};

  MatX observed;
  std::vector<int> mask;
  observed = cam_model.observedFeatures(features, rpy, t, mask);

  MU_CHECK_EQ(1, mask.size());
  MU_CHECK_EQ(0, mask[0]);
  MU_CHECK_EQ(2, observed.rows());
  MU_CHECK_EQ(1, observed.cols());
  MU_CHECK_EQ(320.0, observed(0, 0));
  MU_CHECK_EQ(320.0, observed(1, 0));

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_PinHoleModel_constructor);
  MU_ADD_TEST(test_PinHoleModel_constructor2);
  MU_ADD_TEST(test_PinHoleModel_constructor3);
  MU_ADD_TEST(test_PinHoleModel_focalLength);
  MU_ADD_TEST(test_PinHoleModel_P);
  MU_ADD_TEST(test_PinHoleModel_project);
  MU_ADD_TEST(test_PinHoleModel_pixel2image);
  MU_ADD_TEST(test_PinHoleModel_observedFeatures);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
