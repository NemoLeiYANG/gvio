#include "gvio/munit.hpp"
#include "gvio/dataset/euroc/camera_data.hpp"

namespace gvio {

#define TEST_DATA "test_data/euroc/cam0"

int test_CameraData_constructor() {
  CameraData camera_data;

  MU_CHECK_EQ(0, camera_data.timestamps.size());
  MU_CHECK_EQ(0, camera_data.time.size());
  MU_CHECK_EQ(0, camera_data.image_paths.size());

  MU_CHECK_EQ("", camera_data.sensor_type);
  MU_CHECK_EQ("", camera_data.comment);
  MU_CHECK(camera_data.T_BS.isApprox(I(4)));
  MU_CHECK_FLOAT(0.0, camera_data.rate_hz);
  MU_CHECK(Vec2::Zero().isApprox(camera_data.resolution));
  MU_CHECK_EQ("", camera_data.camera_model);
  MU_CHECK(Vec4::Zero().isApprox(camera_data.intrinsics));
  MU_CHECK_EQ("", camera_data.distortion_model);
  MU_CHECK(Vec4::Zero().isApprox(camera_data.distortion_coefficients));

  return 0;
}

int test_CameraData_load() {
  CameraData camera_data;

  // Sensor extrinsics wrt. the body-frame.
  // clang-format off
  Mat4 T_BS_expected;
  T_BS_expected << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
                   0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
                   -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
                   0.0, 0.0, 0.0, 1.0;
  const Vec2 resolution_expected(752.0, 480.0);
  const Vec4 intrinsics_expected(458.654, 457.296, 367.215, 248.375);
  const Vec4 distortion_expected(-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
  // clang-format on

  int retval = camera_data.load(TEST_DATA);
  MU_CHECK_EQ(0.0, retval);
  MU_CHECK_EQ(10, camera_data.timestamps.size());
  MU_CHECK_EQ(10, camera_data.time.size());
  MU_CHECK_EQ(10, camera_data.image_paths.size());

  MU_CHECK_EQ("camera", camera_data.sensor_type);
  MU_CHECK_EQ("VI-Sensor cam0 (MT9M034)", camera_data.comment);
  MU_CHECK(T_BS_expected.isApprox(camera_data.T_BS));
  MU_CHECK_FLOAT(20.0, camera_data.rate_hz);
  MU_CHECK(resolution_expected.isApprox(camera_data.resolution));
  MU_CHECK_EQ("pinhole", camera_data.camera_model);
  MU_CHECK(intrinsics_expected.isApprox(camera_data.intrinsics));
  MU_CHECK_EQ("radial-tangential", camera_data.distortion_model);
  MU_CHECK(distortion_expected.isApprox(camera_data.distortion_coefficients));

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_CameraData_constructor);
  MU_ADD_TEST(test_CameraData_load);
}
}

MU_RUN_TESTS(gvio::test_suite);
