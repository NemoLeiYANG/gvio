#include "gvio/munit.hpp"
#include "gvio/sim/world.hpp"

namespace gvio {

int test_SimWorld_constructor() {
  SimWorld world;

  MU_CHECK_FLOAT(0.0, world.t);
  MU_CHECK_FLOAT(0.0, world.dt);
  MU_CHECK_FLOAT(0.0, world.t_end);

  return 0;
}

int test_SimWorld_configure() {
  SimWorld world;

  world.configure(0.1);
  MU_CHECK_FLOAT(0.0, world.t);
  MU_CHECK_FLOAT(0.1, world.dt);

  return 0;
}

int test_SimWorld_create3DFeatures() {
  struct feature_bounds bounds;
  bounds.x_min = 1.0;
  bounds.x_max = 2.0;
  bounds.y_min = 3.0;
  bounds.y_max = 4.0;
  bounds.z_min = 5.0;
  bounds.z_max = 6.0;

  SimWorld world;
  const MatX features = world.create3DFeatures(bounds, 1000);

  for (int i = 0; i < features.rows(); i++) {
    const Vec3 feature = features.block(i, 0, 1, 3).transpose();
    MU_CHECK(feature(0) >= bounds.x_min);
    MU_CHECK(feature(0) <= bounds.x_max);
    MU_CHECK(feature(1) >= bounds.y_min);
    MU_CHECK(feature(1) <= bounds.y_max);
    MU_CHECK(feature(2) >= bounds.z_min);
    MU_CHECK(feature(2) <= bounds.z_max);
  }

  return 0;
}

int test_SimWorld_create3DFeaturePerimeter() {
  SimWorld world;
  Vec3 origin{0.0, 0.0, 0.0};
  Vec3 dimensions{10.0, 10.0, 10.0};
  const MatX features =
      world.create3DFeaturePerimeter(origin, dimensions, 1000);

  for (int i = 0; i < features.rows(); i++) {
    const Vec3 feature = features.block(i, 0, 1, 3).transpose();
    MU_CHECK(feature(0) >= origin(0) - dimensions(0) / 2.0);
    MU_CHECK(feature(0) <= origin(0) + dimensions(0) / 2.0);
    MU_CHECK(feature(1) >= origin(1) - dimensions(1) / 2.0);
    MU_CHECK(feature(1) <= origin(1) + dimensions(1) / 2.0);
    MU_CHECK(feature(2) >= origin(2) - dimensions(2) / 2.0);
    MU_CHECK(feature(2) <= origin(2) + dimensions(2) / 2.0);
  }

  mat2csv("/tmp/features.dat", features);
  PYTHON_SCRIPT("scripts/plot_world.py /tmp/features.dat");

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_SimWorld_constructor);
  MU_ADD_TEST(test_SimWorld_constructor);
  MU_ADD_TEST(test_SimWorld_configure);
  MU_ADD_TEST(test_SimWorld_create3DFeatures);
  MU_ADD_TEST(test_SimWorld_create3DFeaturePerimeter);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
