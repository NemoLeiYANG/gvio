#include "gvio/munit.hpp"
#include "gvio/sim/camera_motion.hpp"

namespace gvio {

int test_CameraMotion_constructor() {
  const std::vector<Vec3> pos_points;
  const std::vector<Vec3> att_points;
  const double max_time = 0.0;
  CameraMotion camera_motion(pos_points, att_points, max_time);

  MU_CHECK_EQ(0, camera_motion.pos_points.size());
  MU_CHECK_EQ(0, camera_motion.att_points.size());
  MU_CHECK_FLOAT(0.0, camera_motion.max_time);

  return 0;
}

int test_CameraMotion_update() {
  const std::vector<Vec3> pos_points{Vec3{0.0, 0.0, 0.0},
                                     Vec3{0.0, 1.0, 0.0},
                                     Vec3{4.0, 1.0, 0.0},
                                     Vec3{4.0, 0.0, 0.0}};
  const std::vector<Vec3> att_points{Vec3{0.0, 0.0, 0.0},
                                     Vec3{0.0, 0.0, 0.25},
                                     Vec3{0.0, 0.0, 0.75},
                                     Vec3{0.0, 0.0, 1.0}};
  // Setup camera motion
  const double dt = 0.1;
  const double max_time = 10.0;
  CameraMotion camera_motion(pos_points, att_points, max_time);

  // Simulate camera motion
  double time = 0.0;
  for (int i = 0; i < 100; i++) {
    camera_motion.update(time);
    time += dt;
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_CameraMotion_constructor);
  MU_ADD_TEST(test_CameraMotion_update);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
