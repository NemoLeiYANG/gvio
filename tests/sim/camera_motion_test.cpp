#include "gvio/munit.hpp"
#include "gvio/sim/camera_motion.hpp"

namespace gvio {

int test_CameraMotion_constructor() {
  const std::vector<Vec3> pos_points;
  const std::vector<Vec3> att_points;
  const int max_steps = 100;
  CameraMotion camera_motion(pos_points, att_points, max_steps);

  MU_CHECK_EQ(0, camera_motion.pos_points.size());
  MU_CHECK_EQ(0, camera_motion.att_points.size());
  MU_CHECK_EQ(100, camera_motion.max_steps);

  return 0;
}

int test_CameraMotion_update() {
  const std::vector<Vec3> pos_points{Vec3{0.0, 0.0, 0.0},
                                     Vec3{0.0, 5.0, 0.0},
                                     Vec3{5.0, 5.0, 0.0},
                                     Vec3{5.0, 0.0, 0.0}};
  // const std::vector<Vec3> pos_points{Vec3{0.0, 0.0, 0.0},
  //                                    Vec3{1.0, 1.0, 0.0},
  //                                    Vec3{2.0, 2.0, 0.0},
  //                                    Vec3{3.0, 3.0, 0.0}};
  const std::vector<Vec3> att_points{Vec3{0.0, 0.0, 0.0},
                                     Vec3{0.0, 0.0, 0.25},
                                     Vec3{0.0, 0.0, 0.75},
                                     Vec3{0.0, 0.0, 1.0}};
  // Setup camera motion
  const int max_steps = 100;
  CameraMotion camera_motion(pos_points, att_points, max_steps);

  // Simulate camera motion
  Vec3 pos = camera_motion.p_G;
  Vec3 vel = camera_motion.v_G;
  Vec3 acc = camera_motion.a_G;
  Vec3 rpy = camera_motion.rpy_G;
  Vec3 w_m = camera_motion.w_G;

  std::vector<Vec3> position = {pos_points[0]};
  std::vector<Vec3> attitude = {att_points[0]};
  while (camera_motion.update() == 0) {
    acc = camera_motion.a_G;
    vel = vel + (acc * camera_motion.dt);
    pos = pos + (vel * camera_motion.dt);

    w_m = camera_motion.w_G;
    rpy = rpy + (w_m * camera_motion.dt);

    position.push_back(pos);
    attitude.push_back(rpy);
  }

  std::cout << "gnd pos: " << camera_motion.p_G.transpose() << std::endl;
  std::cout << "est pos: " << position.back().transpose() << std::endl;

  std::cout << "gnd rpy: " << camera_motion.rpy_G.transpose() << std::endl;
  std::cout << "est rpy: " << attitude.back().transpose() << std::endl;

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_CameraMotion_constructor);
  MU_ADD_TEST(test_CameraMotion_update);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
