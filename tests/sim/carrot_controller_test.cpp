#include "gvio/munit.h"
#include "gvio/sim/carrot_controller.hpp"

namespace gvio {

int test_CarrotController_constructor() {
  CarrotController controller;

  MU_CHECK(controller.wp_start.isApprox(Vec3::Zero()));
  MU_CHECK(controller.wp_end.isApprox(Vec3::Zero()));
  MU_CHECK_EQ(0, controller.wp_index);
  MU_CHECK_FLOAT(0.0, controller.look_ahead_dist);

  return 0;
}

int test_CarrotController_configure() {
  CarrotController controller;

  std::vector<Vec3> waypoints;
  waypoints.emplace_back(0.0, 0.0, 0.0);
  waypoints.emplace_back(1.0, 1.0, 0.0);
  waypoints.emplace_back(2.0, 2.0, 0.0);
  waypoints.emplace_back(3.0, 3.0, 0.0);
  controller.configure(waypoints, 0.1);

  MU_CHECK(controller.wp_start.isApprox(Vec3::Zero()));
  MU_CHECK(controller.wp_end.isApprox(Vec3{1.0, 1.0, 0.0}));
  MU_CHECK_EQ(1, controller.wp_index);
  MU_CHECK_FLOAT(0.1, controller.look_ahead_dist);

  return 0;
}

int test_CarrotController_closestPoint() {
  CarrotController controller;

  std::vector<Vec3> wps;
  wps.emplace_back(0.0, 0.0, 0.0);
  wps.emplace_back(1.0, 1.0, 0.0);
  wps.emplace_back(2.0, 2.0, 0.0);
  wps.emplace_back(3.0, 3.0, 0.0);
  controller.configure(wps, 0.1);

  MU_CHECK(controller.wp_start.isApprox(Vec3::Zero()));
  MU_CHECK(controller.wp_end.isApprox(Vec3{1.0, 1.0, 0.0}));
  MU_CHECK_EQ(1, controller.wp_index);
  MU_CHECK_FLOAT(0.1, controller.look_ahead_dist);

  // Test before waypoint start
  Vec3 pos0{-1.0, -1.0, 0.0};
  Vec3 res0;
  int s0 = controller.closestPoint(wps[0], wps[1], pos0, res0);
  MU_CHECK(res0.isApprox(Vec3{-1.0, -1.0, 0.0}));
  MU_CHECK_EQ(-1, s0);

  // Test between waypoint start and end
  Vec3 pos1{0.5, 0.5, 0.0};
  Vec3 res1;
  int s1 = controller.closestPoint(wps[0], wps[1], pos1, res1);
  MU_CHECK(res1.isApprox(Vec3{0.5, 0.5, 0.0}));
  MU_CHECK_EQ(0, s1);

  // Test after waypoint end
  Vec3 pos2{1.5, 1.5, 0.0};
  Vec3 res2;
  int s2 = controller.closestPoint(wps[0], wps[1], pos2, res2);
  MU_CHECK(res2.isApprox(Vec3{1.5, 1.5, 0.0}));
  MU_CHECK_EQ(1, s2);

  return 0;
}

int test_CarrotController_carrotPoint() {
  CarrotController controller;

  std::vector<Vec3> wps;
  wps.emplace_back(0.0, 0.0, 0.0);
  wps.emplace_back(1.0, 0.0, 0.0);
  wps.emplace_back(2.0, 0.0, 0.0);
  wps.emplace_back(3.0, 0.0, 0.0);
  controller.configure(wps, 0.1);

  MU_CHECK(controller.wp_start.isApprox(Vec3::Zero()));
  MU_CHECK(controller.wp_end.isApprox(Vec3{1.0, 0.0, 0.0}));
  MU_CHECK_EQ(1, controller.wp_index);
  MU_CHECK_FLOAT(0.1, controller.look_ahead_dist);

  // Test before waypoint start
  Vec3 pos0{-1.0, 0.0, 0.0};
  Vec3 res0;
  int s0 = controller.carrotPoint(wps[0], wps[1], 0.1, pos0, res0);
  MU_CHECK(res0.isApprox(Vec3{0.0, 0.0, 0.0}));
  MU_CHECK_EQ(-1, s0);

  // Test between waypoint start and end
  Vec3 pos1{0.5, 0.0, 0.0};
  Vec3 res1;
  int s1 = controller.carrotPoint(wps[0], wps[1], 0.1, pos1, res1);
  MU_CHECK(res1.isApprox(Vec3{0.6, 0.0, 0.0}));
  MU_CHECK_EQ(0, s1);

  // Test after waypoint end
  Vec3 pos2{1.5, 0.0, 0.0};
  Vec3 res2;
  int s2 = controller.carrotPoint(wps[0], wps[1], 0.1, pos2, res2);
  MU_CHECK(res2.isApprox(Vec3{1.0, 0.0, 0.0}));
  MU_CHECK_EQ(1, s2);

  return 0;
}

int test_CarrotController_update() {
  CarrotController controller;

  std::vector<Vec3> wps;
  wps.emplace_back(0.0, 0.0, 0.0);
  wps.emplace_back(1.0, 0.0, 0.0);
  wps.emplace_back(2.0, 0.0, 0.0);
  wps.emplace_back(3.0, 0.0, 0.0);
  controller.configure(wps, 0.1);

  MU_CHECK(controller.wp_start.isApprox(Vec3::Zero()));
  MU_CHECK(controller.wp_end.isApprox(Vec3{1.0, 0.0, 0.0}));
  MU_CHECK_EQ(1, controller.wp_index);
  MU_CHECK_FLOAT(0.1, controller.look_ahead_dist);

  // Test before waypoint start
  Vec3 pos0{-1.0, 0.0, 0.0};
  Vec3 res0;
  int s0 = controller.update(pos0, res0);
  MU_CHECK(res0.isApprox(Vec3{0.0, 0.0, 0.0}));
  MU_CHECK_EQ(0, s0);

  // Test between waypoint start and end
  Vec3 pos1{0.5, 0.0, 0.0};
  Vec3 res1;
  int s1 = controller.update(pos1, res1);
  MU_CHECK(res1.isApprox(Vec3{0.6, 0.0, 0.0}));
  MU_CHECK_EQ(0, s1);

  // Test after waypoint end
  Vec3 pos2{1.5, 0.0, 0.0};
  Vec3 res2;
  int s2 = controller.update(pos2, res2);
  MU_CHECK(res2.isApprox(Vec3{1.0, 0.0, 0.0}));
  MU_CHECK_EQ(0, s2);
  MU_CHECK_EQ(2, controller.wp_index);
  MU_CHECK(controller.wp_start.isApprox(Vec3{1.0, 0.0, 0.0}));
  MU_CHECK(controller.wp_end.isApprox(Vec3{2.0, 0.0, 0.0}));

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_CarrotController_constructor);
  MU_ADD_TEST(test_CarrotController_configure);
  MU_ADD_TEST(test_CarrotController_closestPoint);
  MU_ADD_TEST(test_CarrotController_carrotPoint);
  MU_ADD_TEST(test_CarrotController_update);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
