#include "gvio/munit.hpp"
#include "gvio/quadrotor/mission.hpp"

namespace gvio {

#define TEST_GPS_CONFIG "test_configs/missions/mission_gps.yaml"
#define TEST_LOCAL_CONFIG "test_configs/missions/mission_local.yaml"
#define WAYPOINTS_FILE "/tmp/waypoints.dat"
#define STATE_FILE "/tmp/state.dat"

int test_Mission_constructor() {
  Mission mission;

  MU_CHECK(mission.configured == false);
  MU_CHECK(mission.check_waypoints);
  MU_CHECK_FLOAT(20.0, mission.threshold_waypoint_gap);
  MU_CHECK_FLOAT(0.5, mission.desired_velocity);

  return 0;
}

int test_Mission_configure_GPS() {
  Mission mission;

  int retval = mission.configure(TEST_GPS_CONFIG);
  mission.setGPSHomePoint(43.474024, -80.540287);

  MU_CHECK_EQ(0, retval);

  MU_CHECK(mission.check_waypoints);
  MU_CHECK_FLOAT(20.0, mission.threshold_waypoint_gap);

  MU_CHECK_FLOAT(0.5, mission.desired_velocity);
  MU_CHECK_EQ(4, (size_t) mission.local_waypoints.size());

  MU_CHECK_NEAR(0.0, mission.local_waypoints[0](0), 0.001);
  MU_CHECK_NEAR(0.0, mission.local_waypoints[0](1), 0.001);
  MU_CHECK_NEAR(10.0, mission.local_waypoints[0](2), 0.001);

  MU_CHECK_NEAR(6.01125, mission.local_waypoints[1](0), 0.001);
  MU_CHECK_NEAR(-10.1787, mission.local_waypoints[1](1), 0.001);
  MU_CHECK_NEAR(10.0, mission.local_waypoints[1](2), 0.001);

  MU_CHECK_NEAR(-7.45841, mission.local_waypoints[2](0), 0.001);
  MU_CHECK_NEAR(-17.2876, mission.local_waypoints[2](1), 0.001);
  MU_CHECK_NEAR(10.0, mission.local_waypoints[2](2), 0.001);

  MU_CHECK_NEAR(-10.5754, mission.local_waypoints[3](0), 0.001);
  MU_CHECK_NEAR(-9.20928, mission.local_waypoints[3](1), 0.001);
  MU_CHECK_NEAR(10.0, mission.local_waypoints[3](2), 0.001);

  return 0;
}

int test_Mission_configure_LOCAL() {
  Mission mission;

  int retval = mission.configure(TEST_LOCAL_CONFIG);
  MU_CHECK_EQ(0, retval);

  MU_CHECK(mission.check_waypoints);
  MU_CHECK_FLOAT(20.0, mission.threshold_waypoint_gap);

  MU_CHECK_FLOAT(0.5, mission.desired_velocity);
  MU_CHECK_EQ(4, (size_t) mission.local_waypoints.size());

  MU_CHECK_NEAR(0.0, mission.local_waypoints[0](0), 0.001);
  MU_CHECK_NEAR(0.0, mission.local_waypoints[0](1), 0.001);
  MU_CHECK_NEAR(10.0, mission.local_waypoints[0](2), 0.001);

  MU_CHECK_NEAR(5.0, mission.local_waypoints[1](0), 0.001);
  MU_CHECK_NEAR(0.0, mission.local_waypoints[1](1), 0.001);
  MU_CHECK_NEAR(10.0, mission.local_waypoints[1](2), 0.001);

  MU_CHECK_NEAR(5.0, mission.local_waypoints[2](0), 0.001);
  MU_CHECK_NEAR(5.0, mission.local_waypoints[2](1), 0.001);
  MU_CHECK_NEAR(10.0, mission.local_waypoints[2](2), 0.001);

  MU_CHECK_NEAR(0.0, mission.local_waypoints[3](0), 0.001);
  MU_CHECK_NEAR(5.0, mission.local_waypoints[3](1), 0.001);
  MU_CHECK_NEAR(10.0, mission.local_waypoints[3](2), 0.001);

  return 0;
}

int test_Mission_closestPoint() {
  Mission mission;

  // setup
  Vec3 wp_start{1.0, 1.0, 1.0};
  Vec3 wp_end{10.0, 10.0, 10.0};
  Vec3 position{5.0, 5.0, 5.0};
  Vec3 expected{5.0, 5.0, 5.0};

  // test and assert
  mission.wp_start = wp_start;
  mission.wp_end = wp_end;
  Vec3 point = mission.closestPoint(position);
  MU_CHECK(point.isApprox(expected));

  return 0;
}

int test_Mission_pointLineSide() {
  Mission mission;

  Vec3 wp_start{0.0, 0.0, 0.0};
  Vec3 wp_end{5.0, 0.0, 0.0};

  // test colinear
  Vec3 position{0.0, 0.0, 0.0};
  mission.wp_start = wp_start;
  mission.wp_end = wp_end;
  double s = mission.pointLineSide(position);
  MU_CHECK_EQ(0, s);

  // test left side
  position << 0.0, 3.0, 0.0;
  s = mission.pointLineSide(position);
  MU_CHECK_EQ(1, s);

  // test right side
  position << 0.0, -3.0, 0.0;
  s = mission.pointLineSide(position);
  MU_CHECK_EQ(-1, s);

  return 0;
}

int test_Mission_crossTrackError() {
  Mission mission;

  // setup
  mission.wp_start = Vec3{0.0, 0.0, 10.0};
  mission.wp_end = Vec3{10.0, 0.0, 10.0};
  Vec3 position = Vec3::Zero();
  double e = 0.0;

  // test position to the right of waypoint track
  position << 2.0, -3.0, 1.0;
  e = mission.crossTrackError(position);
  MU_CHECK_FLOAT(-3.0, e);

  // test position to the left of waypoint track
  position << 2.0, 3.0, 1.0;
  e = mission.crossTrackError(position);
  MU_CHECK_FLOAT(3.0, e);

  return 0;
}

int test_Mission_waypointInterpolate() {
  Mission mission;

  // setup
  double r = 0.0;
  mission.wp_start = Vec3{0.0, 0.0, 10.0};
  mission.wp_end = Vec3{10.0, 10.0, 10.0};
  Vec3 pos{5.0, 8.0, 0.0};
  Vec3 exp{6.5, 6.5, 10.0};

  // test and assert
  Vec3 point = mission.waypointInterpolate(pos, r);
  MU_CHECK(point.isApprox(exp));

  return 0;
}

int test_Mission_waypointHeading() {
  Mission mission;

  // setup
  mission.wp_start = Vec3{0.0, 0.0, 0.0};
  double heading = 0.0;

  // test 45 degree
  mission.wp_end = Vec3{1.0, 1.0, 0.0};
  heading = mission.waypointHeading();
  MU_CHECK_FLOAT(deg2rad(45.0), heading);

  // test 90 degree
  mission.wp_end = Vec3{0.0, 1.0, 0.0};
  heading = mission.waypointHeading();
  MU_CHECK_FLOAT(deg2rad(90.0), heading);

  // test 135 degree
  mission.wp_end = Vec3{-1.0, 1.0, 0.0};
  heading = mission.waypointHeading();
  MU_CHECK_FLOAT(deg2rad(135.0), heading);

  // test 180 degree
  mission.wp_end = Vec3{-1.0, 0.0, 0.0};
  heading = mission.waypointHeading();
  MU_CHECK_FLOAT(deg2rad(180.0), heading);

  // test -45 degree
  mission.wp_end = Vec3{1.0, -1.0, 0.0};
  heading = mission.waypointHeading();
  MU_CHECK_FLOAT(deg2rad(-45.0), heading);

  // test -90 degree
  mission.wp_end = Vec3{0.0, -1.0, 0.0};
  heading = mission.waypointHeading();
  MU_CHECK_FLOAT(deg2rad(-90.0), heading);

  // test -135 degree
  mission.wp_end = Vec3{-1.0, -1.0, 0.0};
  heading = mission.waypointHeading();
  MU_CHECK_FLOAT(deg2rad(-135.0), heading);

  return 0;
}

int test_Mission_waypointReached() {
  Mission mission;

  // setup
  mission.configured = true;
  mission.wp_end = Vec3{10.0, 10.0, 0.0};
  mission.threshold_waypoint_reached = 2.0;

  // test waypoint not reached
  Vec3 pos{0.0, 0.0, 0.0};
  int retval = mission.waypointReached(pos);
  MU_CHECK_EQ(0, retval);

  // test waypoint reached
  pos << 9.0, 10.0, 0.0;
  retval = mission.waypointReached(pos);
  MU_CHECK_EQ(1, retval);

  return 0;
}

int test_Mission_update() {
  Mission mission;

  // setup
  Vec3 position{2.0, 3.0, 0.0};
  mission.look_ahead_dist = 1;
  mission.threshold_waypoint_reached = 1.0;
  mission.configured = true;

  // push waypoints
  Vec3 wp;
  wp << 0.0, 0.0, 0.0;
  mission.wp_start = wp;
  mission.local_waypoints.push_back(wp);

  wp << 10.0, 10.0, 0.0;
  mission.wp_end = wp;
  mission.local_waypoints.push_back(wp);

  wp << 15.0, 15.0, 0.0;
  mission.local_waypoints.push_back(wp);

  // record waypoints
  std::ofstream waypoints_file(WAYPOINTS_FILE);
  for (auto wp : mission.local_waypoints) {
    waypoints_file << wp(0) << ", ";
    waypoints_file << wp(1) << ", ";
    waypoints_file << wp(2) << std::endl;
  }
  waypoints_file.close();

  // record robot state
  std::ofstream state_file(STATE_FILE);
  for (int i = 0; i < 40; i++) {
    // waypoint update
    Vec3 waypoint;
    int retval = mission.update(position, waypoint);
    if (retval != 0) {
      MU_CHECK_EQ(-2, retval);
      break;
    }

    // record position
    state_file << position(0) << ", ";
    state_file << position(1) << ", ";
    state_file << position(2) << ", ";

    // record interpolated waypoint
    state_file << waypoint(0) << ", ";
    state_file << waypoint(1) << ", ";
    state_file << waypoint(2) << std::endl;

    // update position
    position(0) = position(0) + 0.5;
    position(1) = position(1) + 0.5;
  }
  state_file.close();

  // assert
  MU_CHECK_EQ(3, mission.local_waypoints.size());

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_Mission_constructor);
  MU_ADD_TEST(test_Mission_configure_GPS);
  MU_ADD_TEST(test_Mission_configure_LOCAL);
  MU_ADD_TEST(test_Mission_closestPoint);
  MU_ADD_TEST(test_Mission_pointLineSide);
  MU_ADD_TEST(test_Mission_crossTrackError);
  MU_ADD_TEST(test_Mission_waypointInterpolate);
  MU_ADD_TEST(test_Mission_waypointHeading);
  MU_ADD_TEST(test_Mission_waypointReached);
  MU_ADD_TEST(test_Mission_update);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
