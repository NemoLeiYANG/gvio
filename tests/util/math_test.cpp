#include "gvio/munit.h"
#include "gvio/util/math.hpp"

namespace gvio {

int test_median() {
  std::vector<double> v;

  v.push_back(6);
  v.push_back(3);
  v.push_back(4);
  v.push_back(1);
  v.push_back(5);
  v.push_back(8);

  MU_CHECK_FLOAT(4.5, median(v));

  v.push_back(9);
  MU_CHECK_FLOAT(5.0, median(v));

  return 0;
}

int test_deg2radAndrad2deg() {
  double d_deg;
  double d_rad;

  d_deg = 10;
  d_rad = deg2rad(d_deg);
  MU_CHECK_FLOAT(d_deg, rad2deg(d_rad));

  return 0;
}

int test_wrapTo180() {
  double retval;

  // normal cases
  retval = wrapTo180(90.0);
  MU_CHECK_FLOAT(90.0, retval);

  retval = wrapTo180(180.0);
  MU_CHECK_FLOAT(-180.0, retval);

  retval = wrapTo180(270.0);
  MU_CHECK_FLOAT(-90.0, retval);

  retval = wrapTo180(360.0);
  MU_CHECK_FLOAT(0.0, retval);

  // edge cases
  retval = wrapTo180(-180.0);
  MU_CHECK_FLOAT(-180.0, retval);

  retval = wrapTo180(-90.0);
  MU_CHECK_FLOAT(-90.0, retval);

  retval = wrapTo180(450.0);
  MU_CHECK_FLOAT(90.0, retval);

  return 0;
}

int test_wrapTo360() {
  double retval;

  // normal cases
  retval = wrapTo360(90.0);
  MU_CHECK_FLOAT(90.0, retval);

  retval = wrapTo360(180.0);
  MU_CHECK_FLOAT(180.0, retval);

  retval = wrapTo360(270.0);
  MU_CHECK_FLOAT(270.0, retval);

  retval = wrapTo360(360.0);
  MU_CHECK_FLOAT(0.0, retval);

  retval = wrapTo360(450.0);
  MU_CHECK_FLOAT(90.0, retval);

  // edge cases
  retval = wrapTo360(-180.0);
  MU_CHECK_FLOAT(180.0, retval);

  retval = wrapTo360(-90.0);
  MU_CHECK_FLOAT(270.0, retval);

  return 0;
}

int test_cross_track_error() {
  Vec2 pos, p1, p2;

  pos << 2, 2;
  p1 << 1, 1;
  p2 << 5, 5;
  MU_CHECK_FLOAT(0.0, cross_track_error(p1, p2, pos));

  pos << 2, 3;
  p1 << 1, 1;
  p2 << 5, 5;
  MU_CHECK(0.0 < cross_track_error(p1, p2, pos));

  return 0;
}

int test_point_left_right() {
  Vec2 pos, p1, p2;

  pos << 2, 3;
  p1 << 1, 1;
  p2 << 5, 5;
  MU_CHECK_EQ(1, point_left_right(p1, p2, pos));

  pos << 2, 1;
  p1 << 1, 1;
  p2 << 5, 5;
  MU_CHECK_EQ(2, point_left_right(p1, p2, pos));

  pos << 2, 2;
  p1 << 1, 1;
  p2 << 5, 5;
  MU_CHECK_EQ(0, point_left_right(p1, p2, pos));

  pos << 2, 1;
  p1 << 5, 5;
  p2 << 1, 1;
  MU_CHECK_EQ(1, point_left_right(p1, p2, pos));

  pos << 2, 3;
  p1 << 5, 5;
  p2 << 1, 1;
  MU_CHECK_EQ(2, point_left_right(p1, p2, pos));

  pos << 2, 2;
  p1 << 5, 5;
  p2 << 1, 1;
  MU_CHECK_EQ(0, point_left_right(p1, p2, pos));

  return 0;
}

int test_closest_point() {
  int retval;
  Vec2 p1, p2, p3, closest;

  // setup
  p1 << 0, 0;
  p2 << 5, 0;

  // point middle of point a, b
  p3 << 2, 2;
  retval = closest_point(p1, p2, p3, closest);
  MU_CHECK_EQ(0, retval);
  MU_CHECK_FLOAT(2.0, closest(0));
  MU_CHECK_FLOAT(0.0, closest(1));

  // // point before of point a
  // p3 << -1, 2;
  // retval = closest_point(p1, p2, p3, closest);
  // MU_CHECK_EQ(1, retval);
  // MU_CHECK_FLOAT(-1.0, closest(0));
  // MU_CHECK_FLOAT(0.0, closest(1));
  //
  // // point after point b
  // p3 << 6, 2;
  // retval = closest_point(p1, p2, p3, closest);
  // MU_CHECK_EQ(2, retval);
  // MU_CHECK_FLOAT(6.0, closest(0));
  // MU_CHECK_FLOAT(0.0, closest(1));
  //
  // // if point 1 and 2 are same
  // p1 << 0, 0;
  // p2 << 0, 0;
  // p3 << 0, 2;
  // retval = closest_point(p1, p2, p3, closest);
  // MU_CHECK_EQ(-1, retval);
  // MU_CHECK_FLOAT(0.0, closest(0));
  // MU_CHECK_FLOAT(0.0, closest(1));

  return 0;
}

int test_lerp() {
  const Vec2 a{0.0, 5.0};
  const Vec2 b{5.0, 0.0};
  const Vec2 result = lerp(a, b, 0.8);
  std::cout << result << std::endl;

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_median);
  MU_ADD_TEST(test_deg2radAndrad2deg);
  MU_ADD_TEST(test_wrapTo180);
  MU_ADD_TEST(test_wrapTo360);
  MU_ADD_TEST(test_cross_track_error);
  MU_ADD_TEST(test_point_left_right);
  MU_ADD_TEST(test_closest_point);
  MU_ADD_TEST(test_lerp);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
