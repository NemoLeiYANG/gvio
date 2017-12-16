#include "gvio/munit.h"
#include "gvio/util/stats.hpp"

namespace gvio {

int test_linreg() {
  Eigen::Vector2d p;
  std::vector<Eigen::Vector2d> points;
  double m;
  double c;
  double r;

  p << 1, 4;
  points.push_back(p);
  p << 2, 6;
  points.push_back(p);
  p << 4, 12;
  points.push_back(p);
  p << 5, 15;
  points.push_back(p);
  p << 10, 34;
  points.push_back(p);
  p << 20, 68;
  points.push_back(p);

  linreg(points, &m, &c, &r);
  std::cout << "m: " << m << std::endl;
  std::cout << "c: " << c << std::endl;
  std::cout << "r: " << r << std::endl;

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_linreg);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
