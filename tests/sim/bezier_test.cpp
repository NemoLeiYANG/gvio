#include "gvio/munit.hpp"
#include "gvio/sim/bezier.hpp"

namespace gvio {

int test_bezier() {
  // Setup
  std::vector<Vec3> points;
  points.emplace_back(0, 0, 0);
  points.emplace_back(1, 2, 0);
  points.emplace_back(2, 0, 0);
  points.emplace_back(3, 3, 0);

  // Create bezier curve
  double t = 0.0;
  double dt = 0.1;
  MatX pos_data;
  pos_data.resize((1.0 / dt) + 1, 4);

  int i = 0;
  while (t < 1.0) {
    const Vec3 p = bezier(points, t);
    pos_data.row(i) << t, p.transpose();
    i++;
    t += dt;
  }

  // Points data
  MatX points_data;
  points_data.resize(points.size(), 3);
  for (size_t i = 0; i < points.size(); i++) {
    points_data.row(i) = points[i];
  }

  // Save and plot
  mat2csv("/tmp/bezier_pos.dat", pos_data);
  mat2csv("/tmp/bezier_points.dat", points_data);

  PYTHON_SCRIPT("scripts/plot_bezier.py "
                "--pos_file /tmp/bezier_pos.dat "
                "--points_file /tmp/bezier_points.dat");

  return 0;
}

int test_bezier_derivative() {
  // Setup
  std::vector<Vec3> points;
  points.emplace_back(0, 0, 0);
  points.emplace_back(1, 3, 1);
  points.emplace_back(2, 0, 2);
  points.emplace_back(3, 3, 3);

  const Vec3 v = bezier_derivative(points, 0.5, 1);
  std::cout << v.transpose() << std::endl;
  std::cout << std::endl;

  const Vec3 a = bezier_derivative(points, 0.0, 2);
  std::cout << a.transpose() << std::endl;
  std::cout << std::endl;

  return 0;
}

int test_bezier_derivative2() {
  // Setup
  std::vector<Vec3> points;
  points.emplace_back(0, 0, 0);
  points.emplace_back(1, 2, 1);
  points.emplace_back(2, 1, 2);
  points.emplace_back(3, 3, 3);

  // Create bezier curve and its derivatives
  double t = 0.0;
  double dt = 0.1;
  MatX pos_data;
  MatX vel_data;
  MatX acc_data;
  pos_data.resize((1.0 / dt) + 1, 4);
  vel_data.resize((1.0 / dt) + 1, 4);
  acc_data.resize((1.0 / dt) + 1, 4);

  int i = 0;
  while (t < 1.0) {
    const Vec3 p = bezier(points, t);
    const Vec3 v = bezier_derivative(points, t, 1);
    const Vec3 a = bezier_derivative(points, t, 2);
    pos_data.row(i) << t, p.transpose();
    vel_data.row(i) << t, v.transpose();
    acc_data.row(i) << t, a.transpose();
    i++;
    t += dt;
  }

  // Points data
  MatX points_data;
  points_data.resize(points.size(), 3);
  for (size_t i = 0; i < points.size(); i++) {
    points_data.row(i) = points[i];
  }

  // Save and plot
  mat2csv("/tmp/bezier_pos.dat", pos_data);
  mat2csv("/tmp/bezier_vel.dat", vel_data);
  mat2csv("/tmp/bezier_acc.dat", acc_data);
  mat2csv("/tmp/bezier_points.dat", points_data);

  PYTHON_SCRIPT("scripts/plot_bezier.py "
                "--pos_file /tmp/bezier_pos.dat "
                "--vel_file /tmp/bezier_vel.dat "
                "--acc_file /tmp/bezier_acc.dat "
                "--points_file /tmp/bezier_points.dat");

  return 0;
}

int test_bezier_tangent() {
  // Setup
  std::vector<Vec3> points;
  points.emplace_back(0, 0, 0);
  points.emplace_back(1, 2, 0);
  points.emplace_back(2, 0, 0);
  points.emplace_back(3, 3, 0);

  // Create bezier curve
  double t = 0.0;
  double dt = 0.1;
  MatX pos_data;
  pos_data.resize((1.0 / dt) + 1, 4);
  MatX tangent_data;
  tangent_data.resize((1.0 / dt) + 1, 4);

  int i = 0;
  while (t < 1.0) {
    const Vec3 p = bezier(points, t);
    const Vec3 tangent = bezier_tangent(points, t);
    pos_data.row(i) << t, p.transpose();
    tangent_data.row(i) << t, tangent.transpose();
    i++;
    t += dt;
  }

  // Points data
  MatX points_data;
  points_data.resize(points.size(), 3);
  for (size_t i = 0; i < points.size(); i++) {
    points_data.row(i) = points[i];
  }

  // Save and plot
  mat2csv("/tmp/bezier_pos.dat", pos_data);
  mat2csv("/tmp/bezier_points.dat", points_data);
  mat2csv("/tmp/bezier_tangent.dat", tangent_data);

  PYTHON_SCRIPT("scripts/plot_bezier.py "
                "--pos_file /tmp/bezier_pos.dat "
                "--points_file /tmp/bezier_points.dat "
                "--tangent_file /tmp/bezier_tangent.dat");

  return 0;
}

int test_decasteljau() {
  // Setup
  std::vector<Vec3> points;
  points.emplace_back(0, 0, 0);
  points.emplace_back(1, 2, 0);
  points.emplace_back(2, 0, 0);
  points.emplace_back(3, 3, 0);

  // Create bezier curve
  double t = 0.0;
  double dt = 0.1;

  while (t < 1.0) {
    const Vec3 p0 = bezier(points, t);
    const Vec3 p1 = decasteljau(points, t);
    MU_CHECK((p0 - p1).norm() < 1e-6);
    t += dt;
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_bezier);
  MU_ADD_TEST(test_bezier_derivative);
  MU_ADD_TEST(test_bezier_derivative2);
  MU_ADD_TEST(test_bezier_tangent);
  MU_ADD_TEST(test_decasteljau);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
