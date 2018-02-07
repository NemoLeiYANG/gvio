#include "gvio/munit.hpp"
#include "gvio/control/pid.hpp"

namespace gvio {

int test_PID_constructor() {
  PID controller;

  MU_CHECK_FLOAT(0.0, controller.error_prev);
  MU_CHECK_FLOAT(0.0, controller.error_sum);

  MU_CHECK_FLOAT(0.0, controller.error_p);
  MU_CHECK_FLOAT(0.0, controller.error_i);
  MU_CHECK_FLOAT(0.0, controller.error_d);

  MU_CHECK_FLOAT(0.0, controller.k_p);
  MU_CHECK_FLOAT(0.0, controller.k_i);
  MU_CHECK_FLOAT(0.0, controller.k_d);

  controller = PID(1.0, 2.0, 3.0);

  MU_CHECK_FLOAT(0.0, controller.error_prev);
  MU_CHECK_FLOAT(0.0, controller.error_sum);

  MU_CHECK_FLOAT(0.0, controller.error_p);
  MU_CHECK_FLOAT(0.0, controller.error_i);
  MU_CHECK_FLOAT(0.0, controller.error_d);

  MU_CHECK_FLOAT(1.0, controller.k_p);
  MU_CHECK_FLOAT(2.0, controller.k_i);
  MU_CHECK_FLOAT(3.0, controller.k_d);

  return 0;
}

int test_PID_update() {
  PID controller{1.0, 1.0, 1.0};

  // test and assert
  double output = controller.update(10.0, 0.0, 0.1);

  MU_CHECK_FLOAT(1.0, controller.error_sum);
  MU_CHECK_FLOAT(10.0, controller.error_p);
  MU_CHECK_FLOAT(1.0, controller.error_i);
  MU_CHECK_FLOAT(100.0, controller.error_d);
  MU_CHECK_FLOAT(10.0, controller.error_prev);
  MU_CHECK_FLOAT(111.0, output);

  return 0;
}

int test_PID_reset() {
  PID controller;

  controller.error_prev = 0.1;
  controller.error_sum = 0.2;

  controller.error_p = 0.3;
  controller.error_i = 0.4;
  controller.error_d = 0.5;

  controller.reset();

  MU_CHECK_FLOAT(0.0, controller.error_prev);
  MU_CHECK_FLOAT(0.0, controller.error_sum);

  MU_CHECK_FLOAT(0.0, controller.error_p);
  MU_CHECK_FLOAT(0.0, controller.error_i);
  MU_CHECK_FLOAT(0.0, controller.error_d);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_PID_constructor);
  MU_ADD_TEST(test_PID_update);
  MU_ADD_TEST(test_PID_reset);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
