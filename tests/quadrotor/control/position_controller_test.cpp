#include "gvio/munit.hpp"
#include "gvio/quadrotor/control/position_controller.hpp"

namespace gvio {

#define TEST_CONFIG "tests/configs/control/position_controller.yaml"

int test_PositionController_constructor() {
  PositionController controller;

  MU_CHECK_FLOAT(0.0, controller.dt);

  MU_CHECK_FLOAT(0.0, controller.x_controller.k_p);
  MU_CHECK_FLOAT(0.0, controller.x_controller.k_i);
  MU_CHECK_FLOAT(0.0, controller.x_controller.k_d);

  MU_CHECK_FLOAT(0.0, controller.y_controller.k_p);
  MU_CHECK_FLOAT(0.0, controller.y_controller.k_i);
  MU_CHECK_FLOAT(0.0, controller.y_controller.k_d);

  MU_CHECK_FLOAT(0.0, controller.z_controller.k_p);
  MU_CHECK_FLOAT(0.0, controller.z_controller.k_i);
  MU_CHECK_FLOAT(0.0, controller.z_controller.k_d);

  MU_CHECK_FLOAT(0.0, controller.outputs(0));
  MU_CHECK_FLOAT(0.0, controller.outputs(1));
  MU_CHECK_FLOAT(0.0, controller.outputs(2));
  MU_CHECK_FLOAT(0.0, controller.outputs(3));

  return 0;
}

// int test_PositionController_configure() {
//   PositionController controller;
//
//   controller.configure(TEST_CONFIG);
//
//   MU_CHECK(controller.configured == false);
//
//   MU_CHECK_FLOAT(0.0, controller.dt);
//
//   MU_CHECK_FLOAT(0.1, controller.x_controller.k_p);
//   MU_CHECK_FLOAT(0.2, controller.x_controller.k_i);
//   MU_CHECK_FLOAT(0.3, controller.x_controller.k_d);
//
//   MU_CHECK_FLOAT(0.1, controller.y_controller.k_p);
//   MU_CHECK_FLOAT(0.2, controller.y_controller.k_i);
//   MU_CHECK_FLOAT(0.3, controller.y_controller.k_d);
//
//   MU_CHECK_FLOAT(0.1, controller.z_controller.k_p);
//   MU_CHECK_FLOAT(0.2, controller.z_controller.k_i);
//   MU_CHECK_FLOAT(0.3, controller.z_controller.k_d);
//
//   MU_CHECK_FLOAT(0.6, controller.hover_throttle);
//
//   MU_CHECK_FLOAT(deg2rad(-50.0), controller.roll_limit[0]);
//   MU_CHECK_FLOAT(deg2rad(50.0), controller.roll_limit[1]);
//
//   MU_CHECK_FLOAT(deg2rad(-50.0), controller.pitch_limit[0]);
//   MU_CHECK_FLOAT(deg2rad(50.0), controller.pitch_limit[1]);
//
//   MU_CHECK_FLOAT(0.0, controller.setpoints(0));
//   MU_CHECK_FLOAT(0.0, controller.setpoints(1));
//   MU_CHECK_FLOAT(0.0, controller.setpoints(2));
//
//   MU_CHECK_FLOAT(0.0, controller.outputs(0));
//   MU_CHECK_FLOAT(0.0, controller.outputs(1));
//   MU_CHECK_FLOAT(0.0, controller.outputs(2));
//   MU_CHECK_FLOAT(0.0, controller.outputs(3));
//
//   return 0;
// }

// int test_PositionController_update() {
//   Vec3 setpoint_nwu;
//   Pose actual;
//   float yaw_setpoint, dt;
//   PositionController controller;
//
//   // setup
//   controller.configure(TEST_CONFIG);
//
//   // CHECK HOVERING PID OUTPUT
//   setpoint_nwu << 0, 0, 0;
//   actual.position << 0, 0, 0;
//   yaw_setpoint = 0;
//   dt = 0.1;
//   controller.update(setpoint_nwu, actual, yaw_setpoint, dt);
//   controller.printOutputs();
//
//   MU_CHECK_FLOAT(0.0, controller.outputs(0));
//   MU_CHECK_FLOAT(0.0, controller.outputs(1));
//   MU_CHECK_FLOAT(controller.hover_throttle, controller.outputs(3));
//
//   // CHECK MOVING TOWARDS THE Y LOCATION
//   setpoint_nwu << 0, 1, 0;
//   actual.position << 0, 0, 0;
//   yaw_setpoint = 0;
//   dt = 0.1;
//
//   controller.reset();
//   controller.update(setpoint_nwu, actual, yaw_setpoint, dt);
//   controller.printOutputs();
//
//   MU_CHECK(controller.outputs(0) < 0.0);
//   MU_CHECK_FLOAT(0.0, controller.outputs(1));
//
//   // CHECK MOVING TOWARDS THE X LOCATION
//   setpoint_nwu << 1, 0, 0;
//   actual.position << 0, 0, 0;
//   yaw_setpoint = 0;
//   dt = 0.1;
//
//   controller.reset();
//   controller.update(setpoint_nwu, actual, yaw_setpoint, dt);
//   controller.printOutputs();
//
//   MU_CHECK_FLOAT(0.0, controller.outputs(0));
//   MU_CHECK(controller.outputs(1) > 0.0);
//
//   // CHECK MOVING TOWARDS THE X AND Y LOCATION
//   setpoint_nwu << 1, 1, 0;
//   actual.position << 0, 0, 0;
//   yaw_setpoint = 0;
//   dt = 0.1;
//
//   controller.reset();
//   controller.update(setpoint_nwu, actual, yaw_setpoint, dt);
//   controller.printOutputs();
//
//   MU_CHECK(controller.outputs(0) < 0.0);
//   MU_CHECK(controller.outputs(1) > 0.0);
//
//   // CHECK MOVING YAW
//   setpoint_nwu << 0, 0, 0;
//   actual.position << 0, 0, 0;
//   yaw_setpoint = deg2rad(90.0);
//   dt = 0.1;
//
//   controller.reset();
//   controller.update(setpoint_nwu, actual, yaw_setpoint, dt);
//   controller.printOutputs();
//
//   MU_CHECK_FLOAT(yaw_setpoint, controller.outputs(2));
// }
//
// int test_PositionController_update2() {
//   Vec3 setpoint_nwu, euler;
//   Pose actual;
//   float yaw_setpoint, dt;
//   PositionController controller;
//
//   // setup
//   controller.configure(TEST_CONFIG);
//
//   // CHECK HEADING AT 90 DEGREE
//   setpoint_nwu << 1, 0, 0; // setpoint infront of quad
//   actual.position << 0, 0, 0;
//   euler << 0.0, 0.0, deg2rad(90.0);
//   actual.orientation = euler321ToQuat(euler);
//
//   yaw_setpoint = 0;
//   dt = 0.1;
//
//   controller.update(setpoint_nwu, actual, yaw_setpoint, dt);
//   controller.printOutputs();
//
//   MU_CHECK(controller.outputs(0) > 0);
//   MU_CHECK_NEAR(0.0, controller.outputs(1), 0.1);
//   MU_CHECK_NEAR(controller.hover_throttle, controller.outputs(3), 0.01);
//
//   return 0;
// }

void test_suite() {
  MU_ADD_TEST(test_PositionController_constructor);
  MU_ADD_TEST(test_PositionController_update);
  MU_ADD_TEST(test_PositionController_update2);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
