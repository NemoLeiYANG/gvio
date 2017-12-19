#include "gvio/munit.h"
#include "gvio/gimbal/gimbal.hpp"

#define TEST_CONFIG "test_configs/gimbal/config.yaml"

namespace gvio {

static void print_target_relative_to_body_frame(Vec3 &target) {
  std::cout << "target position (relative to quad in body frame): ";
  std::cout << std::fixed << std::setprecision(2) << target(0) << " ";
  std::cout << std::fixed << std::setprecision(2) << target(1) << " ";
  std::cout << std::fixed << std::setprecision(2) << target(2) << std::endl;
}

static void print_target_relative_to_body_planar_frame(Vec3 &target) {
  std::cout << "target position (relative to quad in body planar frame): ";
  std::cout << std::fixed << std::setprecision(2) << target(0) << " ";
  std::cout << std::fixed << std::setprecision(2) << target(1) << " ";
  std::cout << std::fixed << std::setprecision(2) << target(2) << std::endl;
}

int test_Gimbal_constructor() {
  Gimbal gimbal;

  MU_FALSE(gimbal.configured);
  MU_FALSE(gimbal.sbgc.configured);

  return 0;
}

int test_Gimbal_configure() {
  Gimbal gimbal;

  gimbal.configure(TEST_CONFIG);

  MU_CHECK(gimbal.configured);

  MU_CHECK_EQ("/dev/ttyUSB0", gimbal.sbgc.port);

  MU_CHECK_FLOAT(1.0, gimbal.camera_offset.position(0));
  MU_CHECK_FLOAT(2.0, gimbal.camera_offset.position(1));
  MU_CHECK_FLOAT(3.0, gimbal.camera_offset.position(2));

  MU_CHECK_FLOAT(deg2rad(1.0), gimbal.limits[0]);
  MU_CHECK_FLOAT(deg2rad(2.0), gimbal.limits[1]);
  MU_CHECK_FLOAT(deg2rad(3.0), gimbal.limits[2]);
  MU_CHECK_FLOAT(deg2rad(4.0), gimbal.limits[3]);
  MU_CHECK_FLOAT(deg2rad(5.0), gimbal.limits[4]);
  MU_CHECK_FLOAT(deg2rad(6.0), gimbal.limits[5]);

  return 0;
}

int test_Gimbal_getTargetInBF() {
  Gimbal gimbal;
  Vec3 target_C;
  Vec3 target_B;

  double roll = 0.0;
  double pitch = deg2rad(90);
  double yaw = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;

  gimbal.camera_offset = Pose(BODY_FRAME, roll, pitch, yaw, dx, dy, dz);

  // target is front of camera
  target_C << 0.0, 0.0, 1.0;
  target_B = gimbal.getTargetInBF(gimbal.camera_offset, target_C);
  std::cout << "[target is front of camera]\t\t";
  print_target_relative_to_body_frame(target_B);
  MU_CHECK_NEAR(0.0, target_B(0), 0.0000001);
  MU_CHECK_NEAR(0.0, target_B(1), 0.0000001);
  MU_CHECK_NEAR(-1.0, target_B(2), 0.0000001);

  // target left of camera
  target_C << -1.0, 0.0, 0.0;
  target_B = gimbal.getTargetInBF(gimbal.camera_offset, target_C);
  std::cout << "[target is left of camera]\t\t";
  print_target_relative_to_body_frame(target_B);
  MU_CHECK_NEAR(0.0, target_B(0), 0.0000001);
  MU_CHECK_NEAR(1.0, target_B(1), 0.0000001);
  MU_CHECK_NEAR(0.0, target_B(2), 0.0000001);

  // target is right of camera
  target_C << 1.0, 0.0, 0.0;
  target_B = gimbal.getTargetInBF(gimbal.camera_offset, target_C);
  std::cout << "[target is right of camera]\t\t";
  print_target_relative_to_body_frame(target_B);
  MU_CHECK_NEAR(0.0, target_B(0), 0.0000001);
  MU_CHECK_NEAR(-1.0, target_B(1), 0.0000001);
  MU_CHECK_NEAR(0.0, target_B(2), 0.0000001);

  // target is top of camera
  target_C << 0.0, -1.0, 0.0;
  target_B = gimbal.getTargetInBF(gimbal.camera_offset, target_C);
  std::cout << "[target is top of camera]\t\t";
  print_target_relative_to_body_frame(target_B);
  MU_CHECK_NEAR(1.0, target_B(0), 0.0000001);
  MU_CHECK_NEAR(0.0, target_B(1), 0.0000001);
  MU_CHECK_NEAR(0.0, target_B(2), 0.0000001);

  // target is bottom of camera
  target_C << 0.0, 1.0, 0.0;
  target_B = gimbal.getTargetInBF(gimbal.camera_offset, target_C);
  std::cout << "[target is bottom of camera]\t\t";
  print_target_relative_to_body_frame(target_B);
  MU_CHECK_NEAR(-1.0, target_B(0), 0.0000001);
  MU_CHECK_NEAR(0.0, target_B(1), 0.0000001);
  MU_CHECK_NEAR(0.0, target_B(2), 0.0000001);

  // target is top-left of camera
  target_C << -1.0, -1.0, 0.0;
  target_B = gimbal.getTargetInBF(gimbal.camera_offset, target_C);
  std::cout << "[target is top-left of camera]\t\t";
  print_target_relative_to_body_frame(target_B);
  MU_CHECK_NEAR(1.0, target_B(0), 0.0000001);
  MU_CHECK_NEAR(1.0, target_B(1), 0.0000001);
  MU_CHECK_NEAR(0.0, target_B(2), 0.0000001);

  // target is top-right of camera
  target_C << 1.0, -1.0, 0.0;
  target_B = gimbal.getTargetInBF(gimbal.camera_offset, target_C);
  std::cout << "[target is top-right of camera]\t\t";
  print_target_relative_to_body_frame(target_B);
  MU_CHECK_NEAR(1.0, target_B(0), 0.0000001);
  MU_CHECK_NEAR(-1.0, target_B(1), 0.0000001);
  MU_CHECK_NEAR(0.0, target_B(2), 0.0000001);

  // target is bottom-left of camera
  target_C << -1.0, 1.0, 0.0;
  target_B = gimbal.getTargetInBF(gimbal.camera_offset, target_C);
  std::cout << "[target is bottom-left of camera]\t";
  print_target_relative_to_body_frame(target_B);
  MU_CHECK_NEAR(-1.0, target_B(0), 0.0000001);
  MU_CHECK_NEAR(1.0, target_B(1), 0.0000001);
  MU_CHECK_NEAR(0.0, target_B(2), 0.0000001);

  // target is bottom-right of camera
  target_C << 1.0, 1.0, 0.0;
  target_B = gimbal.getTargetInBF(gimbal.camera_offset, target_C);
  std::cout << "[target is bottom-right of camera]\t";
  print_target_relative_to_body_frame(target_B);
  MU_CHECK_NEAR(-1.0, target_B(0), 0.0000001);
  MU_CHECK_NEAR(-1.0, target_B(1), 0.0000001);
  MU_CHECK_NEAR(0.0, target_B(2), 0.0000001);

  return 0;
}

int test_Gimbal_getTargetInBPF() {
  Gimbal gimbal;
  Quaternion joint;
  Vec3 euler;
  Vec3 target_C;
  Vec3 target_P;

  double roll = 0.0;
  double pitch = deg2rad(90);
  double yaw = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;

  target_C << 0.0, 0.0, 10.0; // let tag be directly infront of camera
  gimbal.camera_offset = Pose(BODY_FRAME, roll, pitch, yaw, dx, dy, dz);
  std::cout << "target is directly infront of camera: ";
  std::cout << "[" << target_C.transpose() << "]" << std::endl;

  // pitch forwards
  euler << 0.0, deg2rad(10), 0.0;
  joint = euler321ToQuat(euler);
  target_P = gimbal.getTargetInBPF(gimbal.camera_offset, target_C, joint);
  std::cout << "[quadrotor pitch forwards]\t\t";
  print_target_relative_to_body_planar_frame(target_P);
  MU_CHECK(target_P(0) < 0.0);
  MU_CHECK_NEAR(0.0, target_P(1), 0.000000001);
  MU_CHECK(target_P(2) < 0.0);

  // pitch backwards
  euler << 0.0, deg2rad(-10), 0.0;
  joint = euler321ToQuat(euler);
  target_P = gimbal.getTargetInBPF(gimbal.camera_offset, target_C, joint);
  std::cout << "[quadrotor pitch backwards]\t\t";
  print_target_relative_to_body_planar_frame(target_P);
  MU_CHECK(target_P(0) > 0.0);
  MU_CHECK_NEAR(0.0, target_P(1), 0.000000001);
  MU_CHECK(target_P(2) < 0.0);

  // roll left
  euler << deg2rad(-10), 0.0, 0.0;
  joint = euler321ToQuat(euler);
  target_P = gimbal.getTargetInBPF(gimbal.camera_offset, target_C, joint);
  std::cout << "[quadrotor roll left]\t\t\t";
  print_target_relative_to_body_planar_frame(target_P);
  MU_CHECK_NEAR(0.0, target_P(0), 0.0000000001);
  MU_CHECK(target_P(1) < 0.0);
  MU_CHECK(target_P(2) < 0.0);

  // roll right
  euler << deg2rad(10), 0.0, 0.0;
  joint = euler321ToQuat(euler);
  target_P = gimbal.getTargetInBPF(gimbal.camera_offset, target_C, joint);
  std::cout << "[quadrotor roll right]\t\t\t";
  print_target_relative_to_body_planar_frame(target_P);
  MU_CHECK_NEAR(0.0, target_P(0), 0.0000000001);
  MU_CHECK(target_P(1) > 0.0);
  MU_CHECK(target_P(2) < 0.0);

  // pitch forward, roll left
  euler << deg2rad(-10), deg2rad(10), 0.0;
  joint = euler321ToQuat(euler);
  target_P = gimbal.getTargetInBPF(gimbal.camera_offset, target_C, joint);
  std::cout << "[quadrotor pitch forward, roll left]\t";
  print_target_relative_to_body_planar_frame(target_P);
  MU_CHECK(target_P(0) < 0.0);
  MU_CHECK(target_P(1) < 0.0);
  MU_CHECK(target_P(2) < 0.0);

  // pitch forward, roll right
  euler << deg2rad(10), deg2rad(10), 0.0;
  joint = euler321ToQuat(euler);
  target_P = gimbal.getTargetInBPF(gimbal.camera_offset, target_C, joint);
  std::cout << "[quadrotor pitch forward, roll right]\t";
  print_target_relative_to_body_planar_frame(target_P);
  MU_CHECK(target_P(0) < 0.0);
  MU_CHECK(target_P(1) > 0.0);
  MU_CHECK(target_P(2) < 0.0);

  // pitch backwards, roll left
  euler << deg2rad(-10), deg2rad(-10), 0.0;
  joint = euler321ToQuat(euler);
  target_P = gimbal.getTargetInBPF(gimbal.camera_offset, target_C, joint);
  std::cout << "[quadrotor pitch backwards, roll left]\t";
  print_target_relative_to_body_planar_frame(target_P);
  MU_CHECK(target_P(0) > 0.0);
  MU_CHECK(target_P(1) < 0.0);
  MU_CHECK(target_P(2) < 0.0);

  // pitch backwards, roll right
  euler << deg2rad(10), deg2rad(-10), 0.0;
  joint = euler321ToQuat(euler);
  target_P = gimbal.getTargetInBPF(gimbal.camera_offset, target_C, joint);
  std::cout << "[quadrotor pitch backwards, roll right]\t";
  print_target_relative_to_body_planar_frame(target_P);
  MU_CHECK(target_P(0) > 0.0);
  MU_CHECK(target_P(1) > 0.0);
  MU_CHECK(target_P(2) < 0.0);

  return 0;
}

int test_Gimbal_getTargetInIF() {
  Quaternion frame;
  Vec3 euler, target_P, gimbal_position, target_W;

  // setup
  target_P << 1.0, 2.0, 0.0;
  gimbal_position << 0.0, 1.0, 2.0;

  // test 0 degree
  euler << 0.0, 0.0, deg2rad(0.0);
  frame = euler321ToQuat(euler);

  target_W = Gimbal::getTargetInIF(target_P, gimbal_position, frame);
  std::cout << "target if: " << target_W.transpose() << std::endl;

  EXPECT_FLOAT_EQ(1.0, target_W(0));
  EXPECT_FLOAT_EQ(3.0, target_W(1));
  EXPECT_FLOAT_EQ(2.0, target_W(2));

  // test 90 degree
  euler << 0.0, 0.0, deg2rad(90.0);
  frame = euler321ToQuat(euler);

  target_W = Gimbal::getTargetInIF(target_P, gimbal_position, frame);
  std::cout << "target if: " << target_W.transpose() << std::endl;

  EXPECT_FLOAT_EQ(-2.0, target_W(0));
  EXPECT_FLOAT_EQ(2.0, target_W(1));
  EXPECT_FLOAT_EQ(2.0, target_W(2));

  // test 180 degree
  euler << 0.0, 0.0, deg2rad(180.0);
  frame = euler321ToQuat(euler);

  target_W = Gimbal::getTargetInIF(target_P, gimbal_position, frame);
  std::cout << "target if: " << target_W.transpose() << std::endl;

  MU_CHECK_NEAR(-1.0, target_W(0), 0.001);
  MU_CHECK_NEAR(-1.0, target_W(1), 0.001);
  MU_CHECK_NEAR(2.0, target_W(2), 0.001);

  // test 270 degree
  euler << 0.0, 0.0, deg2rad(270.0);
  frame = euler321ToQuat(euler);

  target_W = Gimbal::getTargetInIF(target_P, gimbal_position, frame);
  std::cout << "target if: " << target_W.transpose() << std::endl;

  MU_CHECK_NEAR(2.0, target_W(0), 0.001);
  MU_CHECK_NEAR(0.0, target_W(1), 0.001);
  MU_CHECK_NEAR(2.0, target_W(2), 0.001);

  return 0;
}

int test_Gimbal_trackTarget() {
  Gimbal gimbal;
  Quaternion imu;
  Vec3 euler;
  Vec3 target_C;
  Vec3 target_P;

  double roll = 0.0;
  double pitch = deg2rad(90);
  double yaw = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;

  target_C << 0.0, 0.0, 10.0; // let tag be directly infront of camera
  gimbal.camera_offset = Pose(BODY_FRAME, roll, pitch, yaw, dx, dy, dz);
  std::cout << "target is directly infront of camera: ";
  std::cout << "[" << target_C.transpose() << "]" << std::endl;

  // pitch forwards
  euler << 0.0, deg2rad(10), 0.0;
  imu = euler321ToQuat(euler);
  target_P = Gimbal::getTargetInBPF(gimbal.camera_offset, target_C, imu);
  std::cout << "[quadrotor pitch forwards]\t\t";
  print_target_relative_to_body_planar_frame(target_P);
  gimbal.trackTarget(target_P);
  std::cout << gimbal.setpoints.transpose() << std::endl;

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_Gimbal_constructor);
  MU_ADD_TEST(test_Gimbal_getTargetInBF);
  MU_ADD_TEST(test_Gimbal_getTargetInBPF);
  MU_ADD_TEST(test_Gimbal_getTargetInIF);
  MU_ADD_TEST(test_Gimbal_trackTarget);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
