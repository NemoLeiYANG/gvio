#include "gvio/munit.hpp"
#include "gvio/gimbal/sbgc.hpp"

namespace gvio {

int test_SBGC_connectDisconnect() {
  SBGC sbgc("/dev/ttyUSB0");
  MU_CHECK_EQ(0, sbgc.connect());
  MU_CHECK_EQ(0, sbgc.disconnect());

  return 0;
}

int test_SBGC_sendFrame() {
  int retval;
  SBGCFrame frame;
  SBGC sbgc("/dev/ttyUSB0");

  // setup
  sbgc.connect();

  // turn motors on
  frame.buildFrame(CMD_MOTORS_ON);
  retval = sbgc.sendFrame(frame);
  MU_CHECK_EQ(retval, 0);
  sleep(1);

  // turn motors off
  frame.buildFrame(CMD_MOTORS_OFF);
  retval = sbgc.sendFrame(frame);
  MU_CHECK_EQ(retval, 0);
  sleep(1);

  return 0;
}

int test_SBGC_readFrame() {
  int retval;
  SBGCFrame frame;
  SBGC sbgc("/dev/ttyUSB0");

  // setup
  sbgc.connect();

  // read frame
  frame.buildFrame(CMD_BOARD_INFO);
  sbgc.sendFrame(frame);
  retval = sbgc.readFrame(CMD_BOARD_INFO_FRAME_SIZE, frame);

  // assert
  MU_CHECK_EQ(18, frame.data_size);
  MU_CHECK_EQ(0, retval);

  return 0;
}

int test_SBGC_getBoardInfo() {
  SBGC sbgc("/dev/ttyUSB0");

  // setup
  sbgc.connect();

  // test get board info
  sbgc.getBoardInfo();
  printf("board version: %d\n", sbgc.board_version);
  printf("firmware version: %d\n", sbgc.firmware_version);
  printf("debug mode: %d\n", sbgc.debug_mode);
  printf("board features: %d\n", sbgc.board_features);
  printf("connection flags: %d\n", sbgc.connection_flags);

  MU_CHECK_EQ(30, sbgc.board_version);
  MU_CHECK_EQ(2604, sbgc.firmware_version);

  return 0;
}

int test_SBGC_getRealTimeData() {
  SBGC sbgc("/dev/ttyUSB0");

  // setup
  sbgc.connect();

  // test get imu data
  for (int i = 0; i < 100; ++i) {
    sbgc.getRealtimeData();
    sbgc.data.printData();
    // printf("roll %f \n", sbgc.data.rc_angles(0));
    // printf("pitch %f \n", sbgc.data.rc_angles(1));
    // printf("yaw %f \n", sbgc.data.rc_angles(2));
  }

  return 0;
}

int test_SBGC_setAngle() {
  SBGC sbgc("/dev/ttyUSB0");

  MU_CHECK_EQ(0, sbgc.connect());
  sbgc.on();

  // Test Roll
  std::cout << "Testing roll!" << std::endl;
  sbgc.setAngle(-20, 0, 0);
  sleep(2);

  for (int angle = -20; angle <= 20; angle += 5) {
    sbgc.setAngle(angle, 0, 0);
    sleep(2);

    sbgc.getRealtimeData();
    MU_CHECK_NEAR(angle, sbgc.data.camera_angles(0), 2);
  }

  // Zero gimal
  std::cout << "Zero-ing gimbal!" << std::endl;
  sbgc.setAngle(0, 0, 0);
  sleep(2);

  // Test Pitch
  std::cout << "Testing pitch!" << std::endl;
  for (int angle = 0; angle <= 20; angle += 5) {
    sbgc.setAngle(0, angle, 0);
    sleep(2);

    sbgc.getRealtimeData();
    MU_CHECK_NEAR(angle, sbgc.data.camera_angles(1), 2);
  }
  sbgc.off();

  return 0;
}

int test_SBGC_setSpeedAngle() {
  SBGC sbgc("/dev/ttyUSB0");

  MU_CHECK_EQ(0, sbgc.connect());
  sbgc.on();

  sbgc.setSpeedAngle(0, 10, 0, 0, -2, 0);
  sleep(3);

  sbgc.off();

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_SBGC_connectDisconnect);
  MU_ADD_TEST(test_SBGC_sendFrame);
  MU_ADD_TEST(test_SBGC_readFrame);
  MU_ADD_TEST(test_SBGC_getBoardInfo);
  MU_ADD_TEST(test_SBGC_getRealTimeData);
  MU_ADD_TEST(test_SBGC_setAngle);
  MU_ADD_TEST(test_SBGC_setSpeedAngle);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
