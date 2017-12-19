#include "gvio/munit.h"
#include "gvio/camera/camera.hpp"

#define TEST_CONFIG_PATH "test_configs/camera/webcam"

namespace gvio {

int test_Camera_constructor() {
  Camera camera;

  MU_FALSE(camera.configured);
  MU_FALSE(camera.initialized);

  MU_FALSE(camera.config.loaded);
  MU_CHECK(0 == (int) camera.modes.size());
  MU_CHECK(0 == (int) camera.configs.size());

  MU_CHECK(NULL == camera.capture);
  MU_CHECK_FLOAT(0.0, camera.last_tic);

  return 0;
}

int test_Camera_configure() {
  int retval;
  Camera camera;

  retval = camera.configure(TEST_CONFIG_PATH);
  MU_CHECK(0 == retval);

  return 0;
}

int test_Camera_changeMode() {
  cv::Mat image;
  Camera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();

  camera.getFrame(image);
  MU_CHECK(640 == image.cols);
  MU_CHECK(480 == image.rows);

  camera.changeMode("320x240");
  camera.getFrame(image);
  MU_CHECK(320 == image.cols);
  MU_CHECK(240 == image.rows);

  return 0;
}

int test_Camera_getFrame() {
  cv::Mat image;
  Camera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.getFrame(image);

  MU_FALSE(image.empty());

  return 0;
}

int test_Camera_run() {
  Camera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.run();

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_Camera_constructor);
  MU_ADD_TEST(test_Camera_configure);
  MU_ADD_TEST(test_Camera_changeMode);
  MU_ADD_TEST(test_Camera_getFrame);
  // MU_ADD_TEST(test_Camera_run);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
