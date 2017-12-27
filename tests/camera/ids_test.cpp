#include "gvio/munit.h"
#include "gvio/camera/ids.hpp"

#define TEST_CONFIG_PATH "test_configs/camera/webcam"

namespace gvio {

int test_IDSCamera_constructor() {
  IDSCamera camera;

  MU_FALSE(camera.configured);
  MU_FALSE(camera.connected);

  return 0;
}

int test_IDSCamera_configure() {
  IDSCamera camera;
  camera.configure("");

  return 0;
}

int test_IDSCamera_allocBuffers_and_freeBuffers() {
  IDSCamera camera;
  camera.configure("");

  camera.allocBuffers(2, 1);
  MU_CHECK_EQ(2, camera.buffers.size());
  MU_CHECK_EQ(2, camera.buffer_id.size());

  camera.freeBuffers();
  MU_CHECK_EQ(0, camera.buffers.size());
  MU_CHECK_EQ(0, camera.buffer_id.size());

  return 0;
}

int test_IDSCamera_setCaptureMode_and_getCaptureMode() {
  IDSCamera camera;
  camera.configure("");
  return 0;
}

int test_IDSCamera_setResolution_and_getResolution() {
  IDSCamera camera;
  camera.configure("");
  return 0;
}

int test_IDSCamera_getFrame() {
  IDSCamera camera;
  camera.configure("");

  cv::Mat image;
  camera.getFrame(image);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_IDSCamera_constructor);
  MU_ADD_TEST(test_IDSCamera_configure);
  MU_ADD_TEST(test_IDSCamera_allocBuffers_and_freeBuffers);
  MU_ADD_TEST(test_IDSCamera_setCaptureMode_and_getCaptureMode);
  MU_ADD_TEST(test_IDSCamera_setResolution_and_getResolution);
  MU_ADD_TEST(test_IDSCamera_getFrame);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
