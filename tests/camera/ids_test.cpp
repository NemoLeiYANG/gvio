#include "gvio/munit.hpp"
#include "gvio/camera/ids.hpp"

#define TEST_CONFIG_PATH "test_configs/camera/webcam"

namespace gvio {

static const std::string TEST_CONFIG = "test_configs/camera/ueye/config.yaml";

int test_IDSCamera_constructor() {
  IDSCamera camera;

  MU_FALSE(camera.configured);

  // Camera settings
  MU_CHECK_EQ(0, camera.camera_handle);
  MU_CHECK_EQ("NOT_SET", camera.trigger_mode);

  // Image settings
  MU_CHECK_EQ(0, camera.image_width);
  MU_CHECK_EQ(0, camera.image_height);
  MU_CHECK_EQ(0, camera.offset_x);
  MU_CHECK_EQ(0, camera.offset_y);
  MU_CHECK_EQ("MONO8", camera.color_mode);

  // Capture settings
  MU_CHECK_EQ(0, camera.pixel_clock);
  MU_CHECK_FLOAT(0.0, camera.frame_rate);
  MU_CHECK_FLOAT(0.0, camera.gain);

  return 0;
}

int test_IDSCamera_configure() {
  int retval;
  IDSCamera camera;

  retval = camera.configure(TEST_CONFIG);
  MU_CHECK_EQ(0, retval);

  return 0;
}

int test_IDSCamera_allocBuffers_and_freeBuffers() {
  int retval;
  IDSCamera camera;
  retval = camera.configure(TEST_CONFIG);
  MU_CHECK_EQ(0, retval);

  // Allocate buffers
  camera.allocBuffers(2, 1);
  MU_CHECK_EQ(2, camera.buffers.size());
  MU_CHECK_EQ(2, camera.buffer_id.size());

  // Free buffers
  camera.freeBuffers();
  MU_CHECK_EQ(0, camera.buffers.size());
  MU_CHECK_EQ(0, camera.buffer_id.size());

  return 0;
}

int test_IDSCamera_setTriggerMode_and_getTriggerMode() {
  int retval;
  IDSCamera camera;
  retval = camera.configure(TEST_CONFIG);
  MU_CHECK_EQ(0, retval);

  // Set trigger mode
  std::string set_trigger_mode = "FREE_RUN";
  retval = camera.setTriggerMode(set_trigger_mode);
  MU_CHECK_EQ(0, retval);

  // Get trigger mode
  std::string get_trigger_mode;
  camera.getTriggerMode(get_trigger_mode);
  MU_CHECK_EQ(0, retval);
  MU_CHECK_EQ(get_trigger_mode, set_trigger_mode);

  return 0;
}

int test_IDSCamera_setPixelClock_and_getPixelClock() {
  int retval;
  IDSCamera camera;
  retval = camera.configure(TEST_CONFIG);
  MU_CHECK_EQ(0, retval);

  // Set pixel clock
  int set_clock_rate = 32;
  retval = camera.setPixelClock(set_clock_rate);
  MU_CHECK_EQ(0, retval);

  // Get pixel clock
  int get_clock_rate = 0;
  retval = camera.getPixelClock(get_clock_rate);
  MU_CHECK_EQ(0, retval);
  MU_CHECK_EQ(get_clock_rate, set_clock_rate);

  return 0;
}

int test_IDSCamera_setColorMode_and_getColorMode() {
  int retval;
  IDSCamera camera;
  retval = camera.configure(TEST_CONFIG);
  MU_CHECK_EQ(0, retval);

  // Set color mode
  std::string set_color_mode = "MONO8";
  retval = camera.setColorMode(set_color_mode);
  MU_CHECK_EQ(0, retval);

  // Get color mode
  std::string get_color_mode = "MONO8";
  retval = camera.getColorMode(get_color_mode);
  MU_CHECK_EQ(0, retval);
  MU_CHECK_EQ(get_color_mode, set_color_mode);

  return 0;
}

int test_IDSCamera_setFrameRate_and_getFrameRate() {
  int retval;
  IDSCamera camera;
  retval = camera.configure(TEST_CONFIG);
  MU_CHECK_EQ(0, retval);

  // Set frame rate
  double set_frame_rate = 20;
  retval = camera.setFrameRate(set_frame_rate);
  MU_CHECK_EQ(0, retval);

  // Get frame rate
  double get_frame_rate = 0;
  retval = camera.getFrameRate(get_frame_rate);
  MU_CHECK_EQ(0, retval);
  MU_CHECK_NEAR(get_frame_rate, set_frame_rate, 0.1);

  return 0;
}

int test_IDSCamera_setGain_and_getGain() {
  int retval;
  IDSCamera camera;
  retval = camera.configure(TEST_CONFIG);
  MU_CHECK_EQ(0, retval);

  // Set frame rate
  double set_gain = 20;
  retval = camera.setGain(set_gain);
  MU_CHECK_EQ(0, retval);

  // Get frame rate
  int get_gain = 0;
  retval = camera.getGain(get_gain);
  MU_CHECK_EQ(0, retval);
  MU_CHECK_EQ(get_gain, set_gain);

  return 0;
}

int test_IDSCamera_setROI_and_getROI() {
  int retval;
  IDSCamera camera;
  retval = camera.configure(TEST_CONFIG);
  MU_CHECK_EQ(0, retval);

  // Set ROI
  int set_offset_x = 0;
  int set_offset_y = 0;
  int set_image_width = 100;
  int set_image_height = 100;
  retval = camera.setROI(set_offset_x,
                         set_offset_y,
                         set_image_width,
                         set_image_height);
  MU_CHECK_EQ(0, retval);

  // Get ROI
  int get_offset_x = -1;
  int get_offset_y = -1;
  int get_image_width = -1;
  int get_image_height = -1;
  retval = camera.getROI(get_offset_x,
                         get_offset_y,
                         get_image_width,
                         get_image_height);
  MU_CHECK_EQ(0, retval);
  MU_CHECK_EQ(get_offset_x, set_offset_x);
  MU_CHECK_EQ(get_offset_y, set_offset_y);
  MU_CHECK_EQ(get_image_width, set_image_width);
  MU_CHECK_EQ(get_image_height, set_image_height);

  return 0;
}

int test_IDSCamera_getImageSize() {
  int retval;
  IDSCamera camera;
  retval = camera.configure(TEST_CONFIG);
  MU_CHECK_EQ(0, retval);

  int image_width = -1;
  int image_height = -1;
  retval = camera.getImageSize(image_width, image_height);
  MU_CHECK_EQ(0, retval);
  MU_CHECK(image_width > 0);
  MU_CHECK(image_height > 0);

  return 0;
}

int test_IDSCamera_getFrame() {
  IDSCamera camera;
  camera.configure(TEST_CONFIG);

  struct timespec start = tic();
  // int index = 0;

  while (true) {
    cv::Mat image;
    if (camera.getFrame(image) != 0) {
      return -1;
    }

    cv::imshow("Image", image);
    if (cv::waitKey(1) == 113) {
      break;
    }

    printf("fps: %fs\n", 1.0 / toc(&start));
    start = tic();
    // index = 0;
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_IDSCamera_constructor);
  MU_ADD_TEST(test_IDSCamera_configure);
  MU_ADD_TEST(test_IDSCamera_allocBuffers_and_freeBuffers);
  MU_ADD_TEST(test_IDSCamera_setTriggerMode_and_getTriggerMode);
  MU_ADD_TEST(test_IDSCamera_setPixelClock_and_getPixelClock);
  MU_ADD_TEST(test_IDSCamera_setColorMode_and_getColorMode);
  MU_ADD_TEST(test_IDSCamera_setFrameRate_and_getFrameRate);
  MU_ADD_TEST(test_IDSCamera_setGain_and_getGain);
  MU_ADD_TEST(test_IDSCamera_setROI_and_getROI);
  MU_ADD_TEST(test_IDSCamera_getImageSize);
  // MU_ADD_TEST(test_IDSCamera_getFrame);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
