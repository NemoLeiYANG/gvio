#include "gvio/munit.hpp"
#include "gvio/apriltag/mit.hpp"
#include "gvio/camera/camera.hpp"

#define TEST_CONFIG "test_configs/apriltag/config.yaml"
#define TEST_IMAGE_CENTER "test_data/apriltag/center.png"
#define TEST_IMAGE_TOP "test_data/apriltag/top.png"
#define TEST_IMAGE_BOTTOM "test_data/apriltag/bottom.png"
#define TEST_IMAGE_LEFT "test_data/apriltag/left.png"
#define TEST_IMAGE_RIGHT "test_data/apriltag/right.png"
#define TEST_IMAGE_TOP_LEFT "test_data/apriltag/top_left.png"
#define TEST_IMAGE_BOTTOM_LEFT "test_data/apriltag/bottom_left.png"
#define TEST_IMAGE_TOP_RIGHT "test_data/apriltag/top_right.png"
#define TEST_IMAGE_BOTTOM_RIGHT "test_data/apriltag/bottom_right.png"
#define TEST_ILLUM_INVAR "test_data/apriltag/illum_invar.png"

namespace gvio {

int test_MITDetector_constructor() {
  MITDetector detector;

  MU_FALSE(detector.configured);

  MU_CHECK(detector.detector == nullptr);

  MU_CHECK(0 == detector.tag_configs.size());
  MU_CHECK("" == detector.camera_mode);
  MU_CHECK(0 == detector.camera_modes.size());
  MU_CHECK(0 == detector.camera_configs.size());
  MU_CHECK(false == detector.imshow);

  return 0;
}

int test_MITDetector_configure() {
  MITDetector detector;

  detector.configure(TEST_CONFIG);

  MU_CHECK(detector.configured);

  MU_CHECK(detector.detector != nullptr);

  MU_CHECK(2 == detector.tag_configs.size());
  MU_CHECK(detector.camera_modes[0] == detector.camera_mode);
  MU_CHECK(3 == detector.camera_modes.size());
  MU_CHECK(3 == detector.camera_configs.size());
  MU_FALSE(detector.imshow);

  return 0;
}

int test_MITDetector_illuminationInvarientTransform() {
  cv::Mat image;
  MITDetector detector;
  std::vector<AprilTags::TagDetection> tags;

  // setup
  detector.configure(TEST_CONFIG);
  image = cv::imread(TEST_ILLUM_INVAR, CV_LOAD_IMAGE_COLOR);

  // test and assert
  detector.illuminationInvariantTransform(image);
  // cv::imshow("image", image);
  // cv::waitKey();

  tags = detector.detector->extractTags(image);
  MU_CHECK(2 == tags.size());

  return 0;
}

int test_MITDetector_extractTags() {
  // setup
  MITDetector detector;
  detector.configure(TEST_CONFIG);

  // CENTER
  cv::Mat image = cv::imread(TEST_IMAGE_CENTER, CV_LOAD_IMAGE_COLOR);
  std::vector<TagPose> tags;
  int retval = detector.extractTags(image, tags);
  detector.prev_tag.detected = false;
  // tags[0].print();

  MU_CHECK(0 == retval);
  MU_CHECK(1 == tags.size());
  MU_CHECK_NEAR(0.0, tags[0].position(0), 0.15);
  MU_CHECK_NEAR(0.0, tags[0].position(1), 0.15);
  MU_CHECK_NEAR(2.2, tags[0].position(2), 0.15);
  tags.clear();

  // TOP
  image = cv::imread(TEST_IMAGE_TOP, CV_LOAD_IMAGE_COLOR);
  retval = detector.extractTags(image, tags);
  detector.prev_tag.detected = false;
  // tags[0].print();

  MU_CHECK(0 == retval);
  MU_CHECK(1 == tags.size());
  MU_CHECK_NEAR(0.0, tags[0].position(0), 0.15);
  MU_CHECK_NEAR(-0.5, tags[0].position(1), 0.15);
  MU_CHECK_NEAR(2.4, tags[0].position(2), 0.15);
  tags.clear();

  // RIGHT
  image = cv::imread(TEST_IMAGE_RIGHT, CV_LOAD_IMAGE_COLOR);
  retval = detector.extractTags(image, tags);
  detector.prev_tag.detected = false;
  // tags[0].print();

  MU_CHECK(0 == retval);
  MU_CHECK(1 == tags.size());
  MU_CHECK_NEAR(0.5, tags[0].position(0), 0.15);
  MU_CHECK_NEAR(0.0, tags[0].position(1), 0.15);
  MU_CHECK_NEAR(2.30, tags[0].position(2), 0.15);
  tags.clear();

  return 0;
}

int test_MITDetector_changeMode() {
  MITDetector detector;
  cv::Mat image1, image2, image3;

  // setup
  detector.configure(TEST_CONFIG);

  image1 = cv::Mat(480, 640, CV_64F, double(0));
  detector.changeMode(image1);
  MU_CHECK("640x480" == detector.camera_mode);

  image2 = cv::Mat(240, 320, CV_64F, double(0));
  detector.changeMode(image2);
  MU_CHECK("320x240" == detector.camera_mode);

  image3 = cv::Mat(120, 160, CV_64F, double(0));
  detector.changeMode(image3);
  MU_CHECK("160x120" == detector.camera_mode);

  return 0;
}

int test_MITDetector_maskImage() {
  // setup
  MITDetector detector;
  detector.configure(TEST_CONFIG);

  // CENTER
  cv::Mat image = cv::imread(TEST_IMAGE_CENTER, CV_LOAD_IMAGE_COLOR);
  std::vector<TagPose> tags;
  detector.extractTags(image, tags);
  detector.maskImage(tags[0], image);

  // cv::imshow("test", image);
  // cv::waitKey(100000);

  return 0;
}

int test_MITDetector_cropImage() {
  // setup
  MITDetector detector;
  detector.configure(TEST_CONFIG);

  // CENTER
  cv::Mat image = cv::imread(TEST_IMAGE_CENTER, CV_LOAD_IMAGE_COLOR);
  std::vector<TagPose> tags;
  detector.extractTags(image, tags);
  tags[0].print();

  image = cv::imread(TEST_IMAGE_CENTER, CV_LOAD_IMAGE_COLOR);
  detector.extractTags(image, tags);
  tags[0].print();
  cv::waitKey(100000);

  image = cv::imread(TEST_IMAGE_CENTER, CV_LOAD_IMAGE_COLOR);
  detector.extractTags(image, tags);
  tags[0].print();
  cv::waitKey(100000);

  // cv::imshow("test", image);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_MITDetector_constructor);
  MU_ADD_TEST(test_MITDetector_configure);
  MU_ADD_TEST(test_MITDetector_illuminationInvarientTransform);
  MU_ADD_TEST(test_MITDetector_extractTags);
  MU_ADD_TEST(test_MITDetector_changeMode);
  MU_ADD_TEST(test_MITDetector_maskImage);
  MU_ADD_TEST(test_MITDetector_cropImage);
}

} // namespace gvio
MU_RUN_TESTS(gvio::test_suite);
