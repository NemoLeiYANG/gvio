#include "gvio/munit.hpp"
#include "gvio/dataset/kitti/kitti.hpp"
#include "gvio/feature2d/stereo_klt_tracker.hpp"

namespace gvio {

static const std::string KITTI_RAW_DATASET = "/data/kitti/raw";

struct test_config {
  RawDataset raw_dataset;
  StereoKLTTracker tracker;

  test_config() {}
};

struct test_config test_setup() {
  struct test_config test;

  test.raw_dataset =
      RawDataset(KITTI_RAW_DATASET, "2011_09_26", "0001", "extract");
  if (test.raw_dataset.load() != 0) {
    exit(-1);
  }

  // Obtain cam0 to cam1 transform
  const Mat3 R_cam1_cam0 = test.raw_dataset.calib_cam_to_cam.R[1];
  const Vec3 t_cam1_cam0 = test.raw_dataset.calib_cam_to_cam.T[1];
  const Mat4 T_cam1_cam0 = transformation_matrix(R_cam1_cam0, t_cam1_cam0);

  // Create camera properties
  const Vec2 image_size = test.raw_dataset.calib_cam_to_cam.S[0];
  CameraProperty cam0(0,
                      "pinhole",
                      test.raw_dataset.calib_cam_to_cam.K[0],
                      "radtan",
                      test.raw_dataset.calib_cam_to_cam.D[0],
                      image_size);
  CameraProperty cam1(1,
                      "pinhole",
                      test.raw_dataset.calib_cam_to_cam.K[1],
                      "radtan",
                      test.raw_dataset.calib_cam_to_cam.D[1],
                      image_size);

  // Initialize tracker
  test.tracker = StereoKLTTracker(cam0, cam1, T_cam1_cam0, 1, 5);

  return test;
}

int test_StereoKLTTracker_match() {
  struct test_config test = test_setup();

  const cv::Mat cam0_img = cv::imread(test.raw_dataset.cam0[0]);
  const cv::Mat cam1_img = cv::imread(test.raw_dataset.cam1[0]);
  std::vector<cv::Point2f> cam0_pts = test.tracker.detect(cam0_img);
  std::vector<cv::Point2f> cam1_pts = cam0_pts;
  std::vector<uchar> mask;
  test.tracker.match(cam0_img, cam1_img, cam0_pts, cam1_pts, mask);

  // Visualize
  cv::Mat match = draw_matches(cam0_img, cam1_img, cam0_pts, cam1_pts, mask);
  cv::imshow("Matches", match);
  cv::waitKey(0);

  // Assert
  MU_CHECK(cam0_pts.size() > 0);
  MU_CHECK(cam1_pts.size() > 0);
  MU_CHECK(cam0_pts.size() == cam1_pts.size());
  MU_CHECK(cam0_pts.size() == mask.size());

  return 0;
}

int test_StereoKLTTracker_initialize() {
  struct test_config test = test_setup();

  const cv::Mat cam0_img = cv::imread(test.raw_dataset.cam0[0]);
  const cv::Mat cam1_img = cv::imread(test.raw_dataset.cam1[0]);
  test.tracker.initialize(cam0_img, cam1_img);

  // Assert
  MU_CHECK_FLOAT(test.tracker.counter_frame_id, 0);
  MU_CHECK_FLOAT(test.tracker.counter_track_id, 0);
  MU_CHECK(test.tracker.cam0_pts.size() > 0);
  MU_CHECK(test.tracker.cam1_pts.size() > 0);
  MU_CHECK(test.tracker.prev_cam0_img.empty() == false);
  MU_CHECK(test.tracker.prev_cam1_img.empty() == false);

  return 0;
}

int test_StereoKLTTracker_track() {
  struct test_config test = test_setup();

  // Initialize tracker
  const cv::Mat cam0_img = cv::imread(test.raw_dataset.cam0[0]);
  const cv::Mat cam1_img = cv::imread(test.raw_dataset.cam1[0]);
  test.tracker.initialize(cam0_img, cam1_img);

  // Update tracker
  std::vector<cv::Point2f> cam0_pts = test.tracker.cam0_pts;
  std::vector<cv::Point2f> cam1_pts = test.tracker.cam1_pts;
  test.tracker.show_matches = true;

  for (int i = 1; i < 30; i++) {
    const cv::Mat cam0_img = cv::imread(test.raw_dataset.cam0[i]);
    const cv::Mat cam1_img = cv::imread(test.raw_dataset.cam1[i]);
    test.tracker.track(cam0_img, cam1_img, cam0_pts, cam1_pts);

    // Break loop if 'q' was pressed
    if (test.tracker.show_matches && cv::waitKey(0) == 113) {
      break;
    }
  }

  return 0;
}

int test_StereoKLTTracker_update() {
  struct test_config test = test_setup();

  // Initialize tracker
  const cv::Mat cam0_img = cv::imread(test.raw_dataset.cam0[0]);
  const cv::Mat cam1_img = cv::imread(test.raw_dataset.cam1[0]);
  test.tracker.initialize(cam0_img, cam1_img);

  // Update tracker
  std::vector<cv::Point2f> cam0_pts = test.tracker.cam0_pts;
  std::vector<cv::Point2f> cam1_pts = test.tracker.cam1_pts;
  test.tracker.show_matches = true;

  for (int i = 1; i < 30; i++) {
    const cv::Mat cam0_img = cv::imread(test.raw_dataset.cam0[i]);
    const cv::Mat cam1_img = cv::imread(test.raw_dataset.cam1[i]);
    test.tracker.track(cam0_img, cam1_img, cam0_pts, cam1_pts);

    // Break loop if 'q' was pressed
    if (test.tracker.show_matches && cv::waitKey(0) == 113) {
      break;
    }
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_StereoKLTTracker_match);
  MU_ADD_TEST(test_StereoKLTTracker_initialize);
  MU_ADD_TEST(test_StereoKLTTracker_track);
  // MU_ADD_TEST(test_StereoKLTTracker_update);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
