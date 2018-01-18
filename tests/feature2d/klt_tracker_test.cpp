#include "gvio/munit.hpp"
#include "gvio/kitti/kitti.hpp"
#include "gvio/feature2d/klt_tracker.hpp"

namespace gvio {

int test_KLTTracker_detect() {
  KLTTracker tracker;

  RawDataset raw_dataset("/data/kitti/raw", "2011_09_26", "0005");
  if (raw_dataset.load() != 0) {
    return -1;
  }

  cv::Mat img0 = cv::imread(raw_dataset.cam0[0], CV_LOAD_IMAGE_COLOR);

  std::vector<Feature> features;
  tracker.detect(img0, features);
  img0 = draw_features(img0, features);

  MU_CHECK(features.size() > 0);

  // cv::imshow("Image", img0);
  // cv::waitKey(0);
  return 0;
}

int test_KLTTracker_track() {
  KLTTracker tracker;

  // Setup test data
  RawDataset raw_dataset("/data/kitti/raw", "2011_09_26", "0005");
  if (raw_dataset.load() != 0) {
    return -1;
  }

  cv::Mat img0 = cv::imread(raw_dataset.cam0[0], CV_LOAD_IMAGE_COLOR);
  cv::Mat img1 = cv::imread(raw_dataset.cam0[1], CV_LOAD_IMAGE_COLOR);

  // Detect features in image 0
  tracker.initialize(img0);

  // Detect features in image 1
  std::vector<Feature> f1;
  tracker.detect(img1, f1);

  // Match features
  // tracker.show_matches = true;
  Features tracked;
  tracker.track(img0, img1, f1, tracked);
  cv::waitKey(0);

  MU_CHECK(f1.size() > 0);
  MU_CHECK(tracked.size() > 0);

  return 0;
}

int test_KLTTracker_update() {
  // Setup test data
  RawDataset raw_dataset("/data/kitti/raw", "2011_09_26", "0005");
  if (raw_dataset.load() != 0) {
    return -1;
  }

  KLTTracker tracker;
  tracker.update(cv::imread(raw_dataset.cam0[0]));
  tracker.update(cv::imread(raw_dataset.cam0[1]));
  tracker.update(cv::imread(raw_dataset.cam0[2]));
  tracker.update(cv::imread(raw_dataset.cam0[3]));

  FeatureTracks tracks = tracker.getLostTracks();
  for (auto track : tracks) {
    MU_CHECK_EQ(0, track.frame_start);
    MU_CHECK_EQ(track.frame_start + track.frame_end + 1,
                (int) track.trackedLength());
  }

  return 0;
}

int test_KLTTracker_update2() {
  // Setup test data
  RawDataset raw_dataset("/data/kitti/raw", "2011_09_26", "0005");
  if (raw_dataset.load() != 0) {
    return -1;
  }

  KLTTracker tracker;
  tracker.show_matches = true;
  for (size_t i = 0; i < raw_dataset.cam0.size(); i++) {
    cv::Mat img = cv::imread(raw_dataset.cam0[i]);
    tracker.update(img);

    // cv::imshow("Image", img);
    if (cv::waitKey(0) == 113) {
      break;
    }
  }

  // FeatureTracks tracks = tracker.getLostTracks();
  // for (auto track : tracks) {
  //   MU_CHECK_EQ(0, track.frame_start);
  //   MU_CHECK_EQ(track.frame_start + track.frame_end + 1,
  //               (int) track.trackedLength());
  // }

  return 0;
}

int test_KLTTracker_demo() {
  KLTTracker tracker;

  cv::VideoCapture capture(0);

  cv::Mat img0;
  capture >> img0;
  tracker.show_matches = true;
  tracker.initialize(img0);

  while (true) {
    cv::Mat img_cur;
    capture >> img_cur;
    tracker.update(img_cur);

    // Break loop if 'q' was pressed
    if (cv::waitKey(1) == 113) {
      break;
    }
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_KLTTracker_detect);
  MU_ADD_TEST(test_KLTTracker_track);
  // MU_ADD_TEST(test_KLTTracker_update);
  MU_ADD_TEST(test_KLTTracker_update2);
  // MU_ADD_TEST(test_KLTTracker_demo);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
