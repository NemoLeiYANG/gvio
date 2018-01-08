#include "gvio/munit.h"
#include "gvio/feature2d/feature_tracker.hpp"

namespace gvio {

static const std::string TEST_IMAGE_TOP = "test_data/apriltag/top.png";
static const std::string TEST_IMAGE_BOTTOM = "test_data/apriltag/bottom.png";

int test_Feature_constructor() {
  cv::KeyPoint kp;
  Feature f(kp);

  MU_CHECK_FLOAT(-1.0, f.kp.angle);
  MU_CHECK_EQ(-1, f.kp.class_id);
  MU_CHECK_EQ(0, f.kp.octave);
  MU_CHECK_FLOAT(0.0, f.kp.response);
  MU_CHECK_FLOAT(0.0, f.kp.size);

  return 0;
}

int test_Feature_setTrackID() {
  cv::KeyPoint kp;
  Feature f(kp);

  f.setTrackID(100);
  MU_CHECK_EQ(100, f.track_id);

  return 0;
}

int test_Feature_getKeyPoint() {
  cv::KeyPoint kp;
  Feature f(kp);
  const cv::KeyPoint kp2 = f.getKeyPoint();

  MU_CHECK_FLOAT(-1.0, kp2.angle);
  MU_CHECK_EQ(-1, kp2.class_id);
  MU_CHECK_EQ(0, kp2.octave);
  MU_CHECK_FLOAT(0.0, kp2.response);
  MU_CHECK_FLOAT(0.0, kp2.size);

  return 0;
}

int test_FeatureTrack_constructor() {
  cv::KeyPoint kp1;
  cv::KeyPoint kp2;
  Feature f1(kp1);
  Feature f2(kp2);
  FeatureTrack track(0, 1, f1, f2);

  MU_CHECK_EQ(0, track.track_id);
  MU_CHECK_EQ(0, track.frame_start);
  MU_CHECK_EQ(1, track.frame_end);
  MU_CHECK_EQ(2, (int) track.track.size());

  return 0;
}

int test_FeatureTrack_update() {
  cv::KeyPoint kp1;
  cv::KeyPoint kp2;
  Feature f1(kp1);
  Feature f2(kp2);
  FeatureTrack track(0, 1, f1, f2);

  cv::KeyPoint kp3;
  Feature f3(kp3);
  track.update(2, f3);

  MU_CHECK_EQ(0, track.track_id);
  MU_CHECK_EQ(0, track.frame_start);
  MU_CHECK_EQ(2, track.frame_end);
  MU_CHECK_EQ(3, (int) track.track.size());

  return 0;
}

int test_FeatureTrack_last() {
  cv::KeyPoint kp1;
  cv::KeyPoint kp2;
  Feature f1(kp1);
  Feature f2(kp2);
  FeatureTrack track(0, 1, f1, f2);

  cv::Point2f pt(1.0, 2.0);
  cv::KeyPoint kp3(pt, 21);
  Feature f3(kp3);
  track.update(2, f3);
  auto t = track.last();

  MU_CHECK_FLOAT(1.0, t.kp.pt.x);
  MU_CHECK_FLOAT(2.0, t.kp.pt.y);
  MU_CHECK_EQ(21, t.kp.size);

  return 0;
}

int test_FeatureTrack_trackedLength() {
  cv::KeyPoint kp1;
  cv::KeyPoint kp2;
  Feature f1(kp1);
  Feature f2(kp2);
  FeatureTrack track(0, 1, f1, f2);

  cv::KeyPoint kp3;
  Feature f3(kp3);
  track.update(2, f3);

  MU_CHECK_EQ(0, track.track_id);
  MU_CHECK_EQ(0, track.frame_start);
  MU_CHECK_EQ(2, track.frame_end);
  MU_CHECK_EQ(3, (int) track.trackedLength());

  return 0;
}

int test_FeatureTracker_constructor() {
  FeatureTracker tracker;

  MU_CHECK(tracker.show_matches == false);

  MU_CHECK_EQ(-1, (int) tracker.counter_frame_id);
  MU_CHECK_EQ(-1, (int) tracker.counter_track_id);

  MU_CHECK_EQ(0, (int) tracker.tracking.size());
  MU_CHECK_EQ(0, (int) tracker.lost.size());
  MU_CHECK_EQ(0, (int) tracker.buffer.size());

  return 0;
}

int test_FeatureTracker_addTrack() {
  FeatureTracker tracker;
  Feature f1;
  Feature f2;

  tracker.addTrack(f1, f2);

  MU_CHECK_EQ(0, tracker.counter_track_id);
  MU_CHECK_EQ(-1, tracker.counter_frame_id);

  MU_CHECK_EQ(0, f1.track_id);
  MU_CHECK_EQ(0, f2.track_id);

  MU_CHECK_EQ(1, (int) tracker.tracking.size());
  MU_CHECK_EQ(0, (int) tracker.lost.size());
  MU_CHECK_EQ(1, (int) tracker.buffer.size());

  return 0;
}

int test_FeatureTracker_removeTrack() {
  FeatureTracker tracker;
  Feature f1;
  Feature f2;

  // Test remove from buffer as lost
  tracker.addTrack(f1, f2);
  tracker.removeTrack(0);

  MU_CHECK_EQ(0, (int) tracker.tracking.size());
  MU_CHECK_EQ(1, (int) tracker.lost.size());
  MU_CHECK_EQ(1, (int) tracker.buffer.size());

  // Clear lost and buffer for next test
  tracker.lost.clear();
  tracker.buffer.clear();

  // Test remove as not lost
  tracker.addTrack(f1, f2);
  tracker.removeTrack(1, false);

  MU_CHECK_EQ(0, (int) tracker.tracking.size());
  MU_CHECK_EQ(0, (int) tracker.lost.size());
  MU_CHECK_EQ(0, (int) tracker.buffer.size());

  return 0;
}

int test_FeatureTracker_updateTrack() {
  FeatureTracker tracker;
  Feature f1;
  Feature f2;
  Feature f3;

  tracker.addTrack(f1, f2);
  const int retval = tracker.updateTrack(0, f3);

  MU_CHECK_EQ(0, retval);
  MU_CHECK_EQ(3, (int) tracker.buffer[0].trackedLength());

  return 0;
}

int test_FeatureTracker_purge() {
  FeatureTracker tracker;
  const cv::Mat img0 = cv::imread(TEST_IMAGE_TOP, CV_LOAD_IMAGE_COLOR);
  const cv::Mat img1 = cv::imread(TEST_IMAGE_BOTTOM, CV_LOAD_IMAGE_COLOR);

  // Detect keypoints and descriptors
  cv::Mat mask;
  cv::Ptr<cv::ORB> orb = cv::ORB::create();

  // Detect features in image 0
  std::vector<cv::KeyPoint> k0;
  cv::Mat d0;
  std::vector<Feature> f0;
  orb->detectAndCompute(img0, mask, k0, d0);
  tracker.getFeatures(k0, d0, f0);
  tracker.fea_ref = f0;

  // Detect features in image 1
  std::vector<cv::KeyPoint> k1;
  cv::Mat d1;
  std::vector<Feature> f1;
  orb->detectAndCompute(img1, mask, k1, d1);
  tracker.getFeatures(k1, d1, f1);

  // Perform matching
  std::vector<cv::DMatch> matches;
  tracker.img_size = img0.size();
  tracker.match(f1, matches);

  // Purge
  const std::vector<FeatureTrack> tracks = tracker.purge(10);

  // Assert
  TrackID index = 0;
  MU_CHECK_EQ(10, tracks.size());
  for (auto t : tracks) {
    MU_CHECK_EQ(index, t.track_id);
    index++;
  }

  return 0;
}

// int test_FeatureTracker_detect() {
//   FeatureTracker tracker;
//
//   cv::VideoCapture capture(0);
//   cv::Mat frame;
//
//   double time_prev = time_now();
//   int frame_counter = 0;
//
//   std::vector<Feature> features;
//
//   while (cv::waitKey(1) != 113) {
//     capture >> frame;
//
//     tracker.detect(frame, features);
//     cv::imshow("Image", frame);
//
//     frame_counter++;
//     if (frame_counter % 10 == 0) {
//       std::cout << 10.0 / (time_now() - time_prev) << std::endl;
//       time_prev = time_now();
//       frame_counter = 0;
//     }
//   }
//
//   return 0;
// }

int test_FeatureTracker_conversions() {
  FeatureTracker tracker;
  const cv::Mat img = cv::imread(TEST_IMAGE_TOP, CV_LOAD_IMAGE_COLOR);

  // Detect keypoints and descriptors
  cv::Mat mask;
  std::vector<cv::KeyPoint> k0;
  cv::Mat d0;
  cv::Ptr<cv::ORB> orb = cv::ORB::create();
  orb->detectAndCompute(img, mask, k0, d0);

  // Convert keypoints and descriptors to features
  std::vector<Feature> f0;
  tracker.getFeatures(k0, d0, f0);

  // Convert features to keypoints and descriptors
  std::vector<cv::KeyPoint> k1;
  cv::Mat d1;
  tracker.getKeyPointsAndDescriptors(f0, k1, d1);

  // Assert
  int index = 0;
  for (auto &f : f0) {
    MU_CHECK(f.kp.pt.x == k0[index].pt.x);
    MU_CHECK(f.kp.pt.y == k0[index].pt.y);

    MU_CHECK(f.kp.pt.x == k1[index].pt.x);
    MU_CHECK(f.kp.pt.y == k1[index].pt.y);

    MU_CHECK(cvMatIsEqual(f.desc, d0.row(index)));
    MU_CHECK(cvMatIsEqual(f.desc, d1.row(index)));

    MU_CHECK_EQ(f.desc.size(), cv::Size(32, 1));
    index++;
  }

  return 0;
}

int test_FeatureTracker_match() {
  FeatureTracker tracker;
  const cv::Mat img0 = cv::imread(TEST_IMAGE_TOP, CV_LOAD_IMAGE_COLOR);
  const cv::Mat img1 = cv::imread(TEST_IMAGE_BOTTOM, CV_LOAD_IMAGE_COLOR);

  // Detect keypoints and descriptors
  cv::Mat mask;
  cv::Ptr<cv::ORB> orb = cv::ORB::create();

  // Detect features in image 0
  std::vector<cv::KeyPoint> k0;
  cv::Mat d0;
  std::vector<Feature> f0;
  orb->detectAndCompute(img0, mask, k0, d0);
  tracker.getFeatures(k0, d0, f0);
  tracker.fea_ref = f0;

  // Detect features in image 1
  std::vector<cv::KeyPoint> k1;
  cv::Mat d1;
  std::vector<Feature> f1;
  orb->detectAndCompute(img1, mask, k1, d1);
  tracker.getFeatures(k1, d1, f1);

  // Perform matching
  std::vector<cv::DMatch> matches;
  tracker.img_size = img0.size();
  tracker.match(f1, matches);

  // Draw inliers
  cv::Mat matches_img = draw_inliers(img0, img1, k0, k1, matches, 1);

  // cv::imshow("Matches", matches_img);
  // cv::waitKey();

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_Feature_constructor);
  MU_ADD_TEST(test_Feature_setTrackID);
  MU_ADD_TEST(test_Feature_getKeyPoint);

  MU_ADD_TEST(test_FeatureTrack_constructor);
  MU_ADD_TEST(test_FeatureTrack_update);
  MU_ADD_TEST(test_FeatureTrack_last);
  MU_ADD_TEST(test_FeatureTrack_trackedLength);

  MU_ADD_TEST(test_FeatureTracker_constructor);
  MU_ADD_TEST(test_FeatureTracker_addTrack);
  MU_ADD_TEST(test_FeatureTracker_removeTrack);
  MU_ADD_TEST(test_FeatureTracker_updateTrack);
  MU_ADD_TEST(test_FeatureTracker_purge);
  MU_ADD_TEST(test_FeatureTracker_conversions);
  MU_ADD_TEST(test_FeatureTracker_match);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
