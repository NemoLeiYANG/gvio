#include "gvio/gvio_test.hpp"
#include "gvio/kitti/kitti.hpp"
#include "gvio/feature2d/feature_tracker.hpp"

#define TEST_IMAGE_TOP "test_data/apriltag/top.png"
#define TEST_IMAGE_BOTTOM "test_data/apriltag/bottom.png"

namespace gvio {

TEST(Feature, constructor) {
  cv::KeyPoint kp;
  Feature f(kp);

  EXPECT_FLOAT_EQ(-1.0, f.kp.angle);
  EXPECT_EQ(-1, f.kp.class_id);
  EXPECT_EQ(0, f.kp.octave);
  EXPECT_FLOAT_EQ(0.0, f.kp.response);
  EXPECT_FLOAT_EQ(0.0, f.kp.size);
}

TEST(Feature, setTrackID) {
  cv::KeyPoint kp;
  Feature f(kp);

  f.setTrackID(100);
  EXPECT_EQ(100, f.track_id);
}

TEST(Feature, getKeyPoint) {
  cv::KeyPoint kp;
  Feature f(kp);
  const cv::KeyPoint kp2 = f.getKeyPoint();

  EXPECT_FLOAT_EQ(-1.0, kp2.angle);
  EXPECT_EQ(-1, kp2.class_id);
  EXPECT_EQ(0, kp2.octave);
  EXPECT_FLOAT_EQ(0.0, kp2.response);
  EXPECT_FLOAT_EQ(0.0, kp2.size);
}

TEST(FeatureTrack, constructor) {
  cv::KeyPoint kp1;
  cv::KeyPoint kp2;
  Feature f1(kp1);
  Feature f2(kp2);
  FeatureTrack track(0, 1, f1, f2);

  EXPECT_EQ(0, track.track_id);
  EXPECT_EQ(0, track.frame_start);
  EXPECT_EQ(1, track.frame_end);
  EXPECT_EQ(2, (int) track.track.size());
}

TEST(FeatureTrack, update) {
  cv::KeyPoint kp1;
  cv::KeyPoint kp2;
  Feature f1(kp1);
  Feature f2(kp2);
  FeatureTrack track(0, 1, f1, f2);

  cv::KeyPoint kp3;
  Feature f3(kp3);
  track.update(2, f3);

  EXPECT_EQ(0, track.track_id);
  EXPECT_EQ(0, track.frame_start);
  EXPECT_EQ(2, track.frame_end);
  EXPECT_EQ(3, (int) track.track.size());
}

TEST(FeatureTrack, last) {
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

  EXPECT_FLOAT_EQ(1.0, t.kp.pt.x);
  EXPECT_FLOAT_EQ(2.0, t.kp.pt.y);
  EXPECT_EQ(21, t.kp.size);
}

TEST(FeatureTrack, tracked_length) {
  cv::KeyPoint kp1;
  cv::KeyPoint kp2;
  Feature f1(kp1);
  Feature f2(kp2);
  FeatureTrack track(0, 1, f1, f2);

  cv::KeyPoint kp3;
  Feature f3(kp3);
  track.update(2, f3);

  EXPECT_EQ(0, track.track_id);
  EXPECT_EQ(0, track.frame_start);
  EXPECT_EQ(2, track.frame_end);
  EXPECT_EQ(3, (int) track.tracked_length());
}

TEST(FeatureTracker, constructor) {
  FeatureTracker tracker;

  EXPECT_FALSE(tracker.show_matches);

  EXPECT_EQ(-1, (int) tracker.counter_frame_id);
  EXPECT_EQ(-1, (int) tracker.counter_track_id);

  EXPECT_EQ(0, (int) tracker.tracking.size());
  EXPECT_EQ(0, (int) tracker.lost.size());
  EXPECT_EQ(0, (int) tracker.buffer.size());
}

TEST(FeatureTracker, addTrack) {
  FeatureTracker tracker;
  Feature f1;
  Feature f2;

  tracker.addTrack(f1, f2);

  EXPECT_EQ(0, tracker.counter_track_id);
  EXPECT_EQ(-1, tracker.counter_frame_id);

  EXPECT_EQ(0, f1.track_id);
  EXPECT_EQ(0, f2.track_id);

  EXPECT_EQ(1, (int) tracker.tracking.size());
  EXPECT_EQ(0, (int) tracker.lost.size());
  EXPECT_EQ(1, (int) tracker.buffer.size());
}

TEST(FeatureTracker, removeTrack) {
  FeatureTracker tracker;
  Feature f1;
  Feature f2;

  // Test remove from buffer
  tracker.addTrack(f1, f2);
  tracker.removeTrack(0);

  EXPECT_EQ(0, (int) tracker.tracking.size());
  EXPECT_EQ(0, (int) tracker.lost.size());
  EXPECT_EQ(0, (int) tracker.buffer.size());

  // Test remove as lost
  tracker.addTrack(f1, f2);
  tracker.removeTrack(1, true);

  EXPECT_EQ(0, (int) tracker.tracking.size());
  EXPECT_EQ(1, (int) tracker.lost.size());
  EXPECT_EQ(1, (int) tracker.buffer.size());
}

TEST(FeatureTracker, updateTrack) {
  FeatureTracker tracker;
  Feature f1;
  Feature f2;
  Feature f3;

  tracker.addTrack(f1, f2);
  const int retval = tracker.updateTrack(0, f3);

  EXPECT_EQ(0, retval);
  EXPECT_EQ(3, (int) tracker.buffer[0].tracked_length());
}

// TEST(FeatureTracker, detect) {
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
// }

TEST(FeatureTracker, conversions) {
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
    EXPECT_TRUE(f.kp.pt.x == k0[index].pt.x);
    EXPECT_TRUE(f.kp.pt.y == k0[index].pt.y);

    EXPECT_TRUE(f.kp.pt.x == k1[index].pt.x);
    EXPECT_TRUE(f.kp.pt.y == k1[index].pt.y);

    EXPECT_TRUE(cvMatIsEqual(f.desc, d0.row(index)));
    EXPECT_TRUE(cvMatIsEqual(f.desc, d1.row(index)));

    EXPECT_EQ(f.desc.size(), cv::Size(32, 1));
    index++;
  }
}

TEST(FeatureTracker, match) {
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
}

TEST(FeatureTracker, purge) {
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
  EXPECT_EQ(10, tracks.size());
  for (auto t : tracks) {
    EXPECT_EQ(index, t.track_id);
    index++;
  }
}

} // namespace gvio
