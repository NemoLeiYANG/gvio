#include "gvio/gvio_test.hpp"
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

  EXPECT_FALSE(tracker.configured);
  EXPECT_EQ(10, tracker.fast_threshold);
  EXPECT_EQ(true, tracker.fast_nonmax_suppression);

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

TEST(FeatureTracker, match) {
  FeatureTracker tracker;

  cv::Mat img_top = cv::imread(TEST_IMAGE_TOP, CV_LOAD_IMAGE_COLOR);
  cv::Mat img_bottom = cv::imread(TEST_IMAGE_BOTTOM, CV_LOAD_IMAGE_COLOR);

  cv::imshow("Image", img_top);
  cv::waitKey(0);

  std::vector<Feature> f0;
  std::vector<Feature> f1;

  tracker.detect(img_top, f0);
  tracker.fea_ref = f0;
  tracker.detect(img_bottom, f1);

  tracker.img_size = img_top.size();
  tracker.match(f1);
}

} // namespace gvio
