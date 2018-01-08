#include "gvio/munit.h"
#include "gvio/feature2d/feature_track.hpp"

namespace gvio {

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

void test_suite() {
  MU_ADD_TEST(test_FeatureTrack_constructor);
  MU_ADD_TEST(test_FeatureTrack_update);
  MU_ADD_TEST(test_FeatureTrack_last);
  MU_ADD_TEST(test_FeatureTrack_trackedLength);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
