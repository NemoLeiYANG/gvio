#include "gvio/munit.h"
#include "gvio/feature2d/feature.hpp"

namespace gvio {

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

void test_suite() {
  MU_ADD_TEST(test_Feature_constructor);
  MU_ADD_TEST(test_Feature_setTrackID);
  MU_ADD_TEST(test_Feature_getKeyPoint);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
