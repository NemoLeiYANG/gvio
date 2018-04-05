#include "gvio/munit.hpp"
#include "gvio/dataset/euroc/ground_truth.hpp"

namespace gvio {

#define TEST_DATA "test_data/euroc/state_groundtruth_estimate0"

int test_GroundTruth_constructor() {
  GroundTruth ground_truth;

  MU_CHECK_EQ(0, ground_truth.timestamps.size());
  MU_CHECK_EQ(0, ground_truth.time.size());
  MU_CHECK_EQ(0, ground_truth.p_RS_R.size());
  MU_CHECK_EQ(0, ground_truth.q_RS.size());
  MU_CHECK_EQ(0, ground_truth.v_RS_R.size());
  MU_CHECK_EQ(0, ground_truth.b_w_RS_S.size());
  MU_CHECK_EQ(0, ground_truth.b_a_RS_S.size());

  return 0;
}

int test_GroundTruth_load() {
  GroundTruth ground_truth;

  int retval = ground_truth.load(TEST_DATA);
  MU_CHECK_EQ(0, retval);
  MU_CHECK_EQ(999, ground_truth.timestamps.size());
  MU_CHECK_EQ(999, ground_truth.time.size());
  MU_CHECK_EQ(999, ground_truth.p_RS_R.size());
  MU_CHECK_EQ(999, ground_truth.q_RS.size());
  MU_CHECK_EQ(999, ground_truth.v_RS_R.size());
  MU_CHECK_EQ(999, ground_truth.b_w_RS_S.size());
  MU_CHECK_EQ(999, ground_truth.b_a_RS_S.size());

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_GroundTruth_constructor);
  MU_ADD_TEST(test_GroundTruth_load);
}
}

MU_RUN_TESTS(gvio::test_suite);
