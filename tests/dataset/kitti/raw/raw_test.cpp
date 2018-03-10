#include "gvio/munit.hpp"
#include "gvio/dataset/kitti/raw/raw.hpp"

namespace gvio {

#define TEST_DATA_PATH "test_data/kitti/raw/"

int test_RAW_load() {
  RawDataset raw_dataset(TEST_DATA_PATH, "2011_09_26", "0001");
  MU_CHECK(raw_dataset.load() == 0);
  return 0;
}

void test_suite() { MU_ADD_TEST(test_RAW_load); }

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
