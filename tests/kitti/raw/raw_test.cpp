#include "gvio/munit.h"
#include "gvio/kitti/raw/raw.hpp"

namespace gvio {

int test_load() {
  RawDataset raw_dataset("/data/kitti/raw", "2011_09_26", "0005");
  MU_CHECK(raw_dataset.load() == 0);
  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_load);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
