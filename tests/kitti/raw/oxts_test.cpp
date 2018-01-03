#include "gvio/munit.h"
#include "gvio/kitti/raw/oxts.hpp"

#ifndef TEST_KITTI_DATA
#define TEST_KITTI_DATA "test_data/kitti"
#endif

namespace gvio {

int test_OXTS_load() {
  OXTS oxts;

  std::string oxts_path = "/raw/2011_09_26/2011_09_26_drive_0001_sync/oxts";
  oxts.load(TEST_KITTI_DATA + oxts_path);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_OXTS_load);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
