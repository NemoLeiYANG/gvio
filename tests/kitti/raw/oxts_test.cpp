#include "gvio/gvio_test.hpp"
#include "gvio/kitti/raw/oxts.hpp"

namespace gvio {

TEST(OXTS, load) {
  OXTS oxts;

  std::string oxts_path = "/raw/2011_09_26/2011_09_26_drive_0001_sync/oxts";
  oxts.load(TEST_KITTI_DATA + oxts_path);
}

} // namespace gvio
