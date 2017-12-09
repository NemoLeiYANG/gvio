#include "gvio/gvio_test.hpp"
#include "gvio/kitti/kitti.hpp"

namespace gvio {

TEST(RawDataset, load) {
  RawDataset raw_dataset("/data/raw", "2011_09_26", "0005");
  raw_dataset.load();
}

} // namespace gvio
