#include "gvio/gvio_test.hpp"
#include "gvio/kitti/raw/raw.hpp"

namespace gvio {

TEST(RawDataset, load) {
  RawDataset raw_dataset("/data/kitti/raw", "2011_09_26", "0005");
  raw_dataset.load();
}

} // namespace gvio
