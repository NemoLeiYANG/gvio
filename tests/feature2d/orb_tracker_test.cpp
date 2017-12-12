#include "gvio/gvio_test.hpp"
#include "gvio/kitti/kitti.hpp"
#include "gvio/feature2d/orb_tracker.hpp"

namespace gvio {

TEST(ORBTracker, update) {
  ORBTracker tracker;

  RawDataset raw_dataset("/data/kitti/raw", "2011_09_26", "0005");
  raw_dataset.load();

  // cv::VideoCapture capture(0);
  // cv::Mat img0;
  // capture >> img0;

  tracker.show_matches = true;
  tracker.initialize(cv::imread(raw_dataset.cam0[0], CV_LOAD_IMAGE_COLOR));
  // tracker.initialize(img0);

  for (int i = 1; i < 100; i++) {
    cv::Mat image = cv::imread(raw_dataset.cam0[i], CV_LOAD_IMAGE_COLOR);
    // cv::Mat image;
    // capture >> image;

    tracker.update(image);
    // std::cout << tracker << std::endl;

    // Break loop if 'q' was pressed
    if (cv::waitKey(1) == 113) {
      break;
    }
  }
}

} // namespace gvio
