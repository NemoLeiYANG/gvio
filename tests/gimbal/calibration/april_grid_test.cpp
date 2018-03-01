// #define TEST_IMAGE "test_data/calibration/cam0/0.jpg"
//
// int test_AprilGrid_constructor() {
//   AprilGrid grid;
//
//   auto capture = cv::VideoCapture(0);
//   // const cv::Mat image = cv::imread(TEST_IMAGE);
//   if (capture.isOpened() == false) {
//     return -1;
//   }
//
//   while (true) {
//     cv::Mat image;
//     capture.read(image);
//
//     grid.extractCorners(image);
//     cv::imshow("Image", image);
//     if (cv::waitKey(1) == 113) {
//       break;
//     }
//   }
//
//   // cv::imshow("Image", image);
//   // cv::waitKey(1);
//
//   return 0;
// }
