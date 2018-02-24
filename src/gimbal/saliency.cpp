#include "gvio/gimbal/saliency.hpp"

namespace gvio {

Saliency::Saliency() {
  this->detector = cv::saliency::StaticSaliencySpectralResidual::create();
  // this->detector = cv::saliency::StaticSaliencyFineGrained::create();

  // const std::string training_path = "/usr/local/src/opencv_contrib/modules/"
  //                                   "saliency/samples/ObjectnessTrainedModel";
  // this->detector = cv::saliency::ObjectnessBING::create();
  // this->detector.dynamicCast<cv::saliency::ObjectnessBING>()->setTrainingPath(
  //     training_path);
}

Saliency::~Saliency() {}

int Saliency::detect(const cv::Mat &frame) {
  cv::Mat saliency_map;
  if (this->detector->computeSaliency(frame, saliency_map)) {
    cv::imshow("Saliency", saliency_map);
  }

  // std::vector<cv::Vec4i> saliency_map;
  // if (this->detector->computeSaliency(frame, saliency_map)) {
  //   const int ndet = int(saliency_map.size());
  //   std::cout << "Objectness done " << ndet << std::endl;
  //
  //   // The result are sorted by objectness. We only use the first maxd boxes
  //   // here.
  //   int maxd = 7;
  //   int step = 255 / maxd;
  //   int jitter = 9; // jitter to seperate single rects
  //   cv::Mat draw = frame.clone();
  //
  //   for (int i = 0; i < std::min(maxd, ndet); i++) {
  //     cv::Vec4i bb = saliency_map[i];
  //     cv::Scalar col =
  //         cv::Scalar(((i * step) % 255), 50, 255 - ((i * step) % 255));
  //     cv::Point off(cv::theRNG().uniform(-jitter, jitter),
  //                   cv::theRNG().uniform(-jitter, jitter));
  //     rectangle(draw,
  //               cv::Point(bb[0] + off.x, bb[1] + off.y),
  //               cv::Point(bb[2] + off.x, bb[3] + off.y),
  //               col,
  //               2);
  //     rectangle(draw,
  //               cv::Rect(20, 20 + i * 10, 10, 10),
  //               col,
  //               -1); // mini temperature scale
  //   }
  //
  //   cv::imshow("BING", draw);
  // }

  return 0;
}

} // namespace gvio
