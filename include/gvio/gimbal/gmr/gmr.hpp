#ifndef GVIO_GIMBAL_GMR_GMR_HPP
#define GVIO_GIMBAL_GMR_GMR_HPP

#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "gvio/gimbal/gmr/slic.hpp"

using namespace std;
typedef unsigned int UINT;

namespace gvio {

class GMR {
public:
  GMR();
  ~GMR();

  // Get saliency map of an input image
  cv::Mat GetSal(cv::Mat &img);

private:
  int spcount;        // superpxiels number
  double compactness; // superpixels compactness
  float alpha;        // balance the fittness and smoothness
  float delta;        // contral the edge weight
  int spcounta;       // actual superpixel number

private:
  // Get the superpixels of the image
  cv::Mat GetSup(const cv::Mat &img);

  // Get the adjacent matrix
  cv::Mat GetAdjLoop(const cv::Mat &supLab);

  // Get the affinity matrix of edges
  cv::Mat GetWeight(const cv::Mat &img,
                    const cv::Mat &supLab,
                    const cv::Mat &adj);

  // Get the optimal affinity matrix learned by minifold ranking (e.q. 3 in
  // paper)
  cv::Mat GetOptAff(const cv::Mat &W);

  // Get the indicator vector based on boundary prior
  cv::Mat GetBdQuery(const cv::Mat &supLab, int type);

  // Remove the obvious frame of the image
  cv::Mat RemoveFrame(const cv::Mat &img, int *wcut);
};
}

#endif // GVIO_GIMBAL_GMR_GMR_HPP
