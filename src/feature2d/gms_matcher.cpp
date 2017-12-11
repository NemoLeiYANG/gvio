#include "gvio/feature2d/gms_matcher.hpp"

namespace gvio {

std::vector<cv::Point2f> GMSMatcher::normalizePoints(
    const std::vector<cv::KeyPoint> &kp, const cv::Size &size) {
  const size_t numP = kp.size();
  const int width = size.width;
  const int height = size.height;

  std::vector<cv::Point2f> npts;
  npts.resize(numP);

  for (size_t i = 0; i < numP; i++) {
    npts[i].x = kp[i].pt.x / width;
    npts[i].y = kp[i].pt.y / height;
  }

  return npts;
}

void GMSMatcher::convertMatches(const std::vector<cv::DMatch> &vDMatches,
                                std::vector<std::pair<int, int>> &vMatches) {
  vMatches.resize(mNumberMatches);
  for (size_t i = 0; i < mNumberMatches; i++) {
    vMatches[i] =
        std::pair<int, int>(vDMatches[i].queryIdx, vDMatches[i].trainIdx);
  }
}

int GMSMatcher::getGridIndexLeft(const cv::Point2f &pt, const int type) {
  int x = 0, y = 0;

  if (type == 1) {
    x = floor(pt.x * mGridSizeLeft.width);
    y = floor(pt.y * mGridSizeLeft.height);
  }

  if (type == 2) {
    x = floor(pt.x * mGridSizeLeft.width + 0.5);
    y = floor(pt.y * mGridSizeLeft.height);
  }

  if (type == 3) {
    x = floor(pt.x * mGridSizeLeft.width);
    y = floor(pt.y * mGridSizeLeft.height + 0.5);
  }

  if (type == 4) {
    x = floor(pt.x * mGridSizeLeft.width + 0.5);
    y = floor(pt.y * mGridSizeLeft.height + 0.5);
  }

  if (x >= mGridSizeLeft.width || y >= mGridSizeLeft.height) {
    return -1;
  }

  return x + y * mGridSizeLeft.width;
}

int GMSMatcher::getGridIndexRight(const cv::Point2f &pt) {
  int x = floor(pt.x * mGridSizeRight.width);
  int y = floor(pt.y * mGridSizeRight.height);
  return x + y * mGridSizeRight.width;
}

void GMSMatcher::assignMatchPairs(const int GridType) {
  for (size_t i = 0; i < mNumberMatches; i++) {
    cv::Point2f &lp = mvP1[mvMatches[i].first];
    cv::Point2f &rp = mvP2[mvMatches[i].second];

    int lgidx = mvMatchPairs[i].first = getGridIndexLeft(lp, GridType);
    int rgidx = -1;

    if (GridType == 1) {
      rgidx = mvMatchPairs[i].second = getGridIndexRight(rp);
    } else {
      rgidx = mvMatchPairs[i].second;
    }

    if (lgidx < 0 || rgidx < 0)
      continue;

    mMotionStatistics.at<int>(lgidx, rgidx)++;
    mNumberPointsInPerCellLeft[lgidx]++;
  }
}

void GMSMatcher::verifyCellPairs(const int RotationType) {
  const int *currentRP = mRotationPatterns[RotationType - 1];

  for (int i = 0; i < mGridNumberLeft; i++) {
    if (cv::sum(mMotionStatistics.row(i))[0] == 0) {
      mCellPairs[i] = -1;
      continue;
    }

    int max_number = 0;
    for (int j = 0; j < mGridNumberRight; j++) {
      int *value = mMotionStatistics.ptr<int>(i);
      if (value[j] > max_number) {
        mCellPairs[i] = j;
        max_number = value[j];
      }
    }

    int idx_grid_rt = mCellPairs[i];
    const int *NB9_lt = mGridNeighborLeft.ptr<int>(i);
    const int *NB9_rt = mGridNeighborRight.ptr<int>(idx_grid_rt);

    int score = 0;
    double thresh = 0;
    int numpair = 0;

    for (size_t j = 0; j < 9; j++) {
      int ll = NB9_lt[j];
      int rr = NB9_rt[currentRP[j] - 1];
      if (ll == -1 || rr == -1)
        continue;

      score += mMotionStatistics.at<int>(ll, rr);
      thresh += mNumberPointsInPerCellLeft[ll];
      numpair++;
    }

    thresh = THRESH_FACTOR * sqrt(thresh / numpair);

    if (score < thresh) {
      mCellPairs[i] = -2;
    }
  }
}

std::vector<int> GMSMatcher::getNB9(const int idx, const cv::Size &GridSize) {
  std::vector<int> NB9(9, -1);

  int idx_x = idx % GridSize.width;
  int idx_y = idx / GridSize.width;

  for (int yi = -1; yi <= 1; yi++) {
    for (int xi = -1; xi <= 1; xi++) {
      int idx_xx = idx_x + xi;
      int idx_yy = idx_y + yi;

      if (idx_xx < 0 || idx_xx >= GridSize.width || idx_yy < 0 ||
          idx_yy >= GridSize.height)
        continue;

      NB9[xi + 4 + yi * 3] = idx_xx + idx_yy * GridSize.width;
    }
  }

  return NB9;
}

void GMSMatcher::initNeighbors(cv::Mat &neighbor, const cv::Size &GridSize) {
  for (int i = 0; i < neighbor.rows; i++) {
    std::vector<int> NB9 = this->getNB9(i, GridSize);
    int *data = neighbor.ptr<int>(i);
    memcpy(data, &NB9[0], sizeof(int) * 9);
  }
}

void GMSMatcher::setScale(const int Scale) {
  // Set Scale
  mGridSizeRight.width = mGridSizeLeft.width * mScaleRatios[Scale];
  mGridSizeRight.height = mGridSizeLeft.height * mScaleRatios[Scale];
  mGridNumberRight = mGridSizeRight.width * mGridSizeRight.height;

  // Initialize the neihbor of right grid
  mGridNeighborRight = cv::Mat::zeros(mGridNumberRight, 9, CV_32SC1);
  initNeighbors(mGridNeighborRight, mGridSizeRight);
}

int GMSMatcher::run(const int rotationType) {
  mvbInlierMask.assign(mNumberMatches, false);

  // Initialize Motion Statisctics
  mMotionStatistics =
      cv::Mat::zeros(mGridNumberLeft, mGridNumberRight, CV_32SC1);
  mvMatchPairs.assign(mNumberMatches, std::pair<int, int>(0, 0));

  for (int GridType = 1; GridType <= 4; GridType++) {
    // initialize
    mMotionStatistics.setTo(0);
    mCellPairs.assign(mGridNumberLeft, -1);
    mNumberPointsInPerCellLeft.assign(mGridNumberLeft, 0);

    this->assignMatchPairs(GridType);
    this->verifyCellPairs(rotationType);

    // Mark inliers
    for (size_t i = 0; i < mNumberMatches; i++) {
      if (mCellPairs[mvMatchPairs[i].first] == mvMatchPairs[i].second) {
        mvbInlierMask[i] = true;
      }
    }
  }

  int num_inlier = cv::sum(mvbInlierMask)[0];
  return num_inlier;
}

std::vector<bool> GMSMatcher::getInlierMask(const bool withScale,
                                            const bool withRotation) {
  int max_inlier = 0;
  std::vector<bool> inliers;

  // With no rotation and no scale
  if (!withScale && !withRotation) {
    this->setScale(0);
    max_inlier = run(1);
    inliers = mvbInlierMask;
  }

  // With rotation and scale
  if (withRotation && withScale) {
    for (int Scale = 0; Scale < 5; Scale++) {
      this->setScale(Scale);
      for (int RotationType = 1; RotationType <= 8; RotationType++) {
        int num_inlier = run(RotationType);

        if (num_inlier > max_inlier) {
          inliers = mvbInlierMask;
          max_inlier = num_inlier;
        }
      }
    }
  }

  // With rotation
  if (withRotation && !withScale) {
    for (int RotationType = 1; RotationType <= 8; RotationType++) {
      int num_inlier = run(RotationType);

      if (num_inlier > max_inlier) {
        inliers = mvbInlierMask;
        max_inlier = num_inlier;
      }
    }
  }

  // With scale
  if (!withRotation && withScale) {
    for (int Scale = 0; Scale < 5; Scale++) {
      this->setScale(Scale);
      int num_inlier = run(1);

      if (num_inlier > max_inlier) {
        inliers = mvbInlierMask;
        max_inlier = num_inlier;
      }
    }
  }

  return inliers;
}

int GMSMatcher::match(const std::vector<cv::KeyPoint> &k1,
                      const cv::Mat &d1,
                      const std::vector<cv::KeyPoint> &k2,
                      const cv::Mat &d2,
                      const cv::Size &img_size,
                      std::vector<cv::DMatch> &matches) {
  // Match using Brute-force matcher (First pass)
  std::vector<cv::DMatch> matches_bf;

#ifdef USG_CUDA
  cv::GpuMat gd1(d1), gd2(d2);
  this->bf_matcher->match(gd1, gd2, matches_bf);
#else
  this->bf_matcher->match(d1, d2, matches_bf);
#endif

  // Initialize input
  this->mvP1 = this->normalizePoints(k1, img_size);
  this->mvP2 = this->normalizePoints(k2, img_size);
  this->mNumberMatches = matches_bf.size();
  this->convertMatches(matches_bf, this->mvMatches);

  // Initialize Grid
  this->mGridSizeLeft = cv::Size(20, 20);
  this->mGridNumberLeft =
      this->mGridSizeLeft.width * this->mGridSizeLeft.height;

  // Initialize the neihbor of left grid
  this->mGridNeighborLeft = cv::Mat::zeros(mGridNumberLeft, 9, CV_32SC1);
  this->initNeighbors(mGridNeighborLeft, mGridSizeLeft);

  // Matching using GMS matcher (Second pass)
  const std::vector<bool> inliers = this->getInlierMask(false, false);
  for (size_t i = 0; i < inliers.size(); i++) {
    if (inliers[i] == true) {
      matches.push_back(matches_bf[i]);
    }
  }

  return inliers.size();
}

} // namespace gvio
