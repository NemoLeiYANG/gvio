#include "gvio/feature2d/gms_matcher.hpp"

namespace gvio {

GMSMatcher::GMSMatcher() {
#ifdef USG_CUDA
  bf_matcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
#else
  bf_matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
#endif
}

GMSMatcher::~GMSMatcher() {}

std::vector<cv::Point2f> GMSMatcher::normalizePoints(
    const std::vector<cv::KeyPoint> &kp, const cv::Size &size) {
  std::vector<cv::Point2f> npts;
  for (size_t i = 0; i < kp.size(); i++) {
    npts.emplace_back(kp[i].pt.x / size.width, kp[i].pt.y / size.height);
  }

  return npts;
}

std::vector<cv::Point2f> GMSMatcher::normalizePoints(
    const std::vector<cv::Point2f> &kp, const cv::Size &size) {
  std::vector<cv::Point2f> npts;
  for (size_t i = 0; i < kp.size(); i++) {
    npts.emplace_back(kp[i].x / size.width, kp[i].y / size.height);
  }

  return npts;
}

void GMSMatcher::convertMatches(const std::vector<cv::DMatch> &vDMatches,
                                std::vector<std::pair<int, int>> &vMatches) {
  vMatches.resize(this->nb_matches);
  for (size_t i = 0; i < this->nb_matches; i++) {
    vMatches[i] =
        std::pair<int, int>(vDMatches[i].queryIdx, vDMatches[i].trainIdx);
  }
}

int GMSMatcher::getGridIndexLeft(const cv::Point2f &pt, const int type) {
  int x = 0, y = 0;

  if (type == 1) {
    x = floor(pt.x * this->grid_size_left.width);
    y = floor(pt.y * this->grid_size_left.height);
  }

  if (type == 2) {
    x = floor(pt.x * this->grid_size_left.width + 0.5);
    y = floor(pt.y * this->grid_size_left.height);
  }

  if (type == 3) {
    x = floor(pt.x * this->grid_size_left.width);
    y = floor(pt.y * this->grid_size_left.height + 0.5);
  }

  if (type == 4) {
    x = floor(pt.x * this->grid_size_left.width + 0.5);
    y = floor(pt.y * this->grid_size_left.height + 0.5);
  }

  if (x >= this->grid_size_left.width || y >= this->grid_size_left.height) {
    return -1;
  }

  return x + y * this->grid_size_left.width;
}

int GMSMatcher::getGridIndexRight(const cv::Point2f &pt) {
  const int x = floor(pt.x * this->grid_size_right.width);
  const int y = floor(pt.y * this->grid_size_right.height);
  return x + y * this->grid_size_right.width;
}

void GMSMatcher::assignMatchPairs(const int grid_type) {
  for (size_t i = 0; i < this->nb_matches; i++) {
    const cv::Point2f &lp = this->points1[this->matches[i].first];
    const cv::Point2f &rp = this->points2[this->matches[i].second];

    const int lgidx = this->match_pairs[i].first =
        getGridIndexLeft(lp, grid_type);
    int rgidx = -1;

    if (grid_type == 1) {
      rgidx = this->match_pairs[i].second = this->getGridIndexRight(rp);
    } else {
      rgidx = this->match_pairs[i].second;
    }

    if (lgidx < 0 || rgidx < 0)
      continue;

    this->motion_statistics.at<int>(lgidx, rgidx)++;
    this->nb_points_per_cell[lgidx]++;
  }
}

void GMSMatcher::verifyCellPairs(const int rotation_type) {
  const int *current_rp = rotation_patterns[rotation_type - 1];

  for (int i = 0; i < this->grid_number_left; i++) {
    if (cv::sum(this->motion_statistics.row(i))[0] == 0) {
      this->cell_pairs[i] = -1;
      continue;
    }

    int max_number = 0;
    for (int j = 0; j < this->grid_number_right; j++) {
      int *value = this->motion_statistics.ptr<int>(i);
      if (value[j] > max_number) {
        this->cell_pairs[i] = j;
        max_number = value[j];
      }
    }

    int idx_grid_rt = this->cell_pairs[i];
    const int *NB9_lt = this->grid_neighbor_left.ptr<int>(i);
    const int *NB9_rt = this->grid_neighbor_right.ptr<int>(idx_grid_rt);

    int score = 0;
    double thresh = 0;
    int numpair = 0;

    for (size_t j = 0; j < 9; j++) {
      int ll = NB9_lt[j];
      int rr = NB9_rt[current_rp[j] - 1];
      if (ll == -1 || rr == -1)
        continue;

      score += this->motion_statistics.at<int>(ll, rr);
      thresh += this->nb_points_per_cell[ll];
      numpair++;
    }

    thresh = this->threshold_factor * sqrt(thresh / numpair);

    if (score < thresh) {
      this->cell_pairs[i] = -2;
    }
  }
}

std::vector<int> GMSMatcher::getNB9(const int idx, const cv::Size &grid_size) {
  std::vector<int> NB9(9, -1);
  const int idx_x = idx % grid_size.width;
  const int idx_y = idx / grid_size.width;

  for (int yi = -1; yi <= 1; yi++) {
    for (int xi = -1; xi <= 1; xi++) {
      int idx_xx = idx_x + xi;
      int idx_yy = idx_y + yi;

      if (idx_xx < 0 || idx_xx >= grid_size.width || idx_yy < 0 ||
          idx_yy >= grid_size.height)
        continue;

      NB9[xi + 4 + yi * 3] = idx_xx + idx_yy * grid_size.width;
    }
  }

  return NB9;
}

void GMSMatcher::initNeighbors(cv::Mat &neighbor, const cv::Size &grid_size) {
  for (int i = 0; i < neighbor.rows; i++) {
    std::vector<int> NB9 = this->getNB9(i, grid_size);
    int *data = neighbor.ptr<int>(i);
    memcpy(data, &NB9[0], sizeof(int) * 9);
  }
}

void GMSMatcher::setScale(const int scale) {
  // Set scale
  this->grid_size_right.width =
      this->grid_size_left.width * this->scale_ratios[scale];
  this->grid_size_right.height =
      this->grid_size_left.height * this->scale_ratios[scale];
  this->grid_number_right =
      this->grid_size_right.width * this->grid_size_right.height;

  // Initialize the neihbor of right grid
  this->grid_neighbor_right =
      cv::Mat::zeros(this->grid_number_right, 9, CV_32SC1);
  this->initNeighbors(this->grid_neighbor_right, this->grid_size_right);
}

int GMSMatcher::run(const int rotation_type) {
  this->inliers_mask.assign(this->nb_matches, false);

  // Initialize Motion Statisctics
  this->motion_statistics =
      cv::Mat::zeros(this->grid_number_left, this->grid_number_right, CV_32SC1);
  this->match_pairs.assign(this->nb_matches, std::pair<int, int>(0, 0));

  for (int grid_type = 1; grid_type <= 4; grid_type++) {
    // initialize
    this->motion_statistics.setTo(0);
    this->cell_pairs.assign(this->grid_number_left, -1);
    this->nb_points_per_cell.assign(this->grid_number_left, 0);

    this->assignMatchPairs(grid_type);
    this->verifyCellPairs(rotation_type);

    // Mark inliers
    for (size_t i = 0; i < this->nb_matches; i++) {
      if (this->cell_pairs[this->match_pairs[i].first] ==
          this->match_pairs[i].second) {
        this->inliers_mask[i] = true;
      }
    }
  }

  const int num_inlier = cv::sum(this->inliers_mask)[0];
  return num_inlier;
}

std::vector<bool> GMSMatcher::getInlierMask(const bool with_scale,
                                            const bool with_rotation) {
  int max_inlier = 0;
  std::vector<bool> inliers;

  // With no rotation and no scale
  if (!with_scale && !with_rotation) {
    this->setScale(0);
    max_inlier = run(1);
    inliers = this->inliers_mask;
  }

  // With rotation and scale
  if (with_rotation && with_scale) {
    for (int scale = 0; scale < 5; scale++) {
      this->setScale(scale);
      for (int rotation_type = 1; rotation_type <= 8; rotation_type++) {
        int num_inlier = run(rotation_type);

        if (num_inlier > max_inlier) {
          inliers = this->inliers_mask;
          max_inlier = num_inlier;
        }
      }
    }
  }

  // With rotation
  if (with_rotation && !with_scale) {
    for (int rotation_type = 1; rotation_type <= 8; rotation_type++) {
      int num_inlier = run(rotation_type);

      if (num_inlier > max_inlier) {
        inliers = this->inliers_mask;
        max_inlier = num_inlier;
      }
    }
  }

  // With scale
  if (!with_rotation && with_scale) {
    for (int scale = 0; scale < 5; scale++) {
      this->setScale(scale);
      int num_inlier = run(1);

      if (num_inlier > max_inlier) {
        inliers = this->inliers_mask;
        max_inlier = num_inlier;
      }
    }
  }

  return inliers;
}

int GMSMatcher::match(const std::vector<cv::KeyPoint> &k1,
                      const cv::Mat &desc1,
                      const std::vector<cv::KeyPoint> &k2,
                      const cv::Mat &desc2,
                      const cv::Size &img_size,
                      std::vector<cv::DMatch> &matches) {
  assert(img_size.width != 0 && img_size.height != 0);

  // Match using Brute-force matcher (First pass)
  std::vector<cv::DMatch> matches_bf;

#ifdef USG_CUDA
  cv::GpuMat gd1(desc1), gd2(desc2);
  this->bf_matcher->match(gd1, gd2, matches_bf);
#else
  this->bf_matcher->match(desc1, desc2, matches_bf);
#endif

  // Initialize input
  this->points1 = this->normalizePoints(k1, img_size);
  this->points2 = this->normalizePoints(k2, img_size);
  this->nb_matches = matches_bf.size();
  this->convertMatches(matches_bf, this->matches);

  // Initialize Grid
  this->grid_size_left = cv::Size(20, 20);
  this->grid_number_left =
      this->grid_size_left.width * this->grid_size_left.height;

  // Initialize the neihbor of left grid
  this->grid_neighbor_left =
      cv::Mat::zeros(this->grid_number_left, 9, CV_32SC1);
  this->initNeighbors(this->grid_neighbor_left, this->grid_size_left);

  // Matching using GMS matcher (Second pass)
  const std::vector<bool> inliers = this->getInlierMask(false, false);
  for (size_t i = 0; i < inliers.size(); i++) {
    if (inliers[i] == true) {
      matches.push_back(matches_bf[i]);
    }
  }

  return inliers.size();
}

int GMSMatcher::match(const std::vector<cv::Point2f> &kp1,
                      const std::vector<cv::Point2f> &kp2,
                      const cv::Size &img_size,
                      std::vector<bool> &matches) {
  assert(kp1.size() == kp2.size());

  // RANSAC
  std::vector<uchar> ransac_mask;
  cv::findFundamentalMat(kp1, kp2, cv::FM_RANSAC, 0, 0.9999, ransac_mask);

  // Initialize input
  this->points1 = this->normalizePoints(kp1, img_size);
  this->points2 = this->normalizePoints(kp2, img_size);
  this->nb_matches = kp1.size();

  // GMS matcher normally expects keypoints with descriptors, this method
  // however caters for keypoints that are already matched and kp1 kp2 are
  // already matched 1-to-1
  for (size_t i = 0; i < this->nb_matches; i++) {
    this->matches.push_back({i, i});
  }

  // Initialize Grid
  this->grid_size_left = cv::Size(20, 20);
  this->grid_number_left =
      this->grid_size_left.width * this->grid_size_left.height;

  // Initialize the neihbor of left grid
  this->grid_neighbor_left =
      cv::Mat::zeros(this->grid_number_left, 9, CV_32SC1);
  this->initNeighbors(this->grid_neighbor_left, this->grid_size_left);

  // Matching using GMS matcher (Second pass)
  const std::vector<bool> inliers = this->getInlierMask(false, false);
  for (size_t i = 0; i < inliers.size(); i++) {
    matches.push_back(inliers[i]);
  }

  return inliers.size();
}

} // namespace gvio
