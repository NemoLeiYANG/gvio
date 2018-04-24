#include "gvio/feature2d/grid_fast.hpp"

namespace gvio {

std::vector<cv::KeyPoint> SSC(std::vector<cv::KeyPoint> keyPoints,
                              int numRetPoints,
                              float tolerance,
                              int cols,
                              int rows) {
  // Several temp expression variables to simplify solution equation
  int exp1 = rows + cols + 2 * numRetPoints;
  long long exp2 =
      ((long long) 4 * cols + (long long) 4 * numRetPoints +
       (long long) 4 * rows * numRetPoints + (long long) rows * rows +
       (long long) cols * cols - (long long) 2 * rows * cols +
       (long long) 4 * rows * cols * numRetPoints);
  double exp3 = sqrt(exp2);
  double exp4 = (2 * (numRetPoints - 1));

  double sol1 = -round((exp1 + exp3) / exp4); // first solution
  double sol2 = -round((exp1 - exp3) / exp4); // second solution

  // Binary search range initialization with positive solution
  int high = (sol1 > sol2) ? sol1 : sol2;
  int low = floor(sqrt((double) keyPoints.size() / numRetPoints));

  int width;
  int prevWidth = -1;

  std::vector<int> ResultVec;
  bool complete = false;
  unsigned int K = numRetPoints;
  unsigned int Kmin = round(K - (K * tolerance));
  unsigned int Kmax = round(K + (K * tolerance));

  std::vector<int> result;
  result.reserve(keyPoints.size());
  while (!complete) {
    width = low + (high - low) / 2;

    // Needed to reassure the same radius is not repeated again
    if (width == prevWidth || low > high) {
      // Return the keypoints from the previous iteration
      ResultVec = result;
      break;
    }
    result.clear();
    double c = width / 2; // initializing Grid
    int numCellCols = floor(cols / c);
    int numCellRows = floor(rows / c);
    std::vector<std::vector<bool>> coveredVec(numCellRows + 1,
                                              std::vector<bool>(numCellCols + 1,
                                                                false));

    for (unsigned int i = 0; i < keyPoints.size(); ++i) {
      // Get position of the cell current point is located at
      int row = floor(keyPoints[i].pt.y / c);
      int col = floor(keyPoints[i].pt.x / c);

      // If the cell is not covered
      if (coveredVec[row][col] == false) {
        result.push_back(i);
        int rowMin = ((row - floor(width / c)) >= 0)
                         ? (row - floor(width / c))
                         : 0; // get range which current radius is covering
        int rowMax = ((row + floor(width / c)) <= numCellRows)
                         ? (row + floor(width / c))
                         : numCellRows;
        int colMin =
            ((col - floor(width / c)) >= 0) ? (col - floor(width / c)) : 0;
        int colMax = ((col + floor(width / c)) <= numCellCols)
                         ? (col + floor(width / c))
                         : numCellCols;
        for (int rowToCov = rowMin; rowToCov <= rowMax; ++rowToCov) {
          for (int colToCov = colMin; colToCov <= colMax; ++colToCov) {
            if (!coveredVec[rowToCov][colToCov])
              coveredVec[rowToCov][colToCov] = true; // cover cells within the
                                                     // square bounding box with
                                                     // width w
          }
        }
      }
    }

    if (result.size() >= Kmin && result.size() <= Kmax) { // solution found
      ResultVec = result;
      complete = true;
    } else if (result.size() < Kmin)
      high = width - 1; // update binary search range
    else
      low = width + 1;
    prevWidth = width;
  }
  // retrieve final keypoints
  std::vector<cv::KeyPoint> kp;
  for (unsigned int i = 0; i < ResultVec.size(); i++)
    kp.push_back(keyPoints[ResultVec[i]]);

  return kp;
}

std::vector<cv::KeyPoint> grid_fast(const cv::Mat &image,
                                    const int grid_rows,
                                    const int grid_cols,
                                    const double threshold,
                                    const bool nonmax_suppression,
                                    const bool debug) {
  // Prepare input image - make sure it is grayscale
  cv::Mat image_gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
  } else {
    image_gray = image.clone();
  }

  // Detect corners
  std::vector<cv::KeyPoint> keypoints;
  cv::FAST(image_gray, keypoints, threshold, nonmax_suppression);
  keypoints = sort_keypoints(keypoints);
  keypoints = SSC(keypoints, 1000, 0.1, image.cols, image.rows);

  // Debug
  if (debug) {
    // Draw corners
    for (auto kp : keypoints) {
      cv::circle(image, kp.pt, 2, cv::Scalar(0, 255, 0), -1);
    }

    // Draw vertical lines
    const int image_width = image.cols;
    const int image_height = image.rows;
    const int dx = image_width / grid_cols;
    const int dy = image_height / grid_rows;
    for (int x = dx; x < image_width; x += dx) {
      const cv::Point start(x, 0);
      const cv::Point end(x, image_height);
      const cv::Scalar color(0, 0, 255);
      cv::line(image, start, end, color, 2);
    }

    // Draw horizontal lines
    for (int y = dy; y < image_height; y += dy) {
      const cv::Point start(0, y);
      const cv::Point end(image_width, y);
      const cv::Scalar color(0, 0, 255);
      cv::line(image, start, end, color, 2);
    }

    // Draw
    cv::imshow("Grid Fast Debug", image);
    cv::waitKey(1);
  }

  return keypoints;
}

} // namespace gvio
