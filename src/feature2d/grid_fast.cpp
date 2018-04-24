#include "gvio/feature2d/grid_fast.hpp"

namespace gvio {

std::vector<cv::KeyPoint> grid_fast(const cv::Mat &image,
                                    const int max_corners,
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

  // Calculate number of grid cells and max corners per cell
  const int image_width = image.cols;
  const int image_height = image.rows;
  const int dx = image_width / grid_cols;
  const int dy = image_height / grid_rows;
  const int nb_cells = grid_rows * grid_cols;
  const size_t max_corners_per_cell = (float) max_corners / (float) nb_cells;

  // Detect corners in each grid cell
  std::vector<cv::KeyPoint> keypoints_all;
  for (int x = 0; x < image_width; x += dx) {
    for (int y = 0; y < image_height; y += dy) {
      // Make sure roi width and height are not out of bounds
      const double w = (x + dx > image_width) ? image_width - x : dx;
      const double h = (y + dy > image_height) ? image_height - y : dy;

      // Detect corners in grid cell
      cv::Rect roi = cv::Rect(x, y, w, h);
      std::vector<cv::KeyPoint> keypoints;
      cv::FAST(image_gray(roi), keypoints, threshold, nonmax_suppression);

      // Sort by keypoint response
      keypoints = sort_keypoints(keypoints);

      // Adjust keypoint's position according to the offset limit to max
      // corners per cell
      std::vector<cv::KeyPoint> keypoints_adjusted;
      for (auto &kp : keypoints) {
        keypoints_adjusted.emplace_back(kp.pt.x += x, kp.pt.y += y, kp.size);
        if (keypoints_adjusted.size() == max_corners_per_cell) {
          break;
        }
      }

      // Add to total keypoints detected
      keypoints_all.insert(std::end(keypoints_all),
                           std::begin(keypoints_adjusted),
                           std::end(keypoints_adjusted));
    }
  }

  // Debug
  if (debug) {
    // Draw corners
    for (auto kp : keypoints_all) {
      cv::circle(image, kp.pt, 2, cv::Scalar(0, 255, 0), -1);
    }

    // Draw vertical lines
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
    cv::imshow("Grid Fast", image);
    cv::waitKey(1);
  }

  return keypoints_all;
}

} // namespace gvio
