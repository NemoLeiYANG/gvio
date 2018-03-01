#include "gvio/apriltag/mit.hpp"

namespace gvio {

int MITDetector::configure(const std::string &config_file) {
  if (BaseDetector::configure(config_file) != 0) {
    return -1;
  }

  // Tag detector
  this->detector = new AprilTags::TagDetector(AprilTags::tagCodes16h5);

  return 0;
}

int MITDetector::extractTags(cv::Mat &image, std::vector<TagPose> &tags) {
  // Change mode based on image size
  this->changeMode(image);

  // Tranform illumination invariant tag
  if (this->illum_invar) {
    this->illuminationInvariantTransform(image);
  }

  // Mask image if tag was last detected
  cv::Mat cropped_image;
  if (this->prev_tag.detected && this->windowing) {
    this->cropImage(this->prev_tag, image, cropped_image, this->window_padding);
  } else {
    this->crop_x = 0;
    this->crop_y = 0;
    this->crop_width = 0;
    this->crop_height = 0;
    cropped_image = image;
  }
  this->prev_tag.detected = false; // reset previous tag

  // Convert image to gray-scale
  cv::Mat image_gray;
  if (image.channels() == 3) {
    cv::cvtColor(cropped_image, image_gray, cv::COLOR_BGR2GRAY);
  } else {
    image_gray = cropped_image;
  }

  // Extract tags
  std::vector<AprilTags::TagDetection> detections;
  detections = this->detector->extractTags(image_gray);

  // Calculate tag pose
  for (size_t i = 0; i < detections.size(); i++) {
    TagPose tag_pose;
    tag_pose.id = detections[i].id;

    std::pair<float, float> p[4] = {
        detections[i].p[0],
        detections[i].p[1],
        detections[i].p[2],
        detections[i].p[3],
    };
    const Vec2 p1{p[0].first, p[0].second};
    const Vec2 p2{p[1].first, p[1].second};
    const Vec2 p3{p[2].first, p[2].second};
    const Vec2 p4{p[3].first, p[3].second};

    if (this->getRelativePose(p1, p2, p3, p4, tag_pose) == 0) {
      // Add to tags poses
      tags.push_back(tag_pose);

      // Keep track of last tag
      this->prev_tag = tag_pose;
      this->prev_tag_image_width = image.cols;
      this->prev_tag_image_height = image.rows;

      // Only need 1 tag
      break;
    }
  }

  // Imshow
  if (this->imshow && image_gray.rows && image_gray.cols) {
    cv::imshow("MITDetector", image_gray);
    cv::waitKey(1);
  }

  return 0;
}

} // namespace gvio
