#include "gvio/gimbal/calibration/calib_preprocessor.hpp"

namespace gvio {

cv::Mat CameraProperties::K() {
  cv::Mat K = cv::Mat::zeros(3, 3, CV_64F);
  K.at<double>(0, 0) = this->intrinsics[0];
  K.at<double>(1, 1) = this->intrinsics[1];
  K.at<double>(0, 2) = this->intrinsics[2];
  K.at<double>(1, 2) = this->intrinsics[3];
  return K;
}

cv::Mat CameraProperties::D() {
  cv::Mat D = cv::Mat::zeros(4, 1, CV_64F);
  D.at<double>(0, 0) = this->distortion_coeffs[0];
  D.at<double>(1, 0) = this->distortion_coeffs[1];
  D.at<double>(2, 0) = this->distortion_coeffs[2];
  D.at<double>(3, 0) = this->distortion_coeffs[3];
  return D;
}

std::ostream &operator<<(std::ostream &os, const CameraProperties &cam) {
  os << "Camera ID: " << cam.camera_id << std::endl;
  os << "Camera model: " << cam.camera_model << std::endl;
  os << "Intrinsics: " << cam.intrinsics.transpose() << std::endl;
  os << "Distortion model: " << cam.distortion_model << std::endl;
  os << "Distortion coeffs: " << cam.distortion_coeffs.transpose() << std::endl;
  os << "Resolution: " << cam.resolution.transpose() << std::endl;
  return os;
}

std::ostream &operator<<(std::ostream &os, const CalibTarget &target) {
  os << "Target type: " << target.type << std::endl;
  os << "Rows: " << target.rows << std::endl;
  os << "Cols: " << target.cols << std::endl;
  os << "Square_size: " << target.square_size << std::endl;
  os << "Spacing: " << target.spacing << std::endl;
  return os;
}

CalibPreprocessor::CalibPreprocessor() {}

CalibPreprocessor::~CalibPreprocessor() {}

int CalibPreprocessor::loadTargetFile(const std::string &target_file) {
  ConfigParser target_parser;
  target_parser.addParam("target_type", &this->target.type);
  target_parser.addParam("tagCols", &this->target.cols);
  target_parser.addParam("tagRows", &this->target.rows);
  target_parser.addParam("tagSize", &this->target.square_size);
  target_parser.addParam("tagSpacing", &this->target.spacing);
  if (target_parser.load(target_file) != 0) {
    return -1;
  }

  return 0;
}

int CalibPreprocessor::loadJointFile(const std::string &joint_file) {
  if (csv2mat(joint_file, false, this->joint_data) != 0) {
    return -1;
  }
  this->nb_measurements = this->joint_data.rows();

  return 0;
}

int CalibPreprocessor::loadCamchainFile(const std::string &camchain_file) {
  ConfigParser camchain_parser;
  CameraProperties cam0, cam1, cam2;

  // Camera 0
  cam0.camera_id = 0;
  camchain_parser.addParam("cam0.camera_model", &cam0.camera_model);
  camchain_parser.addParam("cam0.intrinsics", &cam0.intrinsics);
  camchain_parser.addParam("cam0.distortion_model", &cam0.distortion_model);
  camchain_parser.addParam("cam0.distortion_coeffs", &cam0.distortion_coeffs);
  camchain_parser.addParam("cam0.resolution", &cam0.resolution);

  // Camera 1
  cam1.camera_id = 1;
  camchain_parser.addParam("cam1.camera_model", &cam1.camera_model);
  camchain_parser.addParam("cam1.intrinsics", &cam1.intrinsics);
  camchain_parser.addParam("cam1.distortion_model", &cam1.distortion_model);
  camchain_parser.addParam("cam1.distortion_coeffs", &cam1.distortion_coeffs);
  camchain_parser.addParam("cam1.resolution", &cam1.resolution);

  // Camera 2
  cam2.camera_id = 2;
  camchain_parser.addParam("cam2.camera_model", &cam2.camera_model);
  camchain_parser.addParam("cam2.intrinsics", &cam2.intrinsics);
  camchain_parser.addParam("cam2.distortion_model", &cam2.distortion_model);
  camchain_parser.addParam("cam2.distortion_coeffs", &cam2.distortion_coeffs);
  camchain_parser.addParam("cam2.resolution", &cam2.resolution);

  // Parse camchain file
  if (camchain_parser.load(camchain_file) != 0) {
    return -1;
  }
  this->camera_properties = {cam0, cam1, cam2};

  return 0;
}

cv::Mat CalibPreprocessor::undistortImage(const cv::Mat &K,
                                          const cv::Mat &D,
                                          const cv::Mat &image,
                                          cv::Mat &Knew) {
  const cv::Size img_size = {image.cols, image.rows};
  const cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
  const double balance = 0.0;

  // Estimate new camera matrix first
  cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K,
                                                          D,
                                                          img_size,
                                                          R,
                                                          Knew,
                                                          balance);

  // Undistort image
  cv::Mat image_ud;
  cv::fisheye::undistortImage(image, image_ud, K, D, Knew);

  return image_ud;
}

cv::Mat CalibPreprocessor::undistortImage(const cv::Mat &K,
                                          const cv::Mat &D,
                                          const cv::Mat &image) {
  cv::Mat Knew;
  return this->undistortImage(K, D, image, Knew);
}

int CalibPreprocessor::findImageFiles(const std::string &search_path,
                                      std::vector<std::string> &image_files) {
  if (list_dir(search_path, image_files) != 0) {
    return -1;
  }
  std::sort(image_files.begin(), image_files.end());

  return 0;
}

std::vector<int> CalibPreprocessor::findCommonTags(
    const std::map<int, std::vector<Vec2>> &tags0,
    const std::map<int, std::vector<Vec2>> &tags1,
    const std::map<int, std::vector<Vec2>> &tags2) {
  std::vector<int> common_tags;

  // Find common tags between the maps
  for (auto &tag : tags0) {
    const int tag_id = tag.first;
    const int in_tags0 = tags0.count(tag_id);
    const int in_tags1 = tags1.count(tag_id);
    const int in_tags2 = tags2.count(tag_id);

    if (in_tags0 && in_tags1 && in_tags2) {
      common_tags.push_back(tag.first);
    }
  }

  return common_tags;
}

int CalibPreprocessor::preprocess(const std::string &dir_path) {
  // Load target file
  const std::string target_file = dir_path + "/target.yaml";
  if (this->loadTargetFile(target_file) != 0) {
    LOG_ERROR("Failed to load target data [%s]!", target_file.c_str());
    return -1;
  }

  // Load joint data
  const std::string joint_file = dir_path + "/joint.csv";
  if (this->loadJointFile(joint_file) != 0) {
    LOG_ERROR("Failed to load joint data [%s]!", joint_file.c_str());
    return -1;
  }

  // Load camchain file
  const std::string camchain_file = dir_path + "/camchain.yaml";
  if (this->loadCamchainFile(camchain_file) != 0) {
    LOG_ERROR("Failed to load camchain data [%s]!", camchain_file.c_str());
    return -1;
  }

  // Get image files for cam0, cam1, cam2
  // -- Get image files for cam0
  const std::string cam0_path = dir_path + "/cam0";
  std::vector<std::string> cam0_files;
  if (this->findImageFiles(cam0_path, cam0_files) != 0) {
    LOG_ERROR("Failed to walk through [%s]!", cam0_path.c_str());
    return -1;
  }
  // -- Get image files for cam1
  const std::string cam1_path = dir_path + "/cam1";
  std::vector<std::string> cam1_files;
  if (this->findImageFiles(cam1_path, cam1_files) != 0) {
    LOG_ERROR("Failed to walk through [%s]!", cam1_path.c_str());
    return -1;
  }
  // -- Get image files for cam2
  const std::string cam2_path = dir_path + "/cam2";
  std::vector<std::string> cam2_files;
  if (this->findImageFiles(cam2_path, cam2_files) != 0) {
    LOG_ERROR("Failed to walk through [%s]!", cam2_path.c_str());
    return -1;
  }

  // Iterate through images
  AprilGrid grid{this->target.rows,
                 this->target.cols,
                 this->target.square_size,
                 this->target.spacing};

  for (int i = 0; i < this->nb_measurements; i++) {
    // Load images
    cv::Mat img0 = cv::imread(cam0_path + "/" + cam0_files[i]);
    cv::Mat img1 = cv::imread(cam1_path + "/" + cam1_files[i]);
    cv::Mat img2 = cv::imread(cam2_path + "/" + cam2_files[i]);

    // Undistort image 0
    const cv::Mat K0 = this->camera_properties[0].K();
    const cv::Mat D0 = this->camera_properties[0].D();
    cv::Mat img0_ud = this->undistortImage(K0, D0, img0);

    // Undistort image 1
    const cv::Mat K1 = this->camera_properties[1].K();
    const cv::Mat D1 = this->camera_properties[1].D();
    cv::Mat img1_ud = this->undistortImage(K1, D1, img1);

    // Undistort image 2
    const cv::Mat K2 = this->camera_properties[2].K();
    const cv::Mat D2 = this->camera_properties[2].D();
    cv::Mat img2_ud = this->undistortImage(K2, D2, img2);

    // Extract tags
    std::map<int, std::vector<Vec2>> tags0;
    std::map<int, std::vector<Vec2>> tags1;
    std::map<int, std::vector<Vec2>> tags2;
    grid.extractTags(img0_ud, tags0);
    grid.extractTags(img1_ud, tags1);
    grid.extractTags(img2_ud, tags2);

    // Find common tags between the images
    std::vector<int> common_ids = this->findCommonTags(tags0, tags1, tags2);
    // -- Estimated 3d position of tag corners
    MatX P_c0 = zeros(common_ids.size() * 4, 3);
    MatX P_c1 = zeros(common_ids.size() * 4, 3);
    MatX P_c2 = zeros(common_ids.size() * 4, 3);
    // -- Observed 2d pixel measurements of tag corners
    MatX Q_c0 = zeros(common_ids.size() * 4, 2);
    MatX Q_c1 = zeros(common_ids.size() * 4, 2);
    MatX Q_c2 = zeros(common_ids.size() * 4, 2);

    int index = 0;
    for (auto &id : common_ids) {
      for (int i = 0; i < 4; i++) {
        Q_c0.block(index, 0, 1, 2) = tags0[id][i].transpose();
        Q_c1.block(index, 0, 1, 2) = tags1[id][i].transpose();
        Q_c2.block(index, 0, 1, 2) = tags2[id][i].transpose();
        index++;
      }
    }

    // // Visualize detected AprilTags
    // cv::imshow("img0", img0_ud);
    // cv::imshow("img1", img1_ud);
    // cv::imshow("img2", img2_ud);
    //
    // cv::waitKey();
  }

  return 0;
}

} // namespace gvio
