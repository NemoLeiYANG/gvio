/**
 * @file
 * @defgroup camera camera
 */
#ifndef GVIO_CAMERA_CAMERA_HPP
#define GVIO_CAMERA_CAMERA_HPP

#include "gvio/util/util.hpp"
#include "gvio/camera/camera_config.hpp"

namespace gvio {
/**
 * @addtogroup camera
 * @{
 */

/**
 * Generic Camera
 */
class Camera {
public:
  bool configured = false;
  bool connected = false;

  CameraConfig config;
  std::vector<std::string> modes;
  std::map<std::string, CameraConfig> configs;

  cv::Mat image = cv::Mat(0, 0, CV_64F);
  double last_tic = 0.0;

  cv::VideoCapture *capture = nullptr;

  Camera() {
  }
  virtual ~Camera();

  /**
   * Configure camera
   *
   * @param config_path Path to config file (YAML)
   * @returns 0 for success, -1 for failure
   */
  virtual int configure(const std::string &config_path);

  /**
   * Connect camera
   *
   * @returns 0 for success, -1 for failure
   */
  virtual int connect();

  /**
   * Disconncet camera
   *
   * @returns 0 for success, -1 for failure
   */
  virtual int disconnect();

  /**
   * Change camera mode
   *
   * @returns 0 for success, -1 for failure
   */
  virtual int changeMode(const std::string &mode);

  /**
   * ROI Image

   * @returns 0 for success, -1 for failure
   */
  virtual int roiImage(cv::Mat &image);

  /**
   * Get camera frame
   *
   * @param image Camera frame image
   * @returns 0 for success, -1 for failure
   */
  virtual int getFrame(cv::Mat &image);

  /**
   * Run camera
   *
   * @returns 0 for success, -1 for failure
   */
  virtual int run();

  /**
   * Show FPS

   * @param last_tic Last tic in seconds
   * @param frame Frame number
   * @returns 0 for success, -1 for failure
   */
  virtual int showFPS(double &last_tic, int &frame);

  /**
   * Show image

   * @param image Image
   * @returns 0 for success, -1 for failure
   */
  virtual int showImage(cv::Mat &image);
};

/** @} group camera */
} // namespace gvio
#endif // GVIO_CAMERA_CAMERA_HPP
