/** * @file
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

/** Generic Camera **/
class Camera {
public:
  bool configured;
  bool initialized;

  CameraConfig config;
  std::vector<std::string> modes;
  std::map<std::string, CameraConfig> configs;

  cv::Mat image;
  double last_tic;

  cv::VideoCapture *capture;

  Camera();
  ~Camera();

  /**
   * Configure camera
   * @param config_path Path to config file (YAML)
   * @returns
   *    - 0 for success
   *    - -1 for failure
   */
  virtual int configure(const std::string &config_path);

  /**
   * Initialize camera
   * @returns
   *    - 0 for success
   *    - -1 for failure
   */
  virtual int initialize();

  /**
   * Shutdown camera
   * @returns
   *    - 0 for success
   *    - -1 for failure
   */
  virtual int shutdown();

  /**
   * Change camera mode
   * @returns
   *    - 0 for success
   *    - -1 for failure
   */
  virtual int changeMode(const std::string &mode);

  /**
   * ROI Image
   * @returns
   *    - 0 for success
   *    - -1 for failure
   */
  virtual int roiImage(cv::Mat &image);

  /**
   * Get camera frame
   *
   * @param image Camera frame image
   * @returns
   *    - 0 for success
   *    - -1 for failure
   */
  virtual int getFrame(cv::Mat &image);

  /**
   * Run camera
   *
   * Run Camera and attempt to detect any Apriltags. The AprilTags detected
   * will be recorded in `Camera::apriltags`.
   *
   * @returns
   *    - 0 for success
   *    - -1 for failure
   */
  int run();

  /**
   * Show FPS
   * @param last_tic Last tic in seconds
   * @param frame Frame number
   * @returns
   *    - 0 for success
   *    - -1 for failure
   */
  int showFPS(double &last_tic, int &frame);

  /**
   * Show image
   * @param image Image
   * @returns
   *    - 0 for success
   *    - -1 for failure
   */
  int showImage(cv::Mat &image);
};

/** @} group camera */
} // namespace gvio
#endif // GVIO_CAMERA_CAMERA_HPP
