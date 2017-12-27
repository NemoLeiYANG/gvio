/** * @file
 * @defgroup camera camera
 */
#ifndef GVIO_CAMERA_IDS_HPP
#define GVIO_CAMERA_IDS_HPP

#include <ueye.h>

#include "gvio/camera/camera.hpp"

namespace gvio {
/**
 * @addtogroup camera
 * @{
 */

enum class CaptureMode { NOT_SET, FREE_RUN, SOFTWARE_TRIGGER };

static inline void ueye_list_cameras() {
  int nb_cameras = -1;
  int retval = 0;
  if ((retval = is_GetNumberOfCameras(&nb_cameras)) != IS_SUCCESS) {
    LOG_ERROR("Failed to get number of connected UEye cameras!");

  } else if (nb_cameras < 1) {
    LOG_ERROR("No UEye cameras are connected!");
    LOG_ERROR("Hint: is the IDS daemon (/etc/init.d/ueyeusbdrc) is running?");
  }
  LOG_INFO("Number of cameras %d", nb_cameras);
}

static inline void ueye_print_error(const HIDS &handle) {
  char *err_msg = (char *) malloc(sizeof(char) * 200);
  memset(err_msg, 0, 200);
  int err_code = 0;

  is_GetError(handle, &err_code, &err_msg);
  LOG_ERROR("Error[%d]: %s\n", err_code, err_msg);

  free(err_msg);
}

/** IDS Camera **/
class IDSCamera {
public:
  bool configured = false;
  bool connected = false;

  SENSORINFO sensor_info;
  CAMINFO camera_info;

  HIDS cam_handle = 0;
  enum CaptureMode capture_mode = CaptureMode::NOT_SET;

  std::vector<char *> buffers;
  std::vector<int> buffer_id;

  IDSCamera() {}
  ~IDSCamera();

  /**
   * Configure
   *
   * @param Path to config file
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Allocate buffers
   *
   * @param nb_buffers Number of buffers
   * @param bpp Bytes per pixel
   * @returns 0 for success, -1 for failure
   */
  int allocBuffers(const int nb_buffers, const int bpp);

  /**
   * Free buffers
   *
   * @returns 0 for success, -1 for failure
   */
  int freeBuffers();

  /**
   * Set capture mode
   *
   * @param capture_mode
   * @returns 0 for success, -1 for failure
   */
  int setCaptureMode(const enum CaptureMode &capture_mode);

  /**
   * Get capture mode
   *
   * @param capture_mode Capture mode
   * @returns 0 for success, -1 for failure
   */
  int getCaptureMode(enum CaptureMode &capture_mode);

  /**
   * Set pixel clock
   *
   * @returns 0 for success, -1 for failure
   */
  int setPixelClock();

  /**
   * Get pixel clock
   *
   * @returns 0 for success, -1 for failure
   */
  int getPixelClock();

  /**
   * Set gain boost
   *
   * @returns 0 for success, -1 for failure
   */
  int setGainBoost();

  /**
   * Get gain boost
   *
   * @returns 0 for success, -1 for failure
   */
  int getGainBoost();

  /**
   * Set resolution
   *
   * @param image_width Image width
   * @param image_height Image height
   * @returns 0 for success, -1 for failure
   */
  int setResolution(const int &width, const int &height);

  /**
   * Get resolution
   *
   * @param image_width Image width
   * @param image_height Image height
   * @returns 0 for success, -1 for failure
   */
  int getResolution(int &width, int &height);

  /**
   * Get camera frame
   *
   * @param image Camera frame image
   * @returns 0 for success, -1 for failure
   */
  int getFrame(cv::Mat &image);

  /**
   * Print sensor information
   *
   * @param info Sensor info
   */
  static void printSensorInfo(const SENSORINFO &info);

  /**
   * Print camera information
   *
   * @param info Camera info
   */
  static void printCameraInfo(const CAMINFO &info);
};

/** @} group camera */
} // namespace gvio
#endif // GVIO_CAMERA_IDS_HPP
