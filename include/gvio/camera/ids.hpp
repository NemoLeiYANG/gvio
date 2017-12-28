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

enum class CaptureMode { NOT_SET, FREE_RUN, SOFTWARE_TRIGGER, INVALID };

/**
 * String to capture mode
 *
 * @param mode Capture mode in string form
 * @returns Capture mode as enum
 */
enum CaptureMode ueye_str2capturemode(const std::string &mode);

/**
 * String to color mode
 *
 * @param mode Color mode in string form
 * @returns Color mode, else -1 for failure
 */
int ueye_str2colormode(const std::string &mode);

/**
 * Color mode to bytes per pixel
 *
 * @param mode Color mode
 * @returns 0 for success, -1 for failure
 */
int ueye_colormode2bpp(int mode);

/**
 * Color mode to number of channels
 *
 * @param mode Color mode
 * @returns 0 for success, -1 for failure
 */
int ueye_colormode2channels(int mode);

/**
 * List uEye cameras detected
 */
void ueye_list_cameras();

/**
 * Print uEye camera error
 *
 * @param handle Camera handle
 */
void ueye_print_error(const HIDS &handle);

/**
  * Print sensor information
  *
  * @param info Sensor info
  */
void ueye_print_sensor_info(const SENSORINFO &info);

/**
  * Print camera information
  *
  * @param info Camera info
  */
void ueye_print_camera_info(const CAMINFO &info);

/**
  * Convert raw image data to `cv::Mat`
  *
  * @param image_width Image width
  * @param image_height Image height
  * @returns 0 for success, -1 for failure
  */
int raw2cvmat(void *image_data,
              const int image_width,
              const int image_height,
              const int channels,
              const int bpp,
              cv::Mat &output);

/** IDS Camera **/
class IDSCamera {
public:
  bool configured = false;

  // Camera and sensor info
  CAMINFO camera_info;
  SENSORINFO sensor_info;

  // Camera settings
  HIDS cam_handle = 0;
  enum CaptureMode capture_mode = CaptureMode::NOT_SET;

  // Image settings
  int image_width = 0;
  int image_height = 0;
  int offset_x = 0;
  int offset_y = 0;
  int color_mode = IS_CM_MONO8;

  // Capture settings
  int pixel_clock = 0;
  double frame_rate = 0.0;
  int gain = 0.0;

  // Buffers
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
   * @param clock_rate Pixel clock rate (MHz)
   * @returns 0 for success, -1 for failure
   */
  int setPixelClock(const int clock_rate);

  /**
   * Get pixel clock
   *
   * @param clock_rate Pixel clock rate (MHz)
   * @returns 0 for success, -1 for failure
   */
  int getPixelClock(int &clock_rate);

  /**
   * Set color mode
   *
   * @param color_mode Color mode
   * @returns 0 for success, -1 for failure
   */
  int setColorMode(const int color_mode);

  /**
   * Get color mode
   *
   * @param color_mode Color mode
   * @returns 0 for success, -1 for failure
   */
  int getColorMode(int &color_mode);

  /**
   * Set frame rate
   *
   * @param frame_rate Frame rate (Hz)
   * @returns 0 for success, -1 for failure
   */
  int setFrameRate(const double frame_rate);

  /**
   * Get frame rate
   *
   * @param frame_rate Frame rate (Hz)
   * @returns 0 for success, -1 for failure
   */
  int getFrameRate(double &frame_rate);

  /**
   * Set gain
   *
   * @returns 0 for success, -1 for failure
   */
  int setGain(const int gain);

  /**
   * Get gain
   *
   * @returns 0 for success, -1 for failure
   */
  int getGain(int &gain);

  /**
   * Set ROI
   *
   * @param offset_x Offset in x-axis
   * @param offset_y Offset in y-axis
   * @param image_width Image width
   * @param image_height Image height
   * @returns 0 for success, -1 for failure
   */
  int setROI(const int offset_x,
             const int offset_y,
             const int image_width,
             const int image_height);

  /**
   * Get ROI
   *
   * @param offset_x Offset in x-axis
   * @param offset_y Offset in y-axis
   * @param image_width Image width
   * @param image_height Image height
   * @returns 0 for success, -1 for failure
   */
  int getROI(int &offset_x, int &offset_y, int &image_width, int &image_height);

  /**
   * Get image size in pixels
   *
   * @param image_width Image width
   * @param image_height Image height
   * @returns 0 for success, -1 for failure
   */
  int getImageSize(int &image_width, int &image_height);

  /**
   * Set HDR mode
   *
   * @returns 0 for success, -1 for failure
   */
  int setHDRMode();

  /**
   * Get HDR mode
   *
   * @returns 0 for success, -1 for failure
   */
  int getHDRMode();

  /**
   * Get camera frame
   *
   * @param image Camera frame image
   * @returns 0 for success, -1 for failure
   */
  int getFrame(cv::Mat &image);
};

/** @} group camera */
} // namespace gvio
#endif // GVIO_CAMERA_IDS_HPP
