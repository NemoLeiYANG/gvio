/**
 * @file
 * @ingroup camera
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

/**
 * Trigger Modes
 */
enum class TriggerMode {
  /** Not set **/
  NOT_SET,

  /** Free run (whenever the camera is ready) **/
  FREE_RUN,

  /** Software trigger **/
  SOFTWARE_TRIGGER,

  /** Hardware trigger (falling signal edge). Enables camera memory mode in
   * post-trigger **/
  TRIGGER_HI_LO,

  /** Hardware trigger (rising signal edge). Enables camera memory mode in
   * post-trigger. **/
  TRIGGER_LO_HI,

  /** Hardware trigger (falling signal edge). Enables the pre-trigger in the
   * memory mode **/
  TRIGGER_PRE_HI_LO,

  /** Hardware trigger (rising signal edge). Enables the pre-trigger in the
   * memory mode **/
  TRIGGER_PRE_LO_HI,

  /** Freerun synchronization / hardware trigger (falling signal edge). The
   * freerun synchronization mode is currently only supported by the sensors of
   * the UI-146x/UI-546x series **/
  TRIGGER_HI_LO_SYNC,

  /** Freerun synchronization / hardware trigger (rising signal edge). The
   * freerun synchronization mode is currently only supported by the sensors of
   * the UI-146x/UI-546x series **/
  TRIGGER_LO_HI_SYNC,

  /** Invalid capture mode **/
  INVALID
};

/**
 * String to capture mode
 *
 * @param mode Trigger mode in string form
 * @returns Trigger mode as enum
 */
enum TriggerMode ueye_str2capturemode(const std::string &mode);

/**
 * String to color mode
 *
 * @param mode Color mode in string form
 * @returns Color mode, else -1 for failure
 */
int ueye_str2colormode(const std::string &mode);

/**
 * Color mode to bits per pixel (bpp)
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
  * @param image_data Input image data
  * @param image_width Image width
  * @param image_height Image height
  * @param channels Image channels
  * @param bpp Bits per pixel
  * @param output Output image in cv::Mat
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
  int camera_index = -1;
  HIDS camera_handle = 0;
  std::string trigger_mode = "NOT_SET";

  // Image settings
  int image_width = 0;
  int image_height = 0;
  int offset_x = 0;
  int offset_y = 0;
  std::string color_mode = "MONO8";

  // Capture settings
  int pixel_clock = 0;
  double frame_rate = 0.0;
  int gain = 0.0;

  // Buffers
  int nb_buffers = 0;
  std::vector<char *> buffers;
  std::vector<int> buffer_id;

  IDSCamera() {
  }
  ~IDSCamera();

  /**
   * Configure
   *
   * @param config_file Path to config file
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Allocate buffers
   *
   * @param nb_buffers Number of buffers
   * @param bpp Bits per pixel (bpp)
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
   * @param trigger_mode
   * @returns 0 for success, -1 for failure
   */
  int setTriggerMode(const std::string &trigger_mode);

  /**
   * Get capture mode
   *
   * @param trigger_mode Trigger mode
   * @returns 0 for success, -1 for failure
   */
  int getTriggerMode(std::string &trigger_mode);

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
   * Supported color modes:
   *
   * - RAW8
   * - MONO8
   * - RAW10
   * - RAW12
   * - RAW16
   * - MONO10
   * - MONO12
   * - MONO12
   * - BGR5
   * - BGR565
   * - UYVY
   * - UYVY_MONO
   * - UYVY_BAYER
   * - CBYCRY_PACKED
   * - RGB8_PACKED
   * - BGR8_PACKED
   * - RGB8_PLANAR
   * - RGBA8_PACKED
   * - BGRA8_PACKED
   * - RGBY8_PACKED
   * - BGRY8_PACKED
   * - RGB10_PACKED
   * - BGR10_PACKED
   * - RGB10_UNPACKED
   * - BGR10_UNPACKED
   * - RGB12_UNPACKED
   * - BGR12_UNPACKED
   * - RGBA12_UNPACKED
   * - BGRA12_UNPACKED
   * - JPEG
   *
   * @param color_mode Color mode
   * @returns 0 for success, -1 for failure
   */
  int setColorMode(const std::string &color_mode);

  /**
   * Get color mode
   *
   * @param color_mode Color mode
   * @returns 0 for success, -1 for failure
   */
  int getColorMode(std::string &color_mode);

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
