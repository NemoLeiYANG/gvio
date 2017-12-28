#include "gvio/camera/ids.hpp"

namespace gvio {

enum CaptureMode ueye_str2capturemode(const std::string &mode) {
  if (mode == "FREE_RUN") {
    return CaptureMode::FREE_RUN;
  } else if (mode == "SOFTARE_TRIGGER") {
    return CaptureMode::SOFTWARE_TRIGGER;
  }
  return CaptureMode::INVALID;
}

int ueye_str2colormode(const std::string &mode) {
  if (mode == "RAW8")
    return IS_CM_SENSOR_RAW8;
  else if (mode == "MONO8")
    return IS_CM_MONO8;
  else if (mode == "RAW10")
    return IS_CM_SENSOR_RAW10;
  else if (mode == "RAW12")
    return IS_CM_SENSOR_RAW12;
  else if (mode == "RAW16")
    return IS_CM_SENSOR_RAW16;
  else if (mode == "MONO10")
    return IS_CM_MONO10;
  else if (mode == "MONO12")
    return IS_CM_MONO12;
  else if (mode == "MONO12")
    return IS_CM_MONO16;
  else if (mode == "BGR5")
    return IS_CM_BGR5_PACKED;
  else if (mode == "BGR565")
    return IS_CM_BGR565_PACKED;
  else if (mode == "UYVY")
    return IS_CM_UYVY_PACKED;
  else if (mode == "UYVY_MONO")
    return IS_CM_UYVY_MONO_PACKED;
  else if (mode == "UYVY_BAYER")
    return IS_CM_UYVY_BAYER_PACKED;
  else if (mode == "CBYCRY_PACKED")
    return IS_CM_CBYCRY_PACKED;
  else if (mode == "RGB8_PACKED")
    return IS_CM_RGB8_PACKED;
  else if (mode == "BGR8_PACKED")
    return IS_CM_BGR8_PACKED;
  else if (mode == "RGB8_PLANAR")
    return IS_CM_RGB8_PLANAR;
  else if (mode == "RGBA8_PACKED")
    return IS_CM_RGBA8_PACKED;
  else if (mode == "BGRA8_PACKED")
    return IS_CM_BGRA8_PACKED;
  else if (mode == "RGBY8_PACKED")
    return IS_CM_RGBY8_PACKED;
  else if (mode == "BGRY8_PACKED")
    return IS_CM_BGRY8_PACKED;
  else if (mode == "RGB10_PACKED")
    return IS_CM_RGB10_PACKED;
  else if (mode == "BGR10_PACKED")
    return IS_CM_BGR10_PACKED;
  else if (mode == "RGB10_UNPACKED")
    return IS_CM_RGB10_UNPACKED;
  else if (mode == "BGR10_UNPACKED")
    return IS_CM_BGR10_UNPACKED;
  else if (mode == "RGB12_UNPACKED")
    return IS_CM_RGB12_UNPACKED;
  else if (mode == "BGR12_UNPACKED")
    return IS_CM_BGR12_UNPACKED;
  else if (mode == "RGBA12_UNPACKED")
    return IS_CM_RGBA12_UNPACKED;
  else if (mode == "BGRA12_UNPACKED")
    return IS_CM_BGRA12_UNPACKED;
  else if (mode == "JPEG")
    return IS_CM_JPEG;
  else
    return -1;
}

int ueye_colormode2bpp(int mode) {
  switch (mode) {
    case IS_CM_SENSOR_RAW8:
    case IS_CM_MONO8: return 8;
    case IS_CM_SENSOR_RAW10:
    case IS_CM_SENSOR_RAW12:
    case IS_CM_SENSOR_RAW16:
    case IS_CM_MONO10:
    case IS_CM_MONO12:
    case IS_CM_MONO16:
    case IS_CM_BGR5_PACKED:
    case IS_CM_BGR565_PACKED:
    case IS_CM_UYVY_PACKED:
    case IS_CM_UYVY_MONO_PACKED:
    case IS_CM_UYVY_BAYER_PACKED:
    case IS_CM_CBYCRY_PACKED: return 16;
    case IS_CM_RGB8_PACKED:
    case IS_CM_BGR8_PACKED:
    case IS_CM_RGB8_PLANAR: return 24;
    case IS_CM_RGBA8_PACKED:
    case IS_CM_BGRA8_PACKED:
    case IS_CM_RGBY8_PACKED:
    case IS_CM_BGRY8_PACKED:
    case IS_CM_RGB10_PACKED:
    case IS_CM_BGR10_PACKED: return 32;
    case IS_CM_RGB10_UNPACKED:
    case IS_CM_BGR10_UNPACKED:
    case IS_CM_RGB12_UNPACKED:
    case IS_CM_BGR12_UNPACKED: return 48;
    case IS_CM_RGBA12_UNPACKED:
    case IS_CM_BGRA12_UNPACKED: return 64;
    case IS_CM_JPEG:
    default: return 0;
  }
}

int ueye_colormode2channels(int mode) {
  switch (mode) {
    case IS_CM_SENSOR_RAW8:
    case IS_CM_MONO8:
    case IS_CM_SENSOR_RAW10:
    case IS_CM_SENSOR_RAW12:
    case IS_CM_SENSOR_RAW16:
    case IS_CM_MONO10:
    case IS_CM_MONO12:
    case IS_CM_MONO16: return 1;
    case IS_CM_BGR5_PACKED:
    case IS_CM_BGR565_PACKED:
    case IS_CM_UYVY_PACKED:
    case IS_CM_UYVY_MONO_PACKED:
    case IS_CM_UYVY_BAYER_PACKED:
    case IS_CM_CBYCRY_PACKED:
    case IS_CM_RGB8_PACKED:
    case IS_CM_BGR8_PACKED:
    case IS_CM_RGB8_PLANAR:
    case IS_CM_RGBA8_PACKED:
    case IS_CM_BGRA8_PACKED:
    case IS_CM_RGBY8_PACKED:
    case IS_CM_BGRY8_PACKED:
    case IS_CM_RGB10_PACKED:
    case IS_CM_BGR10_PACKED:
    case IS_CM_RGB10_UNPACKED:
    case IS_CM_BGR10_UNPACKED:
    case IS_CM_RGB12_UNPACKED:
    case IS_CM_BGR12_UNPACKED:
    case IS_CM_RGBA12_UNPACKED:
    case IS_CM_BGRA12_UNPACKED: return 3;
    case IS_CM_JPEG:
    default: return 0;
  }
}

void ueye_list_cameras() {
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

void ueye_print_error(const HIDS &handle) {
  char *err_msg = (char *) malloc(sizeof(char) * 200);
  memset(err_msg, 0, 200);
  int err_code = 0;

  is_GetError(handle, &err_code, &err_msg);
  LOG_ERROR("Error[%d]: %s\n", err_code, err_msg);

  free(err_msg);
}

void ueye_print_sensor_info(const SENSORINFO &info) {
  std::cout << "Sensor ID: " << info.SensorID << std::endl;
  std::cout << "Sensor name: " << info.strSensorName << std::endl;

  std::cout << "Color mode: ";
  if (info.nColorMode == IS_COLORMODE_MONOCHROME) {
    std::cout << "Monochrome" << std::endl;
  } else if (info.nColorMode == IS_COLORMODE_BAYER) {
    std::cout << "Bayer" << std::endl;
  } else if (info.nColorMode == IS_COLORMODE_CBYCRY) {
    std::cout << "CBYCRY" << std::endl;
  } else if (info.nColorMode == IS_COLORMODE_JPEG) {
    std::cout << "JPEG" << std::endl;
  } else if (info.nColorMode == IS_COLORMODE_INVALID) {
    std::cout << "INVALID" << std::endl;
  } else {
    std::cout << "UNKNOWN" << std::endl;
  }

  std::cout << "Image max width: " << info.nMaxWidth << std::endl;
  std::cout << "Image max height: " << info.nMaxHeight << std::endl;
  std::cout << "Has analogue master gain: " << info.bMasterGain << std::endl;
  std::cout << "Has analogue red channel gain: " << info.bRGain << std::endl;
  std::cout << "Has analogue green channel gain: " << info.bGGain << std::endl;
  std::cout << "Has analogue blue channel gain: " << info.bBGain << std::endl;

  std::cout << "Shutter type: ";
  if (info.bGlobShutter) {
    std::cout << "Global shutter" << std::endl;
  } else {
    std::cout << "Rolling shutter" << std::endl;
  }

  std::cout << "Pixel size (um): " << info.wPixelSize / 100.0 << std::endl;

  std::cout << "Color of first pixel (top left): ";
  if (info.nUpperLeftBayerPixel == BAYER_PIXEL_RED) {
    std::cout << "Red" << std::endl;
  } else if (info.nUpperLeftBayerPixel == BAYER_PIXEL_GREEN) {
    std::cout << "Green" << std::endl;
  } else if (info.nUpperLeftBayerPixel == BAYER_PIXEL_BLUE) {
    std::cout << "Blue" << std::endl;
  }
}

void ueye_print_camera_info(const CAMINFO &info) {
  std::cout << "Serial No.: " << info.SerNo << std::endl;
  std::cout << "ID: " << info.ID << std::endl;
  std::cout << "Version: " << info.Version << std::endl;
  std::cout << "Date: " << info.Date << std::endl;
  std::cout << "Select: " << info.Select << std::endl;

  std::cout << "Type: ";
  if (info.Type == IS_CAMERA_TYPE_UEYE_USB_SE) {
    std::cout << "USB uEye SE" << std::endl;
  } else if (info.Type == IS_CAMERA_TYPE_UEYE_USB_LE) {
    std::cout << "USB uEye LE" << std::endl;
  } else if (info.Type == IS_CAMERA_TYPE_UEYE_USB_ML) {
    std::cout << "USB uEye ML" << std::endl;
  } else if (info.Type == IS_CAMERA_TYPE_UEYE_USB3_CP) {
    std::cout << "USB3 uEye CP" << std::endl;
  } else if (info.Type == IS_CAMERA_TYPE_UEYE_USB3_LE) {
    std::cout << "USB3 uEye LE" << std::endl;
  } else if (info.Type == IS_CAMERA_TYPE_UEYE_USB3_ML) {
    std::cout << "USB3 uEye ML" << std::endl;
  } else if (info.Type == IS_CAMERA_TYPE_UEYE_USB3_XC) {
    std::cout << "USB3 uEye XC" << std::endl;
  } else if (info.Type == IS_CAMERA_TYPE_UEYE_ETH_SE) {
    std::cout << "GigE uEye SE" << std::endl;
  } else if (info.Type == IS_CAMERA_TYPE_UEYE_ETH_REP) {
    std::cout << "GigE uEye RE Poe" << std::endl;
  } else if (info.Type == IS_CAMERA_TYPE_UEYE_ETH_CP) {
    std::cout << "GigE uEye CP" << std::endl;
  } else if (info.Type == IS_CAMERA_TYPE_UEYE_ETH_LE) {
    std::cout << "GigE uEye LE" << std::endl;
  } else if (info.Type == IS_CAMERA_TYPE_UEYE_PMC) {
    std::cout << "Virtual Multicast Camera" << std::endl;
  }
}

int raw2cvmat(void *image_data,
              const int image_width,
              const int image_height,
              const int channels,
              const int bpp,
              cv::Mat &output) {
  // Setup
  const size_t image_size = image_width * image_height * channels;
  const size_t image_rows = image_height;
  const size_t image_cols = image_width;
  const size_t row_bytes = image_size / image_rows;

  // Get cv::Mat type
  int cv_type = -1;
  if (bpp == 8) {
    switch (channels) {
      case 1: cv_type = CV_8UC1; break;
      case 3: cv_type = CV_8UC3; break;
      default: LOG_ERROR("Not implemented!"); return -1;
    }
  } else if (bpp == 16) {
    switch (channels) {
      case 1: cv_type = CV_16UC1; break;
      case 3: cv_type = CV_16UC3; break;
      default: LOG_ERROR("Not implemented!"); return -1;
    }
  } else {
    LOG_ERROR("Not implemented!");
    return -1;
  }

  // Convert raw image data to cv::Mat
  cv::Mat(image_rows, image_cols, cv_type, image_data, row_bytes)
      .copyTo(output);

  return 0;
}

IDSCamera::~IDSCamera() {
  // Pre-check
  if (this->configured == false) {
    return;
  }

  // Free buffers
  this->freeBuffers();

  // Close camera driver
  int retval = is_ExitCamera(this->cam_handle);
  if (retval != IS_SUCCESS) {
    LOG_ERROR("Failed to exit camera!");
  }
}

int IDSCamera::configure(const std::string &config_file) {
  int retval = -1;
  ConfigParser parser;

  int camera_index = 0;
  std::string capture_mode;
  std::string color_mode;
  int nb_buffers = 2;
  parser.addParam("camera_index", &camera_index);
  parser.addParam("nb_buffers", &nb_buffers);
  parser.addParam("capture_mode", &capture_mode);
  parser.addParam("color_mode", &color_mode);
  parser.addParam("image_width", &this->image_width);
  parser.addParam("image_height", &this->image_height);
  parser.addParam("offset_x", &this->offset_x);
  parser.addParam("offset_y", &this->offset_y);
  parser.addParam("pixel_clock", &this->pixel_clock);
  parser.addParam("frame_rate", &this->frame_rate);
  parser.addParam("gain", &this->gain);
  if (parser.load(config_file) != 0) {
    LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
    return -1;
  }

  // Query for number of connected cameras
  int nb_cameras = -1;
  retval = is_GetNumberOfCameras(&nb_cameras);
  if (retval != IS_SUCCESS) {
    LOG_ERROR("Failed to get number of connected UEye cameras!");
    return -1;

  } else if (nb_cameras < 1) {
    LOG_ERROR("No UEye cameras are connected!");
    LOG_ERROR("Hint: is the IDS daemon (/etc/init.d/ueyeusbdrc) is running?");
    return -1;
  }
  LOG_INFO("Number of cameras %d", nb_cameras);

  // Initialize camera
  retval = is_InitCamera(&this->cam_handle, NULL);
  if (retval != IS_SUCCESS) {
    LOG_ERROR("Failed to initialize camera!");
    return -1;
  }

  // Get camera info
  retval = is_GetCameraInfo(this->cam_handle, &this->camera_info);
  if (retval != IS_SUCCESS) {
    LOG_ERROR("Failed to poll camera information!");
    return -1;
  }

  // Get sensor info
  retval = is_GetSensorInfo(this->cam_handle, &this->sensor_info);
  if (retval != IS_SUCCESS) {
    LOG_ERROR("Failed to poll sensor information!");
    return -1;
  }

  // Set display mode
  retval = is_SetDisplayMode(this->cam_handle, IS_SET_DM_DIB);
  if (retval != IS_SUCCESS) {
    LOG_ERROR("This camera does not support Device Independent Bitmap mode");
    return -1;
  }

  // Set capture mode
  this->capture_mode = ueye_str2capturemode(capture_mode);
  if (this->capture_mode == CaptureMode::INVALID) {
    LOG_ERROR("Invalid capture mode [%s]!", capture_mode.c_str());
    return -1;
  }
  retval = this->setCaptureMode(this->capture_mode);
  if (retval != IS_SUCCESS) {
    LOG_ERROR("This camera does not support Device Independent Bitmap mode");
    return -1;
  }

  // Set color mode
  this->color_mode = ueye_str2colormode(color_mode);
  if (this->color_mode == -1) {
    LOG_ERROR("Invalid color mode [%s]!", color_mode.c_str());
    return -1;
  }
  retval = this->setColorMode(this->color_mode);
  if (retval != 0) {
    LOG_ERROR("Failed to set color mode!");
    return -1;
  }

  // Set frame rate
  retval = this->setFrameRate(this->frame_rate);
  if (retval != 0) {
    LOG_ERROR("Failed to set frame rate!");
    return -1;
  }

  // Set gain
  retval = this->setGain(this->gain);
  if (retval != 0) {
    LOG_ERROR("Failed to set gain!");
    return -1;
  }

  // Set ROI
  retval = this->setROI(this->offset_x,
                        this->offset_y,
                        this->image_width,
                        this->image_height);
  if (retval != 0) {
    LOG_ERROR("Failed to set ROI!");
    return -1;
  }

  // Allocate frame buffer memory
  retval = this->allocBuffers(nb_buffers, ueye_colormode2bpp(this->color_mode));
  if (retval != 0) {
    LOG_ERROR("Failed to allocate memory for frame buffers!");
    return -1;
  }

  // Enable frame event
  if (is_EnableEvent(this->cam_handle, IS_SET_EVENT_FRAME) != IS_SUCCESS) {
    LOG_ERROR("Failed to enable frame event!");
    return -1;
  }

  // Start capturing
  if (is_CaptureVideo(this->cam_handle, IS_DONT_WAIT) != IS_SUCCESS) {
    LOG_ERROR("Failed to start capture!");
    return -1;
  }

  this->configured = true;
  return 0;
}

int IDSCamera::allocBuffers(const int nb_buffers, const int bpp) {
  this->buffers.resize(nb_buffers);
  this->buffer_id.resize(nb_buffers);

  // Query camera's current resolution settings
  int img_width, img_height;
  if (this->getImageSize(img_width, img_height) != 0) {
    LOG_ERROR("Failed to get image size!");
    return -1;
  }

  // Malloc buffers
  int retval;
  for (int i = 0; i < nb_buffers; i++) {
    // Allocate buffer
    retval = is_AllocImageMem(this->cam_handle,
                              img_width,
                              img_height,
                              bpp,
                              &this->buffers[i],
                              &this->buffer_id[i]);
    if (retval != IS_SUCCESS) {
      LOG_ERROR("Failed to allocate memory for frame buffers!");
      return -1;
    }

    // Set buffer as frame buffer
    retval =
        is_SetImageMem(this->cam_handle, this->buffers[i], this->buffer_id[i]);
    if (retval != IS_SUCCESS) {
      LOG_ERROR("Failed to set memory for frame buffers!");
      return -1;
    }
  }

  return 0;
}

int IDSCamera::freeBuffers() {
  for (size_t i = 0; i < this->buffers.size(); i++) {
    if (is_FreeImageMem(this->cam_handle,
                        this->buffers[i],
                        this->buffer_id[i]) != IS_SUCCESS) {
      LOG_ERROR("Failed to free frame memory!");
      return -1;
    }
  }

  buffers.clear();
  buffer_id.clear();

  return 0;
}

int IDSCamera::setCaptureMode(const enum CaptureMode &capture_mode) {
  int retval;

  // Set capture mode
  switch (capture_mode) {
    case CaptureMode::FREE_RUN:
      retval = is_SetExternalTrigger(this->cam_handle, IS_SET_TRIGGER_OFF);
      break;
    case CaptureMode::SOFTWARE_TRIGGER:
      retval = is_SetExternalTrigger(this->cam_handle, IS_SET_TRIGGER_SOFTWARE);
      break;
    default:
      LOG_ERROR("Not implemented!");
      return -1;
      break;
  }

  // Check return status
  if (retval != IS_SUCCESS) {
    return -1;
  }

  this->capture_mode = capture_mode;
  return 0;
}

int IDSCamera::getCaptureMode(enum CaptureMode &capture_mode) {
  capture_mode = this->capture_mode;
  return 0;
}

int IDSCamera::setPixelClock(const int clock_rate) {
  // Get number of supported pixel clock rates
  int nb_pixel_clock_rates = 0;
  int retval = is_PixelClock(this->cam_handle,
                             IS_PIXELCLOCK_CMD_GET_NUMBER,
                             (void *) &nb_pixel_clock_rates,
                             sizeof(nb_pixel_clock_rates));
  if ((retval != IS_SUCCESS) || (nb_pixel_clock_rates == 0)) {
    LOG_ERROR("Failed to get number of pixel clock rates!");
    return -1;
  }

  // Get valid pixel clock rates
  // -- No camera has more than 150 different pixel clocks.
  int pixel_clock_rates[150];
  ZeroMemory(&pixel_clock_rates, sizeof(pixel_clock_rates));
  retval = is_PixelClock(this->cam_handle,
                         IS_PIXELCLOCK_CMD_GET_LIST,
                         (void *) pixel_clock_rates,
                         nb_pixel_clock_rates * sizeof(int));
  if (retval != IS_SUCCESS) {
    LOG_ERROR("Failed to get valid pixel clock rates!");
    return -1;
  }

  // Set pixel clock
  bool rate_valid = false;
  for (int i = 0; i < nb_pixel_clock_rates; i++) {
    if (clock_rate == pixel_clock_rates[i]) {
      rate_valid = true;
      break;
    }
  }
  if (rate_valid &&
      is_PixelClock(this->cam_handle,
                    IS_PIXELCLOCK_CMD_SET,
                    (void *) &clock_rate,
                    sizeof(clock_rate)) == IS_SUCCESS) {
    this->pixel_clock = clock_rate;
    return 0;
  } else {
    return -1;
  }
}

int IDSCamera::getPixelClock(int &clock_rate) {
  if (is_PixelClock(this->cam_handle,
                    IS_PIXELCLOCK_CMD_GET,
                    (void *) &clock_rate,
                    sizeof(clock_rate)) == IS_SUCCESS) {
    this->pixel_clock = clock_rate;
    return 0;
  } else {
    return -1;
  }

  return 0;
}

int IDSCamera::setColorMode(const int color_mode) {
  const int retval = is_SetColorMode(this->cam_handle, color_mode);
  if (retval != IS_SUCCESS) {
    return -1;
  }

  this->color_mode = color_mode;
  return 0;
}

int IDSCamera::getColorMode(int &color_mode) {
  color_mode = this->color_mode;
  return 0;
}

int IDSCamera::setFrameRate(const double frame_rate) {
  // Get frame rate range
  double time_min = 0.0;
  double time_max = 0.0;
  double time_interval = 0.0;

  if (is_GetFrameTimeRange(this->cam_handle,
                           &time_min,
                           &time_max,
                           &time_interval) != IS_SUCCESS) {
    LOG_ERROR("Failed to the range of frame rates available!");
    return -1;
  }

  // Check if frame rate is valid
  const double rate_max = 1.0 / time_min;
  const double rate_min = 1.0 / time_max;
  if (frame_rate > rate_max || frame_rate < rate_min) {
    LOG_ERROR("Invalid frame rate [%f]", frame_rate);
    LOG_ERROR("Frame rate max: %f", rate_max);
    LOG_ERROR("Frame rate min: %f", rate_min);
    return -1;
  }

  // Set frame rate
  if (is_SetFrameRate(this->cam_handle, frame_rate, &this->frame_rate) !=
      IS_SUCCESS) {
    return -1;
  }

  return 0;
}

int IDSCamera::getFrameRate(double &frame_rate) {
  frame_rate = this->frame_rate;
  return 0;
}

int IDSCamera::setGain(const int gain) {
  // Pre-check gain value
  if (gain < 0 || gain > 100) {
    return -1;
  }

  // Set hardware gain
  int retval = 0;
  retval = is_SetHardwareGain(this->cam_handle,
                              gain,
                              IS_IGNORE_PARAMETER,
                              IS_IGNORE_PARAMETER,
                              IS_IGNORE_PARAMETER);
  if (retval != IS_SUCCESS) {
    return -1;
  }

  return 0;
}

int IDSCamera::getGain(int &gain) {
  // Get hardware gain
  gain = is_SetHardwareGain(this->cam_handle,
                            IS_GET_MASTER_GAIN,
                            IS_IGNORE_PARAMETER,
                            IS_IGNORE_PARAMETER,
                            IS_IGNORE_PARAMETER);
  this->gain = gain;

  return 0;
}

int IDSCamera::setROI(const int offset_x,
                      const int offset_y,
                      const int image_width,
                      const int image_height) {
  int retval;

  // Get sensor info
  retval = is_GetSensorInfo(this->cam_handle, &this->sensor_info);
  if (retval != IS_SUCCESS) {
    LOG_ERROR("Failed to poll sensor information!");
    return -1;
  }
  const int max_height = this->sensor_info.nMaxHeight;
  const int max_width = this->sensor_info.nMaxWidth;

  // Check input
  if ((image_width + offset_x) > max_width) {
    LOG_ERROR("Image width + offset x > sensor max width!");
    LOG_ERROR("ROI is out of bounds!");
    return -1;
  } else if ((image_width + offset_x) < 0) {
    LOG_ERROR("Image width + offset x < 0!");
    LOG_ERROR("ROI is out of bounds!");
    return -1;
  } else if ((image_height + offset_y) > max_height) {
    LOG_ERROR("Image height + offset y > sensor max width!");
    LOG_ERROR("ROI is out of bounds!");
    return -1;
  } else if ((image_height + offset_y) < 0) {
    LOG_ERROR("Image height + offset y < 0!");
    LOG_ERROR("ROI is out of bounds!");
    return -1;
  }

  // Set ROI
  IS_RECT aoi;
  aoi.s32X = offset_x;
  aoi.s32Y = offset_y;
  aoi.s32Width = (image_width == 0) ? max_width : image_width;
  aoi.s32Height = (image_height == 0) ? max_height : image_height;
  if (is_AOI(this->cam_handle,
             IS_AOI_IMAGE_SET_AOI,
             (void *) &aoi,
             sizeof(aoi)) != IS_SUCCESS) {
    return -1;
  }

  return 0;
}

int IDSCamera::getROI(int &offset_x,
                      int &offset_y,
                      int &image_width,
                      int &image_height) {
  IS_RECT aoi;

  // Get ROI
  if (is_AOI(this->cam_handle,
             IS_AOI_IMAGE_GET_AOI,
             (void *) &aoi,
             sizeof(aoi)) != IS_SUCCESS) {
    return -1;
  }

  // Set output variables
  offset_x = aoi.s32X;
  offset_y = aoi.s32Y;
  image_width = aoi.s32Width;
  image_height = aoi.s32Height;

  return 0;
}

int IDSCamera::getImageSize(int &image_width, int &image_height) {
  int offset_x, offset_y;
  return this->getROI(offset_x, offset_y, image_width, image_height);
}

int IDSCamera::getFrame(cv::Mat &image) {
  // Query camera's current resolution settings
  int image_width = 0;
  int image_height = 0;
  if (this->getImageSize(image_width, image_height) != 0) {
    LOG_ERROR("Failed to get image size!");
    return -1;
  }

  // Wait for new frame
  if (is_WaitEvent(this->cam_handle,
                   IS_SET_EVENT_FRAME,
                   (int) (2000 / this->frame_rate)) != IS_SUCCESS) {
    LOG_ERROR("Image wait timeout!");
    return -1;
  }

  // Obtain image data
  void *image_data;
  if (is_GetImageMem(this->cam_handle, &image_data) != IS_SUCCESS) {
    LOG_ERROR("Failed to get image memory!");
    return -1;
  }

  // Convert image data to cv::Mat
  if (raw2cvmat(image_data,
                image_width,
                image_height,
                ueye_colormode2channels(this->color_mode),
                ueye_colormode2bpp(this->color_mode),
                image) != 0) {
    LOG_ERROR("Failed to convert image data to cv::Mat!");
    return -1;
  }

  return 0;
}

// int IDSCamera::run() {
//   return 0;
// }

} // namespace gvio
