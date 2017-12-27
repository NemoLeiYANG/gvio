#include "gvio/camera/ids.hpp"

namespace gvio {

static int colormode2bpp(int mode) {
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

IDSCamera::~IDSCamera() {
  // Close camera driver
  int retval = is_ExitCamera(this->cam_handle);
  if (retval != IS_SUCCESS) {
    LOG_ERROR("Failed to exit camera!");
  }

  // Free buffers
  this->freeBuffers();
}

int IDSCamera::configure(const std::string &config_file) {
  UNUSED(config_file);

  // Query for number of connected cameras
  int nb_cameras = -1;
  int retval = 0;

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

  // Get sensor info
  retval = is_GetSensorInfo(this->cam_handle, &this->sensor_info);
  if (retval != IS_SUCCESS) {
    LOG_ERROR("Failed to poll sensor information!");
    return -1;
  }

  // Get camera info
  retval = is_GetCameraInfo(this->cam_handle, &this->camera_info);
  if (retval != IS_SUCCESS) {
    LOG_ERROR("Failed to poll camera information!");
    return -1;
  }

  // Set display mode
  retval = is_SetDisplayMode(this->cam_handle, IS_SET_DM_DIB);
  if (retval != IS_SUCCESS) {
    LOG_ERROR("UEye camera does not support Device Independent Bitmap mode");
    return -1;
  }

  // Set color mode
  retval = is_SetColorMode(this->cam_handle, IS_CM_MONO8);
  if (retval != IS_SUCCESS) {
    LOG_ERROR("Failed to set color mode!");
    return -1;
  }

  // Allocate frame buffer memory
  retval = this->allocBuffers(2, colormode2bpp(IS_CM_MONO8));
  if (retval != 0) {
    LOG_ERROR("Failed to allocate memory for frame buffers!");
    return -1;
  }

  return 0;
}

int IDSCamera::allocBuffers(const int nb_buffers, const int bpp) {
  this->buffers.resize(nb_buffers);
  this->buffer_id.resize(nb_buffers);

  // Query camera's current resolution settings
  int img_width, img_height;
  if (this->getResolution(img_width, img_height) != 0) {
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
      return -1;
    }

    // Set buffer as frame buffer
    retval =
        is_SetImageMem(this->cam_handle, this->buffers[i], this->buffer_id[i]);
    if (retval != IS_SUCCESS) {
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
      return -1;
    }
  }

  buffers.clear();
  buffer_id.clear();

  return 0;
}

int IDSCamera::setPixelClock() {
  UINT nNumberOfSupportedPixelClocks = 0;
  int retval = is_PixelClock(this->cam_handle,
                             IS_PIXELCLOCK_CMD_GET_NUMBER,
                             (void *) &nNumberOfSupportedPixelClocks,
                             sizeof(nNumberOfSupportedPixelClocks));

  if ((retval == IS_SUCCESS) && (nNumberOfSupportedPixelClocks > 0)) {
    // No camera has more than 150 different pixel clocks.
    // Of course, the list can be allocated dynamically
    UINT nPixelClockList[150];
    ZeroMemory(&nPixelClockList, sizeof(nPixelClockList));
    retval = is_PixelClock(this->cam_handle,
                           IS_PIXELCLOCK_CMD_GET_LIST,
                           (void *) nPixelClockList,
                           nNumberOfSupportedPixelClocks * sizeof(UINT));
  }

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
    default: return -1; break;
  }

  // Check return status
  if (retval != IS_SUCCESS) {
    return -1;
  }

  return 0;
}

int IDSCamera::getCaptureMode(enum CaptureMode &capture_mode) {
  capture_mode = this->capture_mode;
  return 0;
}

int IDSCamera::getResolution(int &width, int &height) {
  IS_RECT aoi;
  int retval = is_AOI(this->cam_handle,
                      IS_AOI_IMAGE_GET_AOI,
                      (void *) &aoi,
                      sizeof(aoi));
  if (retval != IS_SUCCESS) {
    LOG_ERROR("Failed to retrieve camera resolution!");
    return -1;
  }

  width = aoi.s32Width;
  height = aoi.s32Height;

  return 0;
}

int IDSCamera::getFrame(cv::Mat &image) {
  // Query camera's current resolution settings
  int img_width, img_height;
  if (this->getResolution(img_width, img_height) != 0) {
    return -1;
  }

  for (int i = 0; i < 100; i++) {
    if (is_FreezeVideo(this->cam_handle, IS_WAIT) == IS_SUCCESS) {
      void *img_data;
      if (is_GetImageMem(this->cam_handle, &img_data) != IS_SUCCESS) {
        return -1;
      }

      const size_t channels = 1;
      const size_t img_size = img_width * img_height * channels;
      const size_t img_rows = img_height;
      const size_t img_cols = img_width;
      const size_t row_bytes = img_size / img_rows;
      cv::Mat(img_rows, img_cols, CV_8UC1, img_data, row_bytes).copyTo(image);

      cv::imshow("Image", image);
      cv::waitKey(1);
    }
  }

  return 0;
}

void IDSCamera::printSensorInfo(const SENSORINFO &info) {
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

void IDSCamera::printCameraInfo(const CAMINFO &info) {
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

// int IDSCamera::run() {
//   return 0;
// }

} // namespace gvio
