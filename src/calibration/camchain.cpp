#include "gvio/calibration/camchain.hpp"

namespace gvio {

Camchain::Camchain() {}

Camchain::~Camchain() {}

int Camchain::load(const int nb_cameras, const std::string &camchain_file) {
  assert(nb_cameras > 0);
  assert(camchain_file.empty() == false);

  // Parse camchain file
  ConfigParser parser;
  CameraProperty cam0, cam1, cam2;
  for (int i = 0; i < nb_cameras; i++) {
    switch (i) {
      case 0:
        parser.addParam("cam0.camera_model", &cam0.camera_model);
        parser.addParam("cam0.distortion_model", &cam0.distortion_model);
        parser.addParam("cam0.distortion_coeffs", &cam0.distortion_coeffs);
        parser.addParam("cam0.intrinsics", &cam0.intrinsics);
        parser.addParam("cam0.resolution", &cam0.resolution);
        break;
      case 1:
        parser.addParam("cam1.camera_model", &cam1.camera_model);
        parser.addParam("cam1.distortion_model", &cam1.distortion_model);
        parser.addParam("cam1.distortion_coeffs", &cam1.distortion_coeffs);
        parser.addParam("cam1.intrinsics", &cam1.intrinsics);
        parser.addParam("cam1.resolution", &cam1.resolution);
        break;
      case 2:
        parser.addParam("cam2.camera_model", &cam2.camera_model);
        parser.addParam("cam2.distortion_model", &cam2.distortion_model);
        parser.addParam("cam2.distortion_coeffs", &cam2.distortion_coeffs);
        parser.addParam("cam2.intrinsics", &cam2.intrinsics);
        parser.addParam("cam2.resolution", &cam2.resolution);
        break;
    }
  }
  parser.addParam("T_C1_C0", &this->T_C1_C0);
  parser.addParam("T_C2_C0.tau_s", &this->tau_s);
  parser.addParam("T_C2_C0.tau_d", &this->tau_d);
  parser.addParam("T_C2_C0.w1", &this->w1);
  parser.addParam("T_C2_C0.w2", &this->w2);
  parser.addParam("T_C2_C0.theta1_offset", &this->theta1_offset);
  parser.addParam("T_C2_C0.theta2_offset", &this->theta2_offset);
  if (parser.load(camchain_file) != 0) {
    return -1;
  }
  this->cam = {cam0, cam1, cam2};

  return 0;
}

} // namespace gvio
