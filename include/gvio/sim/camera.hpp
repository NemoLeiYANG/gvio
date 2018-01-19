/**
 * @file
 * @ingroup sim
 */
#ifndef GVIO_SIM_CAMERA_HPP
#define GVIO_SIM_CAMERA_HPP

#include "gvio/util/util.hpp"
#include "gvio/camera/pinhole_model.hpp"

namespace gvio {
/**
 * @addtogroup sim
 * @{
 */

/**
 * Virtual camera
 */
class VirtualCamera {
public:
  PinholeModel camera_model;

  VirtualCamera() {}
  VirtualCamera(const int image_width,
                const int image_height,
                const double fx,
                const double fy,
                const double cx,
                const double cy)
      : camera_model{image_width, image_height, fx, fy, cx, cy} {}
  virtual ~VirtualCamera() {}

  /**
   * Return features are observed by camera
   *
   * **IMPORTANT**: This function assumes the inputs uses a coordinate sytem
   * whwere x-forard, y-left, z-up. This is in contrast to common camera
   * coordinate system where z is forward.
   *
   * @param features Feature matrix witn N features per row (i.e. Nx3 matrix)
   * @param rpy_G Orientation as roll, pitch  yaw. Note: x is forward
   * @param t_G Translation. Note: x is forward
   * @param feature_ids Features observed
   *
   * @returns Observed features in the image plane
   */
  MatX observedFeatures(const MatX &features,
                        const Vec3 &rpy_G,
                        const Vec3 &t_G,
                        std::vector<int> &feature_ids);
};

/** @} group camera */
} // namespace gvio
#endif // GVIO_SIM_SIM_CAMERA_HPP
