/**
 * @file
 * @defgroup sim sim
 */
#ifndef GVIO_SIM_WORLD_HPP
#define GVIO_SIM_WORLD_HPP

#include <vector>
#include <map>

#include "gvio/util/util.hpp"
#include "gvio/camera/pinhole_model.hpp"
#include "gvio/feature2d/feature_container.hpp"
#include "gvio/sim/twowheel.hpp"
#include "gvio/sim/camera.hpp"

namespace gvio {
/**
 * @addtogroup sim
 * @{
 */

struct feature_bounds {
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
  double z_min = 0.0;
  double z_max = 0.0;
};

class SimWorld {
public:
  double t = 0.0;
  double dt = 0.0;
  double t_end = 0.0;
  size_t time_index = 0;

  TwoWheelRobot robot;
  VirtualCamera camera;

  Vec3 origin{0.0, 0.0, 0.0};
  Vec3 dimensions{30.0, 30.0, 60.0};
  size_t nb_features = 1000;
  MatX features3d;

  std::vector<int> features_tracking;
  std::map<int, FeatureTrack> tracks_tracking;
  FeatureTracks tracks_lost;

  std::ofstream trajectory_file;
  std::ofstream feature3d_file;

  SimWorld() {}
  virtual ~SimWorld() {}

  /**
   * Configure
   *
   * @param dt Time step (s)
   * @returns 0 for success, -1 for failure
   */
  int configure(const double dt);

  /**
   * Create 3D features
   *
   * @param bounds Feature bounds
   * @param nb_features Number of 3D features
   * @returns N Features as a Nx3 matrix
   */
  MatX create3DFeatures(const struct feature_bounds &bounds,
                        const size_t nb_features);

  /**
   * Create 3D feature perimeter around a defined origin in
   * a square like manner
   *
   * @param origin Origin of feature perimeter
   * @param dimensions Diemnsions of feature perimeter
   * @param nb_features Number of 3D features
   * @returns N Features as a Nx3 matrix
   */
  MatX create3DFeaturePerimeter(const Vec3 &origin,
                                const Vec3 &dimensions,
                                const size_t nb_features);

  /**
   * Detect features
   */
  void detectFeatures();

  /**
   * Remove lost feature tracks
   *
   * @returns Lost feature tracks
   */
  FeatureTracks removeLostTracks();

  /**
   * Step simulation
   *
   * @returns 0 for success, -1 for failure
   */
  int step();
};

/** @} group sim */
} // namespace gvio
#endif // GVIO_SIM_WORLD_HPP
