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
#include "gvio/sim/camera_motion.hpp"

namespace gvio {
/**
 * @addtogroup sim
 * @{
 */

/**
 * Feature bounds
 */
struct feature_bounds {
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
  double z_min = 0.0;
  double z_max = 0.0;
};

#define MONOCULAR_CAMERA "MONOCULAR_CAMERA"
#define STEREO_CAMERA "STEREO_CAMERA"

/**
 * Simulation world
 */
class SimWorld {
public:
  double t = 0.0;
  double t_end = 0.0;
  double dt = 0.0;
  size_t time_index = 0;
  size_t frame_index = 0;

  VirtualCamera camera;
  std::string camera_type = MONOCULAR_CAMERA;
  Mat4 T_C1_C0 = I(4);
  CameraMotion camera_motion;

  Vec3 origin{0.0, 0.0, 0.0};
  Vec3 dimensions{30.0, 30.0, 10.0};
  size_t nb_features = 1000;
  MatX features3d;

  long track_id_counter = 0;
  std::vector<int> features_tracking;
  std::map<int, FeatureTrack> tracks_tracking;
  FeatureTracks tracks_lost;

  std::string output_path = "/tmp/sim";
  std::ofstream gnd_file;
  std::ofstream est_file;
  std::ofstream mea_file;
  std::ofstream cam0_idx_file;

  SimWorld();
  virtual ~SimWorld();

  /**
   * Configure
   *
   * @param t_end Time end (s)
   * @param dt Time step (s)
   * @returns 0 for success, -1 for failure
   */
  int configure(const double t_end, const double dt);

  /**
   * Configure
   *
   * @param config_file Path to config file
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Setup output directory and files
   * @returns 0 for success, -1 for failure
   */
  int setupOutput();

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
   * Get lost feature tracks
   *
   * @returns Lost feature tracks
   */
  FeatureTracks getLostTracks();

  /**
   * Record ground truth
   *
   * @param time Relative time in seconds (where 0 is start)
   * @param p_G Ground Truth position in global frame
   * @param v_G Ground Truth velocity in global frame
   * @param rpy_G Ground Truth roll, pitch, and yaw in global frame
   *
   * @returns 0 for success, -1 for failure
   */
  int recordGroundTruth(const double time,
                        const Vec3 &p_G,
                        const Vec3 &v_G,
                        const Vec3 &rpy_G);

  /**
   * Record measurement
   *
   * @param time Relative time in seconds (where 0 is start)
   * @param measurement_a_B Accelerometer measurement
   * @param measurement_w_B Gyroscope measurement
   *
   * @returns 0 for success, -1 for failure
   */
  int recordMeasurement(const double time,
                        const Vec3 &measurement_a_B,
                        const Vec3 &measurement_w_B);

  /**
   * Record camera observation
   *
   * @param feature_ids Feature IDs
   * @param keypoints Observed keypoints
   * @param landmarks Observed landmarks
   */
  int recordCameraObservation(const std::vector<size_t> &feature_ids,
                              const std::vector<Vec2> &keypoints,
                              const std::vector<Vec3> &landmarks);

  /**
   * Record estimate
   *
   * @param time Relative time in seconds (where 0 is start)
   * @param p_G Position in global frame
   * @param v_G Velocity in global frame
   * @param rpy_G Roll, pitch and yaw in global frame
   *
   * @returns 0 for success, -1 for failure
   */
  int recordEstimate(const double time,
                     const Vec3 &p_G,
                     const Vec3 &v_G,
                     const Vec3 &rpy_G);

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
