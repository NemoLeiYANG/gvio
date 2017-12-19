/**
 * @file
 * @defgroup msckf msckf
 */
#ifndef GVIO_MSCKF_MSCKF_HPP
#define GVIO_MSCKF_MSCKF_HPP

#include <vector>

#include "gvio/util/util.hpp"
#include "gvio/quaternion/jpl.hpp"
#include "gvio/feature2d/feature_tracker.hpp"

#include "gvio/msckf/camera_state.hpp"
#include "gvio/msckf/imu_state.hpp"
#include "gvio/msckf/msckf.hpp"

namespace gvio {
/**
 * @addtogroup msckf
 * @{
 */

/**
 * Multi-State Constraint Kalman Filter
 *
 * This class implements the MSCKF based on:
 *
 *   Mourikis, Anastasios I., and Stergios I. Roumeliotis. "A multi-state
 *   constraint Kalman filter for vision-aided inertial navigation." Robotics
 *   and automation, 2007 IEEE international conference on. IEEE, 2007.  APA
 *
 *   A.I. Mourikis, S.I. Roumeliotis: "A Multi-state Kalman Filter for
 *   Vision-Aided Inertial Navigation," Technical Report, September 2006
 *   [http://www.ee.ucr.edu/~mourikis/tech_reports/TR_MSCKF.pdf]
 */
struct MSCKF {
  // Covariance matrices
  MatX P_cam;
  MatX P_imu_cam;

  // IMU
  IMUState imu_state;

  // Camera
  // -- State
  CameraStates cam_states;
  FrameID counter_frame_id = 0;
  // -- Extrinsics
  Vec3 ext_p_IC;
  Vec4 ext_q_CI;
  // -- Noise
  double n_u;
  double n_v;

  // Settings
  bool enable_ns_trick = true;
  bool enable_qr_trick = true;

  MSCKF() {}

  /**
   * Augment state vector with new camera state
   *
   * Augment state and covariance matrix with a copy of the current camera
   * pose estimate when a new image is recorded
   */
  void augmentState();

  /**
   * Get camera states the feature track was observed in
   */
  int getTrackCameraStates(const FeatureTrack &track,
                           CameraStates &track_cam_states);

  /**
   * Return covariance matrix P
   */
  MatX P();

  /**
   * Return length of sliding window
   */
  int N();

  /**
   * Return measurement matrix H
   */
  void H(MatX &H_f_j, MatX &H_x_j);

  /**
   * Prediction update
   */
  void predictionUpdate(const Vec3 &a_m, const Vec3 &w_m, const double dt);

  /**
   * Calculate track residuals
   */
  void calTrackResiduals();

  /**
   * Measurmement update
   */
  void measurementUpdate();
};

/** @} group msckf */
} // namespace gvio
#endif // GVIO_MSCKF_MSCKF_HPP
