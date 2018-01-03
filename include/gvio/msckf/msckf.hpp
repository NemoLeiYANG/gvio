/**
 * @file
 * @defgroup msckf msckf
 */
#ifndef GVIO_MSCKF_MSCKF_HPP
#define GVIO_MSCKF_MSCKF_HPP

#include <vector>
#include <algorithm>

#include "gvio/util/util.hpp"
#include "gvio/quaternion/jpl.hpp"
#include "gvio/camera/camera_model.hpp"
#include "gvio/feature2d/feature_tracker.hpp"

#include "gvio/msckf/camera_state.hpp"
#include "gvio/msckf/feature_estimator.hpp"
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
class MSCKF {
public:
  // Covariance matrices
  MatX P_cam = zeros(CameraState::size, CameraState::size);
  MatX P_imu_cam = zeros(IMUState::size, CameraState::size);
  // MatX P_cam;
  // MatX P_imu_cam;

  // IMU
  IMUState imu_state;

  // Camera
  // -- State
  CameraStates cam_states;
  FrameID counter_frame_id = 0;
  // -- Extrinsics
  Vec3 ext_p_IC = zeros(3, 1);
  Vec4 ext_q_CI = zeros(4, 1);
  // -- Noise
  double n_u = 0.0;
  double n_v = 0.0;
  // -- Model
  CameraModel *camera_model;

  // Settings
  size_t max_nb_tracks = 1000;
  size_t min_track_length = 10;
  bool enable_ns_trick = true;
  bool enable_qr_trick = true;

  MSCKF() {}

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
   *
   * @param track Feature track
   * @param track_cam_states List of camera states
   * @param p_G_f Feature position in the global frame
   * @param H_f_j Measurement matrix
   * @param H_x_j Measurement matrix
   */
  void H(const FeatureTrack &track,
         const CameraStates &track_cam_states,
         const Vec3 &p_G_f,
         MatX &H_f_j,
         MatX &H_x_j);

  /**
   * Augment state vector with new camera state
   *
   * Augment state and covariance matrix with a copy of the current camera
   * pose estimate when a new image is recorded
   */
  void augmentState();

  /**
   * Get camera states the feature track was observed in
   *
   * @param track Feature track
   * @returns Camera states where feature track was observed
   */
  CameraStates getTrackCameraStates(const FeatureTrack &track);

  /**
   * Prediction update
   *
   * @param a_m Measured acceleration in body frame
   * @param w_m Measured angular velicty in body frame
   * @param dt Time difference in seconds
   */
  void predictionUpdate(const Vec3 &a_m, const Vec3 &w_m, const double dt);

  /**
   * Calculate track residuals
   *
   * @param track Feature track
   * @param H_j Measurement jacobian matrix
   * @param r_j Residuals vector
   * @param R_j Measurement covariance matrix
   *
   * @returns
   *  - -1: Track length < Min track length
   *  - -2: Failed to estimate feature position
   */
  int calTrackResiduals(const FeatureTrack &track,
                        MatX &H_j,
                        VecX &r_j,
                        MatX &R_j);

  /**
   * Calculate residuals
   *
   * @param tracks Feature tracks
   * @param T_H
   * @param r_n Residuals vector
   * @param R_n Measurement covariance matrix
   *
   * @returns
   *  - -1: Track length < Min track length
   *  - -2: Failed to estimate feature position
   */
  int calResiduals(const FeatureTracks &tracks,
                   MatX &T_H,
                   VecX &r_n,
                   VecX &R_n);

  /**
   * Correct IMU state
   *
   * @param dx Correction vector
   */
  void correctIMUState(const VecX &dx);

  /**
   * Correct camera states
   *
   * @param dx Correction vector
   */
  void correctCameraStates(const VecX &dx);

  /**
   * Measurmement update
   *
   * @param tracks Feature tracks
   */
  int measurementUpdate(FeatureTracks &tracks);
};

/** @} group msckf */
} // namespace gvio
#endif // GVIO_MSCKF_MSCKF_HPP
