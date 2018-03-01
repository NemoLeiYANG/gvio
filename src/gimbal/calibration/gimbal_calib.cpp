#include "gvio/gimbal/calibration/calibration.hpp"

namespace gvio {

GimbalCalib::GimbalCalib() {}

GimbalCalib::~GimbalCalib() {}

int GimbalCalib::load(const std::string &data_dir) {
  if (this->data.load(data_dir) != 0) {
    return -1;
  }

  // Setup optimization problem
  for (int i = 0; i < this->data.nb_measurements; i++) {
    // Form residual
    Mat3 K_s = I(3);
    Mat3 K_d = I(3);
    auto residual = new GimbalCalibResidual(this->data.P_s[i],
                                            this->data.P_d[i],
                                            this->data.Q_s[i],
                                            this->data.Q_d[i],
                                            K_s,
                                            K_d);

    // Build cost function
    auto cost_func =
        new ceres::AutoDiffCostFunction<GimbalCalibResidual, // Residual type
                                        4,                   // size of residual
                                        6, // size of 1st parameter - tau_s
                                        1, // size of 2nd parameter - Lambda1
                                        3, // size of 3rd parameter - w1
                                        1, // size of 4th parameter - Lambda2
                                        3, // size of 5th parameter - w2
                                        6  // size of 6th parameter - tau_d
                                        >(residual);

    // Add residual block to problem
    this->problem
        .AddResidualBlock(cost_func,     // cost function
                          NULL,          // loss function
                          this->tau_s,   // static camera to base mechanism
                          this->Lambda1, // Roll
                          this->w1,      // DH
                          this->Lambda2, // Pitch
                          this->w2,      // DH
                          this->tau_d);  // end effector to dynamic camera
  }

  return 0;
}

int GimbalCalib::calibrate() {
  // Set options
  this->options.max_num_iterations = 200;
  this->options.use_nonmonotonic_steps = false;
  this->options.use_inner_iterations = true;
  this->options.preconditioner_type = ceres::SCHUR_JACOBI;
  this->options.linear_solver_type = ceres::SPARSE_SCHUR;
  this->options.parameter_tolerance = 1e-10;
  this->options.num_threads = 8;
  this->options.num_linear_solver_threads = 8;
  this->options.minimizer_progress_to_stdout = true;

  // Solve
  ceres::Solve(this->options, &this->problem, &this->summary);
  std::cout << summary.FullReport() << "\n";

  return 0;
}

} // namespace gvio
