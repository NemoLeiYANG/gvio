#include "gvio/gimbal/calibration/calibration.hpp"

namespace gvio {

GimbalCalib::GimbalCalib() {}

GimbalCalib::~GimbalCalib() {}

int GimbalCalib::load(const std::string &data_dir) {
  // Load calibration data
  if (this->data.load(data_dir) != 0) {
    LOG_ERROR("Failed to load calibration data [%s]!", data_dir.c_str());
    return -1;
  }

  // Load optimization params
  const std::string config_file = data_dir + "/camchain.yaml";
  const std::string joint_file = data_dir + "/joint.csv";
  if (this->params.load(config_file, joint_file) != 0) {
    LOG_ERROR("Failed to load optimization params!");
    return -1;
  }

  // Setup optimization problem
  const Mat3 K_s = this->params.camchain.cam[0].K();
  const Mat3 K_d = this->params.camchain.cam[2].K();
  const Vec4 D_s = this->params.camchain.cam[0].D();
  const Vec4 D_d = this->params.camchain.cam[2].D();

  for (int i = 0; i < this->data.nb_measurements; i++) {
    // Form residual
    for (int j = 0; j < this->data.P_s[i].rows(); j++) {
      const Vec3 P_s = this->data.P_s[i].row(j);
      const Vec3 P_d = this->data.P_d[i].row(j);
      const Vec2 Q_s = this->data.Q_s[i].row(j);
      const Vec2 Q_d = this->data.Q_d[i].row(j);
      auto residual =
          new GimbalCalibResidual(P_s, P_d, Q_s, Q_d, K_s, K_d, D_s, D_d);

      // Build cost function
      auto cost_func =
          new ceres::AutoDiffCostFunction<GimbalCalibResidual, // Residual type
                                          4, // Size of residual
                                          6, // Size of: tau_s
                                          6, // Size of: tau_d
                                          3, // Size of: w1
                                          3, // Size of: w2
                                          1, // Size of: Lambda1
                                          1  // Size of: Lambda2
                                          >(residual);

      // Add residual block to problem
      this->problem.AddResidualBlock(cost_func, // Cost function
                                     NULL,      // Loss function
                                     this->params.tau_s,
                                     this->params.tau_d,
                                     this->params.w1,
                                     this->params.w2,
                                     &this->params.Lambda1[i],
                                     &this->params.Lambda2[i]);
    }
  }

  this->problem.SetParameterBlockConstant(this->params.w2);

  return 0;
}

int GimbalCalib::calibrate() {
  // Set options
  this->options.max_num_iterations = 1000;
  // this->options.use_nonmonotonic_steps = false;
  // this->options.use_inner_iterations = true;
  // this->options.preconditioner_type = ceres::SCHUR_JACOBI;
  // this->options.linear_solver_type = ceres::SPARSE_SCHUR;
  // this->options.minimizer_type = ceres::LINE_SEARCH;
  this->options.function_tolerance = 1e-12;
  this->options.parameter_tolerance = 1e-12;
  this->options.num_threads = 8;
  this->options.num_linear_solver_threads = 8;
  this->options.minimizer_progress_to_stdout = true;

  // Solve
  std::cout << this->params << std::endl;
  ceres::Solve(this->options, &this->problem, &this->summary);
  std::cout << summary.FullReport() << std::endl;
  std::cout << this->params << std::endl;

  return 0;
}

} // namespace gvio
