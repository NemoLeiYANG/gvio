#include "gvio/munit.hpp"
#include "gvio/gimbal/calibration/residual.hpp"

namespace gvio {

#define TEST_DATA "/home/chutsu/Dropbox/measurements"

int test_GimbalCalibResidual_dhTransform() {
  GimbalCalibResidual err;

  auto result = err.dhTransform(0.0, 0.0, 0.0, 0.0);
  std::cout << result << std::endl;

  return 0;
}

int test_GimbalCalibResidual_euler321ToRot() {
  GimbalCalibResidual err;

  double rpy[3] = {0.0, 0.0, 0.0};
  auto result = err.euler321ToRot(rpy);
  std::cout << result << std::endl;

  return 0;
}

int test_GimbalCalibResidual_K() {
  GimbalCalibResidual err;

  const double fx = 1.0;
  const double fy = 2.0;
  const double cx = 3.0;
  const double cy = 4.0;
  const Mat3 K = err.K(fx, fy, cx, cy);
  std::cout << K << std::endl;

  MU_CHECK_FLOAT(K(0, 0), fx);
  MU_CHECK_FLOAT(K(1, 1), fy);
  MU_CHECK_FLOAT(K(0, 2), cx);
  MU_CHECK_FLOAT(K(1, 2), cy);

  return 0;
}

// int test_GimbalCalibResidual_T_sd() {
//   // Load params
//   CalibParams params;
//   const std::string config_file = TEST_DATA "/params.yaml";
//   const std::string joint_file = TEST_DATA "/joint.csv";
//   if (params.load(config_file, joint_file) != 0) {
//     LOG_ERROR("Failed to load optimization params!");
//     return -1;
//   }
//
//   // Test T_sd
//   GimbalCalibResidual err;
//   double Lambda1[1] = {0.0};
//   double Lambda2[1] = {0.0};
//   Mat4 T_sd = err.T_sd(params.tau_s,
//                        params.tau_d,
//                        params.w1,
//                        params.w2,
//                        Lambda1,
//                        Lambda2);
//
//   std::cout << T_sd << std::endl;
//
//   return 0;
// }

int test_GimbalCalibResidual_evaluate() {
  // Data
  CalibData data;
  if (data.load(TEST_DATA) != 0) {
    LOG_ERROR("Failed to load calibration data [%s]!", TEST_DATA);
    return -1;
  }

  // clang-format off
  Mat3 K_s;
  K_s << 393.05958542802006, 0.0, 369.5410032157271,
      0.0, 392.7958587055595, 241.34514001589662,
      0.0, 0.0, 1.0;

  Mat3 K_d;
  K_d << 524.2644080937374, 0.0, 358.2750466868412,
      0.0, 524.2498715777907, 238.4992907044288,
      0.0, 0.0, 1.0;
  // clang-format on

  // Params
  CalibParams params;
  const std::string config_file = TEST_DATA "/params.yaml";
  const std::string joint_file = TEST_DATA "/joint.csv";
  if (params.load(config_file, joint_file) != 0) {
    LOG_ERROR("Failed to load optimization params!");
    return -1;
  }

  // Residuals
  for (int i = 0; i < data.nb_measurements; i++) {
    for (int j = 0; j < data.P_s[i].rows(); j++) {
      double residual[4] = {0.0, 0.0, 0.0, 0.0};

      auto err = new GimbalCalibResidual(data.P_s[i].row(j),
                                         data.P_d[i].row(j),
                                         data.Q_s[i].row(j),
                                         data.Q_d[i].row(j),
                                         K_s,
                                         K_d);

      err->operator()(params.tau_s,
                      params.tau_d,
                      params.w1,
                      params.w2,
                      &params.Lambda1[i],
                      &params.Lambda2[i],
                      residual);

      delete err;

      if (std::isfinite(residual[0]) == false) {
        std::cout << "i: " << i << std::endl;
        std::cout << "j: " << j << std::endl;
        std::cout << "residual[0]: " << residual[0] << std::endl;
        std::cout << "residual[1]: " << residual[1] << std::endl;
        std::cout << "residual[2]: " << residual[2] << std::endl;
        std::cout << "residual[3]: " << residual[3] << std::endl;

        std::cout << "data.P_s[i].row(j): " << data.P_s[i].row(j) << std::endl;
        std::cout << "data.P_d[i].row(j): " << data.P_d[i].row(j) << std::endl;
        std::cout << "data.Q_s[i].row(j): " << data.Q_s[i].row(j) << std::endl;
        std::cout << "data.Q_d[i].row(j): " << data.Q_d[i].row(j) << std::endl;
        std::cout << "Lambda1: " << params.Lambda1[i] << std::endl;
        std::cout << "Lambda2: " << params.Lambda2[i] << std::endl;
        exit(0);
      }

      std::cout << "residual[0]: " << residual[0] << std::endl;
      std::cout << "residual[1]: " << residual[1] << std::endl;
      std::cout << "residual[2]: " << residual[2] << std::endl;
      std::cout << "residual[3]: " << residual[3] << std::endl;
    }
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_GimbalCalibResidual_dhTransform);
  MU_ADD_TEST(test_GimbalCalibResidual_euler321ToRot);
  MU_ADD_TEST(test_GimbalCalibResidual_K);
  // MU_ADD_TEST(test_GimbalCalibResidual_T_sd);
  MU_ADD_TEST(test_GimbalCalibResidual_evaluate);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
