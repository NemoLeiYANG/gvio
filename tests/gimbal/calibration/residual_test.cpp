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

  auto result = err.euler321ToRot(0.0, 0.0, 0.0);
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

  // Params
  CalibParams params;
  const std::string camchain_file = TEST_DATA "/camchain.yaml";
  const std::string joint_file = TEST_DATA "/joint.csv";
  if (params.load(camchain_file, joint_file) != 0) {
    LOG_ERROR("Failed to load optimization params!");
    return -1;
  }

  const Mat3 K_s = params.camchain.cam[0].K();
  const Mat3 K_d = params.camchain.cam[2].K();
  const Vec4 D_s = params.camchain.cam[0].D();
  const Vec4 D_d = params.camchain.cam[2].D();

  // Residuals
  for (int i = 0; i < data.nb_measurements; i++) {
    for (int j = 0; j < data.P_s[i].rows(); j++) {
      double residual[4] = {0.0, 0.0, 0.0, 0.0};

      auto err = new GimbalCalibResidual(data.P_s[i].row(j),
                                         data.P_d[i].row(j),
                                         data.Q_s[i].row(j),
                                         data.Q_d[i].row(j),
                                         K_s,
                                         K_d,
                                         D_s,
                                         D_d);

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
      std::cout << std::endl;
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
