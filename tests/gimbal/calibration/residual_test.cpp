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
  for (int i = 0; i < data.P_s[0].rows(); i++) {
    auto err = GimbalCalibResidual(data.P_s[0].row(i),
                                   data.P_d[0].row(i),
                                   data.Q_s[0].row(i),
                                   data.Q_d[0].row(i),
                                   K_s,
                                   K_d);

    double residual[4] = {0.0, 0.0, 0.0, 0.0};

    err(params.tau_s,
        params.tau_d,
        params.w1,
        params.w2,
        &params.Lambda1[0],
        &params.Lambda2[0],
        residual);

    std::cout << residual[0] << std::endl;
    std::cout << residual[1] << std::endl;
    std::cout << residual[2] << std::endl;
    std::cout << residual[3] << std::endl;
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_GimbalCalibResidual_dhTransform);
  MU_ADD_TEST(test_GimbalCalibResidual_euler321ToRot);
  MU_ADD_TEST(test_GimbalCalibResidual_evaluate);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
