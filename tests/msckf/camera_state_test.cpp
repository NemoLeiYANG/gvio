#include "gvio/munit.hpp"
#include "gvio/msckf/camera_state.hpp"

namespace gvio {

int test_CameraState_constructor() {
  CameraState cam_state;

  MU_CHECK_EQ(6, cam_state.size);
  MU_CHECK_EQ(-1, cam_state.frame_id);
  MU_CHECK(zeros(3, 1).isApprox(cam_state.p_G));
  MU_CHECK(Vec4(0.0, 0.0, 0.0, 1.0).isApprox(cam_state.q_CG));

  return 0;
}

int test_CameraState_correct() {
  CameraState cam_state;

  Vec3 dtheta_CG{0.0, 0.0, 0.0};
  Vec3 dp_G{0.0, 0.0, 0.0};

  VecX dx;
  dx.resize(6, 1);
  dx.block(0, 0, 3, 1) = dtheta_CG;
  dx.block(3, 0, 3, 1) = dp_G;

  cam_state.correct(dx);

  return 0;
}

int test_CameraState_setFrameID() {
  CameraState cam_state;

  FrameID id = 123;
  cam_state.setFrameID(id);

  MU_CHECK_EQ(id, cam_state.frame_id);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_CameraState_constructor);
  MU_ADD_TEST(test_CameraState_correct);
  MU_ADD_TEST(test_CameraState_setFrameID);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
