
int test_CalibValidator_K() {
  CalibValidator validator;

  // Load validator
  validator.load(3, TEST_CALIB_FILE, TEST_TARGET_FILE);
  std::cout << validator.K(0) << std::endl;
  std::cout << convert(validator.K(0)) << std::endl;

  return 0;
}

int test_CalibValidator_D() {
  CalibValidator validator;

  // Load validator
  validator.load(3, TEST_CALIB_FILE, TEST_TARGET_FILE);
  std::cout << validator.D(0) << std::endl;
  std::cout << convert(validator.D(0)) << std::endl;

  return 0;
}
