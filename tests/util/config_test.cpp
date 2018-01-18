#include "gvio/munit.hpp"
#include "gvio/util/data.hpp"
#include "gvio/util/config.hpp"

#define TEST_CONFIG "test_data/utils/config.yaml"

namespace gvio {

int test_ConfigParam_constructor() {
  ConfigParam param;

  MU_CHECK_EQ(TYPE_NOT_SET, param.type);
  MU_CHECK_EQ("", param.key);
  MU_FALSE(param.optional);
  MU_CHECK_EQ(NULL, param.data);

  return 0;
}

int test_ConfigParser_constructor() {
  ConfigParser parser;

  MU_FALSE(parser.config_loaded);

  return 0;
}

int test_ConfigParser_addParam() {
  bool b;
  int i;
  float f;
  double d;
  std::string s;

  std::vector<bool> b_array;
  std::vector<int> i_array;
  std::vector<float> f_array;
  std::vector<double> d_array;
  std::vector<std::string> s_array;

  Vec2 vec2;
  Vec3 vec3;
  Vec4 vec4;
  VecX vecx;

  Mat2 mat2;
  Mat3 mat3;
  Mat4 mat4;
  MatX matx;
  cv::Mat cvmat;

  ConfigParser parser;

  parser.addParam("bool", &b);
  parser.addParam("int", &i);
  parser.addParam("float", &f);
  parser.addParam("double", &d);
  parser.addParam("string", &s);

  parser.addParam("bool_array", &b_array);
  parser.addParam("int_array", &i_array);
  parser.addParam("float_array", &f_array);
  parser.addParam("double_array", &d_array);
  parser.addParam("string_array", &s_array);

  parser.addParam("vector2", &vec2);
  parser.addParam("vector3", &vec3);
  parser.addParam("vector4", &vec4);
  parser.addParam("vector", &vecx);

  parser.addParam("matrix2", &mat2);
  parser.addParam("matrix3", &mat3);
  parser.addParam("matrix4", &mat4);
  parser.addParam("matrix", &matx);
  parser.addParam("matrix", &cvmat);

  MU_CHECK_EQ(19, (int) parser.params.size());
  MU_CHECK_EQ(BOOL, parser.params[0].type);
  MU_CHECK_EQ("bool", parser.params[0].key);
  MU_CHECK(parser.params[0].data != NULL);

  return 0;
}

int test_ConfigParser_getYamlNode() {
  YAML::Node node1, node2;
  ConfigParser parser;

  parser.load(TEST_CONFIG);

  parser.getYamlNode("level3.a.b.c", node1);
  MU_CHECK_EQ(3, node1.as<int>());

  parser.getYamlNode("float", node2);
  MU_CHECK_FLOAT(2.2, node2.as<float>());

  return 0;
}

int test_ConfigParser_loadPrimitive() {
  int i;
  float f;
  double d;
  std::string s;
  ConfigParser parser;
  ConfigParam param;

  // setup
  parser.root = YAML::LoadFile(TEST_CONFIG);
  parser.config_loaded = true;

  // INTEGER
  param.optional = false;
  param.type = INT;
  param.key = "int";
  param.data = &i;
  parser.loadPrimitive(param);
  MU_CHECK_EQ(1, i);

  // FLOAT
  param.optional = false;
  param.type = FLOAT;
  param.key = "float";
  param.data = &f;
  parser.loadPrimitive(param);
  MU_CHECK_FLOAT(2.2, f);

  // DOUBLE
  param.optional = false;
  param.type = DOUBLE;
  param.key = "double";
  param.data = &d;
  parser.loadPrimitive(param);
  MU_CHECK_FLOAT(3.3, d);

  // STRING
  param.optional = false;
  param.type = STRING;
  param.key = "string";
  param.data = &s;
  parser.loadPrimitive(param);
  MU_CHECK_EQ("hello world!", s);

  return 0;
}

int test_ConfigParser_loadArray() {
  std::vector<bool> b_array;
  std::vector<int> i_array;
  std::vector<float> f_array;
  std::vector<double> d_array;
  std::vector<std::string> s_array;
  ConfigParser parser;
  ConfigParam param;

  // setup
  parser.root = YAML::LoadFile(TEST_CONFIG);
  parser.config_loaded = true;

  // BOOL ARRAY
  param.optional = false;
  param.type = BOOL_ARRAY;
  param.key = "bool_array";
  param.data = &b_array;
  parser.loadArray(param);

  MU_CHECK(b_array[0]);
  MU_FALSE(b_array[1]);
  MU_CHECK(b_array[2]);
  MU_FALSE(b_array[3]);

  // INTEGER
  param.optional = false;
  param.type = INT_ARRAY;
  param.key = "int_array";
  param.data = &i_array;
  parser.loadArray(param);

  for (int i = 0; i < 4; i++) {
    MU_CHECK_EQ(i + 1, i_array[i]);
  }

  // FLOAT
  param.optional = false;
  param.type = FLOAT_ARRAY;
  param.key = "float_array";
  param.data = &f_array;
  parser.loadArray(param);

  for (int i = 0; i < 4; i++) {
    MU_CHECK_FLOAT((i + 1) * 1.1, f_array[i]);
  }

  // DOUBLE
  param.optional = false;
  param.type = DOUBLE_ARRAY;
  param.key = "double_array";
  param.data = &d_array;
  parser.loadArray(param);

  for (int i = 0; i < 4; i++) {
    MU_CHECK_FLOAT((i + 1) * 1.1, d_array[i]);
  }

  // STRING
  param.optional = false;
  param.type = STRING_ARRAY;
  param.key = "string_array";
  param.data = &s_array;
  parser.loadArray(param);

  MU_CHECK_EQ("1.1", s_array[0]);
  MU_CHECK_EQ("2.2", s_array[1]);
  MU_CHECK_EQ("3.3", s_array[2]);
  MU_CHECK_EQ("4.4", s_array[3]);

  return 0;
}

int test_ConfigParser_loadVector() {
  Vec2 vec2;
  Vec3 vec3;
  Vec4 vec4;
  VecX vecx;
  ConfigParser parser;
  ConfigParam param;

  // setup
  parser.root = YAML::LoadFile(TEST_CONFIG);
  parser.config_loaded = true;

  // VECTOR 2
  param.optional = false;
  param.type = VEC2;
  param.key = "vector2";
  param.data = &vec2;
  parser.loadVector(param);

  MU_CHECK_FLOAT(1.1, vec2(0));
  MU_CHECK_FLOAT(2.2, vec2(1));

  // VECTOR 3
  param.optional = false;
  param.type = VEC3;
  param.key = "vector3";
  param.data = &vec3;
  parser.loadVector(param);

  MU_CHECK_FLOAT(1.1, vec3(0));
  MU_CHECK_FLOAT(2.2, vec3(1));
  MU_CHECK_FLOAT(3.3, vec3(2));

  // VECTOR 4
  param.optional = false;
  param.type = VEC4;
  param.key = "vector4";
  param.data = &vec4;
  parser.loadVector(param);

  MU_CHECK_FLOAT(1.1, vec4(0));
  MU_CHECK_FLOAT(2.2, vec4(1));
  MU_CHECK_FLOAT(3.3, vec4(2));
  MU_CHECK_FLOAT(4.4, vec4(3));

  // VECTOR X
  param.optional = false;
  param.type = VECX;
  param.key = "vector";
  param.data = &vecx;
  parser.loadVector(param);

  for (int i = 0; i < 9; i++) {
    MU_CHECK_FLOAT((i + 1) * 1.1, vecx(i));
  }

  return 0;
}

int test_ConfigParser_loadMatrix() {
  int index;
  Mat2 mat2;
  Mat3 mat3;
  Mat4 mat4;
  MatX matx;
  cv::Mat cvmat;
  ConfigParser parser;
  ConfigParam param;

  // setup
  parser.root = YAML::LoadFile(TEST_CONFIG);
  parser.config_loaded = true;

  // MATRIX 2
  param.optional = false;
  param.type = MAT2;
  param.key = "matrix2";
  param.data = &mat2;
  parser.loadMatrix(param);

  MU_CHECK_FLOAT(1.1, mat2(0, 0));
  MU_CHECK_FLOAT(2.2, mat2(0, 1));
  MU_CHECK_FLOAT(3.3, mat2(1, 0));
  MU_CHECK_FLOAT(4.4, mat2(1, 1));

  // MATRIX 3
  param.optional = false;
  param.type = MAT3;
  param.key = "matrix3";
  param.data = &mat3;
  parser.loadMatrix(param);

  index = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      MU_CHECK_FLOAT((index + 1) * 1.1, mat3(i, j));
      index++;
    }
  }

  // MATRIX 4
  param.optional = false;
  param.type = MAT4;
  param.key = "matrix4";
  param.data = &mat4;
  parser.loadMatrix(param);

  index = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      MU_CHECK_FLOAT((index + 1) * 1.1, mat4(i, j));
      index++;
    }
  }

  // MATRIX X
  param.optional = false;
  param.type = MATX;
  param.key = "matrix";
  param.data = &matx;
  parser.loadMatrix(param);

  index = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      MU_CHECK_FLOAT((index + 1) * 1.1, matx(i, j));
      index++;
    }
  }

  // CV MATRIX
  param.optional = false;
  param.type = CVMAT;
  param.key = "matrix";
  param.data = &cvmat;
  parser.loadMatrix(param);

  index = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      MU_CHECK_FLOAT((index + 1) * 1.1, cvmat.at<double>(i, j));
      index++;
    }
  }

  return 0;
}

int test_ConfigParser_load() {
  int retval;
  bool b;
  int i;
  float f;
  double d;
  std::string s;

  std::vector<bool> b_array;
  std::vector<int> i_array;
  std::vector<float> f_array;
  std::vector<double> d_array;
  std::vector<std::string> s_array;

  Vec2 vec2;
  Vec3 vec3;
  Vec4 vec4;
  VecX vecx;

  Mat2 mat2;
  Mat3 mat3;
  Mat4 mat4;
  MatX matx;
  cv::Mat cvmat;

  ConfigParser parser;

  parser.addParam("bool", &b);
  parser.addParam("int", &i);
  parser.addParam("float", &f);
  parser.addParam("double", &d);
  parser.addParam("string", &s);

  parser.addParam("bool_array", &b_array);
  parser.addParam("int_array", &i_array);
  parser.addParam("float_array", &f_array);
  parser.addParam("double_array", &d_array);
  parser.addParam("string_array", &s_array);

  parser.addParam("vector2", &vec2);
  parser.addParam("vector3", &vec3);
  parser.addParam("vector4", &vec4);
  parser.addParam("vector", &vecx);

  parser.addParam("matrix2", &mat2);
  parser.addParam("matrix3", &mat3);
  parser.addParam("matrix4", &mat4);
  parser.addParam("matrix", &matx);
  parser.addParam("matrix", &cvmat);

  retval = parser.load(TEST_CONFIG);
  if (retval != 0) {
    return -1;
  }

  std::cout << "bool: " << b << std::endl;
  std::cout << "int: " << i << std::endl;
  std::cout << "float: " << f << std::endl;
  std::cout << "double: " << d << std::endl;
  std::cout << "string: " << s << std::endl;
  std::cout << std::endl;

  std::cout << "vector2: " << vec2.transpose() << std::endl;
  std::cout << "vector3: " << vec3.transpose() << std::endl;
  std::cout << "vector4: " << vec4.transpose() << std::endl;
  std::cout << "vector: " << vecx.transpose() << std::endl;
  std::cout << std::endl;

  std::cout << "matrix2: \n" << mat2 << std::endl;
  std::cout << "matrix3: \n" << mat3 << std::endl;
  std::cout << "matrix4: \n" << mat4 << std::endl;
  std::cout << "matrix: \n" << matx << std::endl;
  std::cout << "cvmatrix: \n" << cvmat << std::endl;
  std::cout << std::endl;

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_ConfigParam_constructor);
  MU_ADD_TEST(test_ConfigParser_constructor);
  MU_ADD_TEST(test_ConfigParser_addParam);
  MU_ADD_TEST(test_ConfigParser_getYamlNode);
  MU_ADD_TEST(test_ConfigParser_loadPrimitive);
  MU_ADD_TEST(test_ConfigParser_loadArray);
  MU_ADD_TEST(test_ConfigParser_loadVector);
  MU_ADD_TEST(test_ConfigParser_loadMatrix);
  MU_ADD_TEST(test_ConfigParser_load);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
