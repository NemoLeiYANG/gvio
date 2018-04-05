#include "gvio/dataset/euroc/ground_truth.hpp"

namespace gvio {

int GroundTruth::load(const std::string &data_dir) {
  // Load ground truth data
  const std::string gnd_data_path = data_dir + "/data.csv";
  MatX data;
  if (csv2mat(gnd_data_path, true, data) != 0) {
    LOG_ERROR("Failed to load ground truth data [%s]!", gnd_data_path.c_str());
    return -1;
  }

  const double t0 = data(0, 0);
  for (long i = 0; i < data.rows(); i++) {
    const long ts = data(i, 0);
    this->timestamps.push_back(ts);
    this->time.push_back(((double) ts - t0) * 1e-9);
    this->p_RS_R.emplace_back(data(i, 1), data(i, 2), data(i, 3));
    this->q_RS.emplace_back(data(i, 5), data(i, 6), data(i, 7), data(i, 4));
    this->v_RS_R.emplace_back(data(i, 8), data(i, 9), data(i, 10));
    this->b_w_RS_S.emplace_back(data(i, 11), data(i, 12), data(i, 13));
    this->b_a_RS_S.emplace_back(data(i, 14), data(i, 15), data(i, 16));
  }

  return 0;
}

} // namespace gvio
