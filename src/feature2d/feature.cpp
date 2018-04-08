#include "gvio/feature2d/feature.hpp"

namespace gvio {

std::ostream &operator<<(std::ostream &os, const Feature &f) {
  os << "track_id: " << f.track_id << ", ";
  os << "kp: (" << f.kp.pt.x << ", " << f.kp.pt.y << "), ";
  os << "desc: " << f.desc.size() << std::endl;
  return os;
}

} // namespace gvio
