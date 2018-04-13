#include "gvio/feature2d/feature.hpp"

namespace gvio {

Feature::Feature() {}

Feature::Feature(const Vec2 &pt) : kp{cv::Point2f(pt(0), pt(1)), 1.0f} {}

Feature::Feature(const Vec2 &pt, const Vec3 &ground_truth)
    : kp{cv::Point2f(pt(0), pt(1)), 1.0f}, ground_truth{ground_truth} {}

Feature::Feature(const cv::Point2f &pt) : kp{pt, 1.0f} {}

Feature::Feature(const cv::KeyPoint &kp) : kp{kp} {}

Feature::Feature(const cv::KeyPoint &kp, const cv::Mat &desc)
    : kp{kp}, desc{desc} {}

void Feature::setTrackID(const TrackID &track_id) { this->track_id = track_id; }

Vec2 Feature::getKeyPoint() { return Vec2{this->kp.pt.x, this->kp.pt.y}; }

Vec2 Feature::getKeyPoint() const { return Vec2{this->kp.pt.x, this->kp.pt.y}; }

std::ostream &operator<<(std::ostream &os, const Feature &f) {
  os << "track_id: " << f.track_id << ", ";
  os << "kp: (" << f.kp.pt.x << ", " << f.kp.pt.y << "), ";
  os << "desc: " << f.desc.size() << std::endl;
  return os;
}

} // namespace gvio
