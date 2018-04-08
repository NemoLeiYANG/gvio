#include "gvio/feature2d/feature_track.hpp"

namespace gvio {

std::ostream &operator<<(std::ostream &os, const FeatureTrack &track) {
  os << "track_id: " << track.track_id << std::endl;
  os << "frame_start: " << track.frame_start << std::endl;
  os << "frame_end: " << track.frame_end << std::endl;
  os << "length: " << track.track.size() << std::endl;
  for (auto f : track.track) {
    os << f;
  }
  os << std::endl;
  return os;
}

int save_feature_track(const FeatureTrack &track,
                       const std::string &output_path) {
  // Setup output file
  std::ofstream output_file(output_path);
  if (output_file.good() == false) {
    LOG_ERROR("Failed to open file for output [%s]", output_path.c_str());
    return -1;
  }

  // Output states
  for (auto t : track.track) {
    output_file << t.kp.pt.x << ",";
    output_file << t.kp.pt.y << std::endl;
  }

  return 0;
}

int save_feature_tracks(const FeatureTracks &tracks,
                        const std::string &output_dir) {

  // Create output dir
  if (create_dir(output_dir) != 0) {
    LOG_ERROR("Failed to create dir [%s]!", output_dir.c_str());
  }

  // Output tracks
  for (auto track : tracks) {
    const std::string track_id = std::to_string(track.track_id);
    const std::string output_file = "track_" + track_id + ".dat";
    const std::string output_path = output_dir + "/" + output_file;

    int retval = save_feature_track(track, output_path);
    if (retval != 0) {
      LOG_ERROR("Failed to save track id [%ld]", track.track_id);
      return -1;
    }
  }

  return 0;
}

} // namespace gvio
