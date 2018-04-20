#include "gvio/feature2d/feature_track.hpp"

namespace gvio {

FeatureTrack::FeatureTrack() {}

FeatureTrack::FeatureTrack(const TrackID &track_id,
                           const FrameID &frame_id,
                           const Feature &f1,
                           const Feature &f2)
    : type{MONO_TRACK}, track_id{track_id}, frame_start{frame_id - 1},
      frame_end{frame_id}, track{f1, f2} {}

FeatureTrack::FeatureTrack(const TrackID &track_id,
                           const FrameID &frame_start,
                           const FrameID &frame_end,
                           const Features &track0,
                           const Features &track1)
    : type{STEREO_TRACK}, track_id{track_id}, frame_start{frame_start},
      frame_end{frame_end}, track0{track0}, track1{track1} {}

void FeatureTrack::update(const FrameID &frame_id, const Feature &data) {
  this->frame_end = frame_id;
  this->track.push_back(data);
}

void FeatureTrack::updateStereo(const FrameID &frame_id,
                                const Feature &cam0_f,
                                const Feature &cam1_f) {
  this->frame_end = frame_id;
  this->track0.push_back(cam0_f);
  this->track1.push_back(cam1_f);
}

void FeatureTrack::slice(const size_t frame_start, const size_t frame_end) {
  const size_t diff_start = (frame_start - this->frame_start);
  const size_t diff_end = (this->frame_end - frame_end);
  this->frame_start += diff_start;
  this->frame_end -= diff_end;
  assert(this->frame_start >= 0);
  assert(this->frame_end > this->frame_start);

  const auto first = this->track.begin() + diff_start;
  const auto last = this->track.end() - diff_end;
  const std::vector<Feature> track_sliced(first, last);
  this->track = track_sliced;
}

Feature &FeatureTrack::last() { return this->track.back(); }

size_t FeatureTrack::trackedLength() {
  if (this->type == MONO_TRACK) {
    return this->track.size();
  } else {
    return this->track0.size();
  }
}

size_t FeatureTrack::trackedLength() const {
  if (this->type == MONO_TRACK) {
    return this->track.size();
  } else {
    return this->track0.size();
  }
}

std::ostream &operator<<(std::ostream &os, const FeatureTrack &track) {
  os << "track_id: " << track.track_id << std::endl;
  os << "frame_start: " << track.frame_start << std::endl;
  os << "frame_end: " << track.frame_end << std::endl;
  os << "related: " << track.related << std::endl;
  os << "length: " << track.trackedLength() << std::endl;
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
  if (track.type == MONO_TRACK) {
    for (auto t : track.track) {
      output_file << t.kp.pt.x << ",";
      output_file << t.kp.pt.y << std::endl;
    }
  } else {
    for (auto t : track.track0) {
      output_file << t.kp.pt.x << ",";
      output_file << t.kp.pt.y << std::endl;
    }
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
