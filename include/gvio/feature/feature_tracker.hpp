#ifndef GVIO_FEATURE_HPP
#define GVIO_FEATURE_HPP

#include <stdio.h>
#include <vector>
#include <map>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace gvio {

// Track and frame ID typedefs
using TrackID = long int;
using FrameID = long int;

/**
 * Feature
 */
struct Feature {
  float angle = -1;
  int class_id = -1;
  int octave = 0;
  cv::Point2f pt;
  float response = 0;
  float size = 0;

  TrackID track_id = -1;

  Feature(const cv::KeyPoint &kp) {
    this->angle = kp.angle;
    this->class_id = kp.class_id;
    this->octave = kp.octave;
    this->pt = kp.pt;
    this->response = kp.response;
    this->size = kp.size;
  }

  void setTrackID(const TrackID &track_id) { this->track_id = track_id; }

  cv::KeyPoint asCvKeypoint() {
    return cv::KeyPoint(this->pt,
                        this->size,
                        this->angle,
                        this->response,
                        this->octave,
                        this->class_id);
  }
};

/**
 * Feature track
 */
struct FeatureTrack {
  TrackID track_id;
  FrameID frame_start;
  FrameID frame_end;
  std::vector<Feature> track;

  FeatureTrack(const TrackID &track_id,
               const FrameID &frame_id,
               const Feature &data)
      : track_id{track_id}, frame_start{frame_id - 1}, frame_end{frame_id},
        track{data} {}

  void update(const FrameID &frame_id, const Feature &data) {
    this->frame_end = frame_id;
    this->track.push_back(data);
  }

  int last(Feature &data) {
    data = this->track.back();
    return 0;
  }

  size_t tracked_length() { return this->track.size(); }
};

class FeatureTracker {
public:
  bool configured = false;

  // Frame and track ounters
  size_t counter_frame_id = -1;
  size_t counter_track_id = -1;

  // Feature track book keeping
  std::vector<FeatureTrack> tracking;
  std::vector<TrackID> lost;
  std::map<TrackID, FeatureTrack> buffer;

  // Image, feature, unmatched features book keeping
  cv::Mat img_ref;
  cv::Mat fea_ref;
  cv::Mat unmatched;

  FeatureTracker() {}

  /**
   * Configure
   * @param config_file Path to config file
   */
  int configure(const std::string &config_file);

  /**
   * Detect features
   * @param image Input image
   */
  int detect(const cv::Mat &image);

  /**
   * Add feature track
   */
  int addTrack(const Feature &f1, const Feature &f2);

  /**
   * Remove feature track
   */
  int removeTrack(const TrackID &track_id, const bool lost = false);

  /**
   * Update feature track
   */
  int updateTrack(const TrackID &track_id, const Feature &f);

  /**
   * Match features
   */
  int match();

  /**
   * Process matches
   */
  int processMatches();

  /**
   * Clear old feature tracks
   */
  int clear();

  /**
   * Update feature tracker
   */
  int update();
};

} // namespace gvio
#endif
