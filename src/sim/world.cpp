#include "gvio/sim/world.hpp"

namespace gvio {

SimWorld::SimWorld() {}

SimWorld::~SimWorld() {
  if (this->gnd_file.good()) {
    this->gnd_file.close();
  }

  if (this->mea_file.good()) {
    this->mea_file.close();
  }

  if (this->est_file.good()) {
    this->est_file.close();
  }

  if (this->cam0_idx_file.good()) {
    this->cam0_idx_file.close();
  }
}

int SimWorld::configure(const double t_end, const double dt) {
  assert(t_end > 0.0);
  assert(dt > 0.0);

  // Time
  this->t_end = t_end;
  this->dt = dt;

  // Camera
  const int image_width = 640;
  const int image_height = 640;
  const double fov = 120.0;
  const double fx = PinholeModel::focalLengthX(image_width, fov);
  const double fy = PinholeModel::focalLengthY(image_height, fov);
  const double cx = image_width / 2.0;
  const double cy = image_height / 2.0;
  this->camera = VirtualCamera(image_width, image_height, fx, fy, cx, cy);

  // Camera motion
  const std::vector<Vec3> pos_points{Vec3{0.0, 0.0, 0.0},
                                     Vec3{0.0, 5.0, 0.0},
                                     Vec3{5.0, 5.0, 1.0},
                                     Vec3{5.0, 0.0, 1.0}};
  const std::vector<Vec3> att_points{Vec3{0.0, 0.0, 0.0},
                                     Vec3{0.0, 0.0, 0.25},
                                     Vec3{0.0, 0.0, 0.75},
                                     Vec3{0.0, 0.0, 1.0}};
  this->camera_motion = CameraMotion(pos_points, att_points, this->t_end);
  this->camera_motion.update(this->t);

  // Features
  this->features3d = this->create3DFeaturePerimeter(this->origin,
                                                    this->dimensions,
                                                    this->nb_features);
  this->detectFeatures();

  // Output directory and files
  if (this->setupOutput() != 0) {
    LOG_ERROR("Failed to setup simulation output directory / files!");
    return -1;
  }

  // Record initial ground truth, measurement and camera id
  this->recordGroundTruth(this->t,
                          this->camera_motion.p_G,
                          this->camera_motion.v_G,
                          this->camera_motion.rpy_G);
  this->recordMeasurement(this->t,
                          this->camera_motion.a_B,
                          this->camera_motion.w_B);

  return 0;
}

int SimWorld::configure(const std::string &config_file) {
  int image_width, image_height;
  double fov;
  MatX p_points;
  MatX a_points;

  // Load config file
  ConfigParser parser;
  // -- Simulation settings
  parser.addParam("t_end", &this->t_end);
  parser.addParam("dt", &this->dt);
  parser.addParam("output_path", &this->output_path);
  // -- Camera settings
  parser.addParam("camera.image_width", &image_width);
  parser.addParam("camera.image_height", &image_height);
  parser.addParam("camera.fov", &fov);
  // -- Camera motion settings
  parser.addParam("camera_motion.pos_points", &p_points);
  parser.addParam("camera_motion.att_points", &a_points);
  if (parser.load(config_file) != 0) {
    LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
    return -1;
  }

  // Assert
  assert(this->t_end > 1.0);
  assert(this->dt > 0.0);
  assert(this->output_path != "");
  assert(image_width > 0);
  assert(image_height > 0);
  assert(fov > 0);
  assert(p_points.size() > 0);
  assert(a_points.size() > 0);

  // Camera
  const double fx = PinholeModel::focalLengthX(image_width, fov);
  const double fy = PinholeModel::focalLengthY(image_height, fov);
  const double cx = image_width / 2.0;
  const double cy = image_height / 2.0;
  this->camera = VirtualCamera(image_width, image_height, fx, fy, cx, cy);

  // Camera motion
  this->camera_motion =
      CameraMotion(mat2vec3(p_points), mat2vec3(a_points), this->t_end);
  this->camera_motion.update(this->t);

  // Features
  this->features3d = this->create3DFeaturePerimeter(this->origin,
                                                    this->dimensions,
                                                    this->nb_features);
  this->detectFeatures();

  // Output directory and files
  if (this->setupOutput() != 0) {
    LOG_ERROR("Failed to setup simulation output directory / files!");
    return -1;
  }

  // Record initial ground truth, measurement and camera id
  this->recordGroundTruth(this->t,
                          this->camera_motion.p_G,
                          this->camera_motion.v_G,
                          this->camera_motion.rpy_G);
  this->recordMeasurement(this->t,
                          this->camera_motion.a_B,
                          this->camera_motion.w_B);

  return 0;
}

int SimWorld::setupOutput() {
  // Setup output files
  // -- Make directory if it does not exist already
  if (dir_exists(this->output_path) == false &&
      dir_create(this->output_path) != 0) {
    LOG_ERROR("Failed to create directory [%s] for simulation!",
              output_path.c_str());
    return -1;
  }
  // -- Ground truth file
  this->gnd_file.open(this->output_path + "/ground_truth.csv");
  if (this->gnd_file.good() == false) {
    LOG_ERROR("Failed to open ground truth file for recording [%s]",
              this->output_path.c_str());
    return -1;
  }
  // -- Estimation file
  this->est_file.open(this->output_path + "/estimate.csv");
  if (this->est_file.good() == false) {
    LOG_ERROR("Failed to open estimate file for recording [%s]",
              this->output_path.c_str());
    return -1;
  }
  // -- Measurement file
  this->mea_file.open(this->output_path + "/measurements.csv");
  if (this->mea_file.good() == false) {
    LOG_ERROR("Failed to open measurement file for recording [%s]",
              this->output_path.c_str());
    return -1;
  }
  // -- Camera 0 file
  dir_create(this->output_path + "/cam0");
  this->cam0_idx_file.open(this->output_path + "/cam0/index.csv");
  if (this->cam0_idx_file.good() == false) {
    LOG_ERROR("Failed to open camera 0 index file for recording [%s]",
              this->output_path.c_str());
    return -1;
  }
  // -- Record 3d features
  mat2csv(this->output_path + "/features.csv", this->features3d);

  // Write header
  // clang-format off
  const std::string est_header = "t,x,y,z,vx,vy,vz,roll,pitch,yaw";
  this->est_file << est_header << std::endl;

  const std::string mea_header = "t,ax_B,ay_B,az_B,wx_B,wy_B,wz_B";
  this->mea_file << mea_header << std::endl;

  const std::string gnd_header = "t,x,y,z,vx,vy,vz,roll,pitch,yaw";
  this->gnd_file << gnd_header << std::endl;

  const std::string cam0_header = "t,frame_id";
  this->cam0_idx_file << cam0_header << std::endl;
  // clang-format on

  return 0;
}

void SimWorld::detectFeatures() {
  // Check what features are observed
  std::vector<int> feature_ids;
  MatX keypoints = camera.observedFeatures(this->features3d,
                                           this->camera_motion.rpy_G,
                                           this->camera_motion.p_G,
                                           feature_ids);

  // Get features lost
  std::vector<int> tracks_lost;
  std::set_difference(this->features_tracking.begin(),
                      this->features_tracking.end(),
                      feature_ids.begin(),
                      feature_ids.end(),
                      std::inserter(tracks_lost, tracks_lost.begin()));

  // Add or updated features observed
  std::vector<size_t> record_feature_ids;
  std::vector<Vec2> record_keypoints;
  std::vector<Vec3> record_landmarks;
  for (size_t i = 0; i < feature_ids.size(); i++) {
    const size_t feature_id = feature_ids[i];
    const Vec2 kp = keypoints.row(i).transpose();
    const Vec2 img_pt = camera.camera_model.pixel2image(kp);
    const Vec3 ground_truth = this->features3d.row(feature_id).transpose();
    const Feature f{img_pt, ground_truth};

    if (this->tracks_tracking.count(feature_id)) {
      // Update feature track
      this->tracks_tracking[feature_id].update(this->time_index, f);

    } else {
      // Add feature track
      FeatureTrack track;
      track.frame_start = this->time_index;
      track.frame_end = this->time_index;
      track.track.push_back(f);
      this->tracks_tracking[feature_id] = track;
    }

    // Keep track of keypoints, landmarks and feature for recording
    record_feature_ids.push_back(feature_id);
    record_keypoints.push_back(kp);
    record_landmarks.push_back(ground_truth);
  }
  this->features_tracking = feature_ids;

  // Remove lost features
  for (auto feature_id : tracks_lost) {
    this->tracks_lost.push_back(this->tracks_tracking[feature_id]);
    this->tracks_tracking.erase(this->tracks_tracking.find(feature_id));
  }

  // Record observed keypoints and landmarks
  this->recordCameraObservation(record_feature_ids,
                                record_keypoints,
                                record_landmarks);

  this->frame_index++;
}

FeatureTracks SimWorld::getLostTracks() {
  FeatureTracks lost_tracks;
  for (auto track : this->tracks_lost) {
    lost_tracks.push_back(track);
  }

  this->tracks_lost.clear();
  return lost_tracks;
}

MatX SimWorld::create3DFeatures(const struct feature_bounds &bounds,
                                const size_t nb_features) {
  // Create random 3D features
  MatX features = zeros(nb_features, 3);
  for (size_t i = 0; i < nb_features; i++) {
    features(i, 0) = randf(bounds.x_min, bounds.x_max);
    features(i, 1) = randf(bounds.y_min, bounds.y_max);
    features(i, 2) = randf(bounds.z_min, bounds.z_max);
  }

  return features;
}

MatX SimWorld::create3DFeaturePerimeter(const Vec3 &origin,
                                        const Vec3 &dimensions,
                                        const size_t nb_features) {
  // Dimension of the outskirt
  const double width = dimensions(0);
  const double length = dimensions(1);
  const double height = dimensions(2);

  // Features per side
  const size_t nb_fps = nb_features / 4.0;

  // Features in the North side
  struct feature_bounds north_bounds;
  north_bounds.x_min = origin(0) - width / 2.0;
  north_bounds.x_max = origin(0) + width / 2.0;
  north_bounds.y_min = origin(1) + length / 2.0;
  north_bounds.y_max = origin(1) + length / 2.0;
  north_bounds.z_min = origin(2) - height / 2.0;
  north_bounds.z_max = origin(2) + height / 2.0;
  const MatX north_features = this->create3DFeatures(north_bounds, nb_fps);

  // Features in the East side
  struct feature_bounds east_bounds;
  east_bounds.x_min = origin(0) + width / 2.0;
  east_bounds.x_max = origin(0) + width / 2.0;
  east_bounds.y_min = origin(1) - length / 2.0;
  east_bounds.y_max = origin(1) + length / 2.0;
  east_bounds.z_min = origin(2) - height / 2.0;
  east_bounds.z_max = origin(2) + height / 2.0;
  const MatX east_features = this->create3DFeatures(east_bounds, nb_fps);

  // Features in the South side
  struct feature_bounds south_bounds;
  south_bounds.x_min = origin(0) - width / 2.0;
  south_bounds.x_max = origin(0) + width / 2.0;
  south_bounds.y_min = origin(1) - length / 2.0;
  south_bounds.y_max = origin(1) - length / 2.0;
  south_bounds.z_min = origin(2) - height / 2.0;
  south_bounds.z_max = origin(2) + height / 2.0;
  const MatX south_features = this->create3DFeatures(south_bounds, nb_fps);

  // Features in the West side
  struct feature_bounds west_bounds;
  west_bounds.x_min = origin(0) - width / 2.0;
  west_bounds.x_max = origin(0) - width / 2.0;
  west_bounds.y_min = origin(1) - length / 2.0;
  west_bounds.y_max = origin(1) + length / 2.0;
  west_bounds.z_min = origin(2) - height / 2.0;
  west_bounds.z_max = origin(2) + height / 2.0;
  const MatX west_features = this->create3DFeatures(west_bounds, nb_fps);

  // Stack features and return
  MatX features;
  features = vstack(north_features, east_features);
  features = vstack(features, south_features);
  features = vstack(features, west_features);
  return features;
}

int SimWorld::recordGroundTruth(const double time,
                                const Vec3 &p_G,
                                const Vec3 &v_G,
                                const Vec3 &rpy_G) {
  assert(this->gnd_file.good());

  // -- Time
  this->gnd_file << time << ",";
  // -- Position
  this->gnd_file << p_G(0) << ",";
  this->gnd_file << p_G(1) << ",";
  this->gnd_file << p_G(2) << ",";
  // -- Velocity
  this->gnd_file << v_G(0) << ",";
  this->gnd_file << v_G(1) << ",";
  this->gnd_file << v_G(2) << ",";
  // -- Roll, pitch and yaw
  this->gnd_file << rpy_G(0) << ",";
  this->gnd_file << rpy_G(1) << ",";
  this->gnd_file << rpy_G(2) << std::endl;

  return 0;
}

int SimWorld::recordMeasurement(const double time,
                                const Vec3 &a_B,
                                const Vec3 &w_B) {
  assert(this->mea_file.good());

  // -- Time
  this->mea_file << time << ",";
  // -- Acceleration
  this->mea_file << a_B(0) << ",";
  this->mea_file << a_B(1) << ",";
  this->mea_file << a_B(2) << ",";
  // -- Angular velocity
  this->mea_file << w_B(0) << ",";
  this->mea_file << w_B(1) << ",";
  this->mea_file << w_B(2) << std::endl;

  return 0;
}

int SimWorld::recordCameraObservation(const std::vector<size_t> &feature_ids,
                                      const std::vector<Vec2> &keypoints,
                                      const std::vector<Vec3> &landmarks) {
  assert(this->cam0_idx_file.good());

  // Add new entry to camera index
  this->cam0_idx_file << this->t << "," << this->frame_index << std::endl;

  // Setup camera observed features file
  std::string frame_index_str = std::to_string(this->frame_index);
  std::string path = this->output_path + "/cam0/" + frame_index_str + ".csv";
  std::ofstream feature_file(path);
  if (feature_file.good() == false) {
    LOG_ERROR("Failed to open file [%s]!", path.c_str());
    return -1;
  }

  // Output observed features
  feature_file << "feature_id,kp_x,kp_y,lm_x,lm_y,lm_z" << std::endl;
  for (size_t i = 0; i < feature_ids.size(); i++) {
    // -- Feature id
    feature_file << feature_ids[i] << ",";
    // -- Keypoint
    feature_file << keypoints[i](0) << ",";
    feature_file << keypoints[i](1) << ",";
    // -- Landmark
    feature_file << landmarks[i](0) << ",";
    feature_file << landmarks[i](1) << ",";
    feature_file << landmarks[i](2) << std::endl;
  }
  feature_file.close();

  return 0;
}

int SimWorld::recordEstimate(const double time,
                             const Vec3 &p_G,
                             const Vec3 &v_G,
                             const Vec3 &rpy_G) {
  assert(this->est_file.good());

  // -- Time
  this->est_file << time << ",";
  // -- Position
  this->est_file << p_G(0) << ",";
  this->est_file << p_G(1) << ",";
  this->est_file << p_G(2) << ",";
  // -- Velocity
  this->est_file << v_G(0) << ",";
  this->est_file << v_G(1) << ",";
  this->est_file << v_G(2) << ",";
  // -- Roll, pitch and yaw
  this->est_file << rpy_G(0) << ",";
  this->est_file << rpy_G(1) << ",";
  this->est_file << rpy_G(2) << std::endl;

  return 0;
}

int SimWorld::step() {
  // Update robot
  this->t += this->dt;
  this->time_index++;

  // Update camera
  this->camera_motion.update(this->t);
  this->detectFeatures();

  // Record ground truth, measurement
  this->recordGroundTruth(this->t,
                          this->camera_motion.p_G,
                          this->camera_motion.v_G,
                          this->camera_motion.rpy_G);
  this->recordMeasurement(this->t,
                          this->camera_motion.a_B,
                          this->camera_motion.w_B);

  return 0;
}

} // namespace gvio
