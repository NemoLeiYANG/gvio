#include "gvio/sim/world.hpp"

namespace gvio {

int SimWorld::configure(const double dt) {
  // Time settings
  this->t = 0.0;
  this->dt = dt;

  // Features
  this->features3d = this->create3DFeaturePerimeter(this->origin,
                                                    this->dimensions,
                                                    this->nb_features);

  // Robot settings
  double circle_radius = 10.0;
  double vx_B = 1.0;
  double wz_B = 0.0;
  circle_trajectory(circle_radius, vx_B, &wz_B, &this->t_end);
  this->robot.w_B(2) = wz_B;
  this->robot.v_B(0) = vx_B;
  this->robot.a_B = zeros(3, 1);

  // Camera settings
  const int image_width = 640;
  const int image_height = 640;
  const double fov = 60.0;
  const double fx = PinholeModel::focalLengthX(image_width, fov);
  const double fy = PinholeModel::focalLengthY(image_height, fov);
  const double cx = image_width / 2.0;
  const double cy = image_height / 2.0;
  this->camera = VirtualCamera(image_width, image_height, fx, fy, cx, cy);

  // Setup output file
  // std::ofstream output_file("/tmp/twowheel.dat");
  // if (output_file.good() == false) {
  //   LOG_ERROR("Failed to open file for output!");
  //   return -1;
  // }
  //
  // const std::string header = "t,x,y,z,roll,pitch,yaw";
  // output_file << header << std::endl;
  //
  // Record initial robot state
  // output_file << 0.0 << ",";
  // output_file << robot.p_G(0) << ",";
  // output_file << robot.p_G(1) << ",";
  // output_file << robot.p_G(2) << ",";
  // output_file << robot.rpy_G(0) << ",";
  // output_file << robot.rpy_G(1) << ",";
  // output_file << robot.rpy_G(2) << std::endl;
  //
  // // Record robot state
  // output_file << t << ",";
  // output_file << robot.p_G(0) << ",";
  // output_file << robot.p_G(1) << ",";
  // output_file << robot.p_G(2) << ",";
  // output_file << robot.rpy_G(0) << ",";
  // output_file << robot.rpy_G(1) << ",";
  // output_file << robot.rpy_G(2) << std::endl;

  return 0;
}

void SimWorld::detectFeatures() {
  // Check what features are observed
  std::vector<int> feature_ids;
  MatX keypoints = camera.observedFeatures(this->features3d,
                                           this->robot.rpy_G,
                                           this->robot.p_G,
                                           feature_ids);

  // Get features lost
  std::vector<int> tracks_lost;
  std::set_difference(this->features_tracking.begin(),
                      this->features_tracking.end(),
                      feature_ids.begin(),
                      feature_ids.end(),
                      std::inserter(tracks_lost, tracks_lost.begin()));

  // Add or updated features observed
  for (size_t i = 0; i < feature_ids.size(); i++) {
    const size_t feature_id = feature_ids[i];
    const Vec2 kp = keypoints.row(i).transpose();
    const Feature f{kp};

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
  }
  this->features_tracking = feature_ids;

  // Remove lost features
  for (auto feature_id : tracks_lost) {
    this->tracks_lost.push_back(this->tracks_tracking[feature_id]);
    this->tracks_tracking.erase(this->tracks_tracking.find(feature_id));
  }
}

FeatureTracks SimWorld::removeLostTracks() {
  FeatureTracks lost_tracks;
  for (auto track : this->tracks_lost) {
    lost_tracks.push_back(track);
  }

  this->tracks_lost.clear();
  return lost_tracks;
}

MatX SimWorld::create3DFeatures(const struct feature_bounds &bounds,
                                const size_t nb_features) {
  // Setup random number generator
  // -- Seed random number engine
  std::random_device rd_x;
  std::random_device rd_y;
  std::random_device rd_z;
  // -- Standard mersenne_twister_engine
  std::mt19937 gen_x(rd_x());
  std::mt19937 gen_y(rd_y());
  std::mt19937 gen_z(rd_z());
  // -- Create uniform real distribution
  std::uniform_real_distribution<> x_dist(bounds.x_min, bounds.x_max);
  std::uniform_real_distribution<> y_dist(bounds.y_min, bounds.y_max);
  std::uniform_real_distribution<> z_dist(bounds.z_min, bounds.z_max);

  // Create random 3D features
  MatX features = zeros(nb_features, 3);
  for (size_t i = 0; i < nb_features; i++) {
    features(i, 0) = x_dist(gen_x);
    features(i, 1) = y_dist(gen_y);
    features(i, 2) = z_dist(gen_z);
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

int SimWorld::step() {
  // Update robot
  this->robot.update(this->dt);
  this->t += this->dt;
  this->time_index++;

  // Update camera
  this->detectFeatures();

  return 0;
}

} // namespace gvio
