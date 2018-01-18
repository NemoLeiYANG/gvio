#include "gvio/sim/world.hpp"

namespace gvio {

int SimWorld::configure(const double dt) {
  // Time settings
  this->t = 0.0;
  this->dt = dt;

  // Robot settings
  double circle_radius = 10.0;
  double vx_B = 1.0;
  double wz_B = 0.0;
  circle_trajectory(circle_radius, vx_B, &wz_B, &this->t_end);
  this->robot.w_B(2) = wz_B;
  this->robot.v_B(0) = vx_B;
  this->robot.a_B = zeros(3, 1);

  return 0;
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
  robot.update(this->dt);
  this->t += this->dt;

  return 0;
}

} // namespace gvio
