#include "odometry.h"

Odometry::Odometry() 
    : first_(true), noise_mean_(0), noise_stddev_(0),
      seed_(std::chrono::system_clock::now().time_since_epoch().count()),
      generator_(seed_), distribution_(0, noise_stddev_), n_(0) {
  all_data_.reserve(5000);
}

void Odometry::Update(const Rigid2f& odometry) {
  Rigid2f noise_odometry = AddNoise(odometry);
  if (first_) {
    previous_ = noise_odometry;
    current_ = noise_odometry;
    first_ = false;
  } else {
    previous_ = current_;
    current_ = noise_odometry;
  }
  all_data_.push_back(noise_odometry);
}

Rigid2f Odometry::AddNoise(const Rigid2f& odometry) {
  Rigid2f::Vector noise_translate;
  noise_translate(0) = odometry.translation()(0) + n_ * noise_mean_ + 
      distribution_(generator_);
  noise_translate(1) = odometry.translation()(1) + n_ * noise_mean_ + 
      distribution_(generator_);
  float noise_theta = odometry.rotation().angle() + distribution_(generator_);
  n_++;
  return Rigid2f(noise_translate, noise_theta);
}