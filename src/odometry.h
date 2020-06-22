#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "rigid_transform.h"
#include <random>
#include <chrono>
#include <iostream>
using namespace std;

struct OdometryData {
  Rigid2f current_;
  Rigid2f previous_;
};

class Odometry {
 public:
  Odometry();

  void Update(const Rigid2f& odometry);

  OdometryData Data() {
    return OdometryData{current_, previous_};
  }

  const std::vector<Rigid2f>& AllOdometryData() {
    return all_data_;
  }

 private:
  Rigid2f AddNoise(const Rigid2f& odometry);

  bool first_;
  Rigid2f current_;
  Rigid2f previous_;
  std::vector<Rigid2f> all_data_;
  
  // noise
  float noise_mean_;
  float noise_stddev_;
  unsigned seed_;
  std::default_random_engine generator_;
  std::normal_distribution<double> distribution_;
  int n_;
};

#endif