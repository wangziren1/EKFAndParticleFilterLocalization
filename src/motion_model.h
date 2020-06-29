#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include "odometry.h"
#include "common.h"
#include <ostream>
#include <yaml-cpp/yaml.h>

class MotionModel {
 public:
  MotionModel(const YAML::Node& config);

  Pose Predict(const Pose& pose, const OdometryData& odometry_data);
 
 private:
  Rigid2f ComputeMean(float x, float y, float theta);

  Eigen::Matrix3f ComputeCovariance(float theta, 
                                    const Eigen::Matrix3f& sigma);

  Eigen::Matrix3f R_;
  float delta_rot1_;
  float delta_trans_;
  float delta_rot2_;
  Eigen::Matrix3f G_x_;
  Eigen::Matrix3f G_u_;
};

class SlamMotionModel {
 public:
  SlamMotionModel(const YAML::Node& config, int state_dim);

  State Predict(const State& state, const OdometryData& odometry_data);
 
 private:
  Eigen::VectorXf ComputeMean(const State& state);

  Eigen::MatrixXf ComputeCovariance(const State& state);

  Eigen::Matrix3f R_;
  float delta_rot1_;
  float delta_trans_;
  float delta_rot2_;
  Eigen::MatrixXf F_;
  Eigen::MatrixXf G_y_;
  Eigen::MatrixXf G_u_;
};

#endif