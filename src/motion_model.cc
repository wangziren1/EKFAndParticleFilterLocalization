#include "motion_model.h"
#include <cmath>
#include <iostream>
using namespace std;

MotionModel::MotionModel(const YAML::Node& config) {
  auto& covariance = config["motion_model_covariance"];
  for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 3; ++i) {
      R_(j, i) = covariance[j*3+i].as<float>();
    }
  }
  cout << "R:\n" << R_ << endl;
  G_x_ = Eigen::Matrix3f::Identity();
  G_u_ = Eigen::Matrix3f::Zero();
  G_u_(2, 0) = 1;
  G_u_(2, 2) = 1;
}

Pose MotionModel::Predict(const Pose& pose, 
                          const OdometryData& odometry_data) {
  ComputeMotionParameter(odometry_data);
  float x = pose.mu_.translation()(0);
  float y = pose.mu_.translation()(1);
  float theta = pose.mu_.rotation().angle();
  Rigid2f mu = ComputeMean(x, y, theta);
  Eigen::Matrix3f sigma = ComputeCovariance(theta, pose.sigma_);
  return Pose{mu, sigma};
}

void MotionModel::ComputeMotionParameter(const OdometryData& odometry_data) {
  Rigid2f current = odometry_data.current_;
  Rigid2f previous = odometry_data.previous_;
  float x_c = current.translation()(0);
  float y_c = current.translation()(1);
  float theta_c = current.rotation().angle();
  float x = previous.translation()(0);
  float y = previous.translation()(1);
  float theta = previous.rotation().angle();
  delta_rot1_ = atan2(y_c - y, x_c - x) - theta;
  delta_trans_ = sqrt(pow((y_c - y), 2) + pow((x_c - x), 2));
  delta_rot2_ = theta_c - theta - delta_rot1_;
  // cout << "atan2(y_c - y, x_c - x):" << atan2(y_c - y, x_c - x) << endl;
  // cout << "theta:" << theta << " theta_c:" << theta_c << endl;
  // cout << "delta_rot1_:" << delta_rot1_ << " delta_trans_:" << delta_trans_ 
  //      << " delta_rot2_:" << delta_rot2_ << endl;  
}

Rigid2f MotionModel::ComputeMean(float x, float y, float theta) {
  float new_x = x + delta_trans_ * cos(theta + delta_rot1_);
  float new_y = y + delta_trans_ * sin(theta + delta_rot1_);
  float new_theta = theta + delta_rot1_ + delta_rot2_;
  Rigid2f new_pose{{new_x, new_y}, new_theta};
  return new_pose;
}

Eigen::Matrix3f MotionModel::ComputeCovariance(
    float theta, 
    const Eigen::Matrix3f& sigma) {
  float s = sin(theta + delta_rot1_);
  float c = cos(theta + delta_rot1_);
  G_x_(0, 2) = - delta_trans_*s;
  G_x_(1, 2) = - delta_trans_*c;
  G_u_(0, 0) = - delta_trans_*s;
  G_u_(1, 0) = - delta_trans_*c;
  G_u_(0, 1) = c;
  G_u_(1, 1) = s;
  // cout << "G_x_:\n" << G_x_ << "\n";
  // cout << "G_u_:\n" << G_u_ << endl;
  Eigen::Matrix3f new_covariance;
  new_covariance = G_x_*sigma*G_x_.transpose() + G_u_*R_*G_u_.transpose();
  return new_covariance;
}