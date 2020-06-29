#include "motion_model.h"
#include <cmath>
#include <iostream>
using namespace std;

Eigen::Vector3f ComputeMotionParameter(const OdometryData& odometry_data) {
  Rigid2f current = odometry_data.current_;
  Rigid2f previous = odometry_data.previous_;
  float x_c = current.translation()(0);
  float y_c = current.translation()(1);
  float theta_c = current.rotation().angle();
  float x = previous.translation()(0);
  float y = previous.translation()(1);
  float theta = previous.rotation().angle();
  float delta_rot1 = atan2(y_c - y, x_c - x) - theta;
  float delta_trans = sqrt(pow((y_c - y), 2) + pow((x_c - x), 2));
  float delta_rot2 = theta_c - theta - delta_rot1;
  // cout << "atan2(y_c - y, x_c - x):" << atan2(y_c - y, x_c - x) << endl;
  // cout << "theta:" << theta << " theta_c:" << theta_c << endl;
  // cout << "delta_rot1_:" << delta_rot1_ << " delta_trans_:" << delta_trans_ 
  //      << " delta_rot2_:" << delta_rot2_ << endl;
  return Eigen::Vector3f(delta_rot1, delta_trans, delta_rot2);
}

Eigen::Matrix3f getMotionModelCovariance(const YAML::Node& covariance) {
  Eigen::Matrix3f R;
  for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < 3; ++i) {
      R(j, i) = covariance[j*3+i].as<float>();
    }
  }
  return R;
}

MotionModel::MotionModel(const YAML::Node& config) {
  auto& covariance = config["motion_model_covariance"];
  R_ = getMotionModelCovariance(covariance);
  cout << "R:\n" << R_ << endl;
  G_x_ = Eigen::Matrix3f::Identity();
  G_u_ = Eigen::Matrix3f::Zero();
  G_u_(2, 0) = 1;
  G_u_(2, 2) = 1;
}

Pose MotionModel::Predict(const Pose& pose, 
                          const OdometryData& odometry_data) {
  Eigen::Vector3f motion_parameter = ComputeMotionParameter(odometry_data);
  delta_rot1_ = motion_parameter(0);
  delta_trans_ = motion_parameter(1);
  delta_rot2_ = motion_parameter(2);
  float x = pose.mu_.translation()(0);
  float y = pose.mu_.translation()(1);
  float theta = pose.mu_.rotation().angle();
  Rigid2f mu = ComputeMean(x, y, theta);
  Eigen::Matrix3f sigma = ComputeCovariance(theta, pose.sigma_);
  return Pose{mu, sigma};
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
  G_x_(1, 2) = delta_trans_*c;
  G_u_(0, 0) = - delta_trans_*s;
  G_u_(1, 0) = delta_trans_*c;
  G_u_(0, 1) = c;
  G_u_(1, 1) = s;
  // cout << "G_x_:\n" << G_x_ << "\n";
  // cout << "G_u_:\n" << G_u_ << endl;
  Eigen::Matrix3f new_covariance;
  new_covariance = G_x_*sigma*G_x_.transpose() + G_u_*R_*G_u_.transpose();
  return new_covariance;
}

SlamMotionModel::SlamMotionModel(const YAML::Node& config, int state_dim)
    : F_(3, state_dim), G_y_(state_dim, state_dim), 
      G_u_(state_dim, state_dim) {
  auto& covariance = config["motion_model_covariance"];
  R_ = getMotionModelCovariance(covariance);
  cout << "R:\n" << R_ << endl;
  F_.setZero();
  F_.block(0, 0, 3, 3).setIdentity();
}

State SlamMotionModel::Predict(const State& state, 
                               const OdometryData& odometry_data) {
  Eigen::Vector3f motion_parameter = ComputeMotionParameter(odometry_data);
  delta_rot1_ = motion_parameter(0);
  delta_trans_ = motion_parameter(1);
  delta_rot2_ = motion_parameter(2);
  Eigen::VectorXf mu = ComputeMean(state);
  Eigen::MatrixXf sigma = ComputeCovariance(state);
  State predict_state(mu, sigma);
  // cout << predict_state << endl;
  return predict_state;
}

Eigen::VectorXf SlamMotionModel::ComputeMean(const State& state) {
  float theta = state.angle();
  Eigen::Vector3f delta;
  delta(0) = delta_trans_ * cos(theta + delta_rot1_);
  delta(1) = delta_trans_ * sin(theta + delta_rot1_);
  delta(2) = delta_rot1_ + delta_rot2_;
  Eigen::VectorXf predict_mu = state.mu();
  // equal to "predict_mu += F_.transpose() * delta"
  predict_mu.head(3) += delta;
  return predict_mu;
}

Eigen::MatrixXf SlamMotionModel::ComputeCovariance(const State& state) {
  float theta = state.angle();
  float s = sin(theta + delta_rot1_);
  float c = cos(theta + delta_rot1_);
  Eigen::Matrix3f dx;
  dx.setZero();
  dx(0, 2) = - delta_trans_*s;
  dx(1, 2) = delta_trans_*c;
  
  Eigen::Matrix3f du;
  du.setZero();
  du(0, 0) = - delta_trans_*s;
  du(1, 0) = delta_trans_*c;
  du(0, 1) = c;
  du(1, 1) = s;
  du(2, 0) = 1;
  du(2, 2) = 1;

  // equal to "G_y_ = I + F_.transpose() * dx * F_"
  G_y_.setIdentity();
  G_y_.block<3, 3>(0, 0) += dx;
  Eigen::MatrixXf predict_sigma = G_y_*state.sigma()*G_y_.transpose();
  predict_sigma.block<3, 3>(0, 0) += du*R_*du.transpose();
  return predict_sigma;
}