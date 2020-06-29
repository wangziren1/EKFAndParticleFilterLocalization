#include "measurement_model.h"
#include <ostream>
#include <cmath>
using namespace std;

MeasurementModel::MeasurementModel(const YAML::Node& node) {
  auto& covariance = node["measurement_model_covariance"];
  for (int j = 0; j < 2; ++j) {
    for (int i = 0; i < 2; ++i) {
      Q_(j, i) = covariance[j*2+i].as<float>();
    }
  }
  cout << "Q:\n" << Q_ << endl;
}
  
PredictMeasurements MeasurementModel::Predict(const MapsData& maps,
                                              const Pose& pose) {
  PredictMeasurements predict_measurements;
  for (auto& m : maps) {
    int id = m.first;
    Eigen::Vector2f mu = pose.mu_.inverse() * m.second.position_;
    auto H = ComputeJacobian(pose, m.second);
    Eigen::Matrix2f sigma = H * pose.sigma_ * H.transpose() + Q_;
    predict_measurements.insert({id, {id, mu, sigma, H}});
  }
  return predict_measurements;
}

Eigen::Matrix<float, 2, 3> MeasurementModel::ComputeJacobian(
    const Pose& pose, const MarkerPosition& marker_position) {
  float cos_theta = cos(pose.mu_.rotation().angle());
  float sin_theta = sin(pose.mu_.rotation().angle());
  float tx = pose.mu_.translation()(0);
  float ty = pose.mu_.translation()(1);
  float mx = marker_position.position_(0);
  float my = marker_position.position_(1);
  Eigen::Matrix<float, 2, 3> H;
  H << -cos_theta, -sin_theta, -sin_theta*(mx-tx) + cos_theta*(my-ty),
       sin_theta,  -cos_theta, -cos_theta*(mx-tx) - sin_theta*(my-ty);
  return H;
}

SlamMeasurementModel::SlamMeasurementModel(const YAML::Node& node) {
  auto& covariance = node["measurement_model_covariance"];
  for (int j = 0; j < 2; ++j) {
    for (int i = 0; i < 2; ++i) {
      Q_(j, i) = covariance[j*2+i].as<float>();
    }
  }
  cout << "Q:\n" << Q_ << endl;
}

PredictMarkerPosition SlamMeasurementModel::Predict(const State& state, 
                                                    int id) {
  Eigen::Vector2f mu = ComputeMean(state, id);
  Eigen::MatrixXf H = ComputeJacobian(state, id);
  Eigen::Matrix2f sigma = H*state.sigma()*H.transpose() + Q_;
  return PredictMarkerPosition{id, mu, sigma, H};
}

Eigen::Vector2f SlamMeasurementModel::ComputeMean(const State& state, int id) {
  Rigid2f base_footprint_to_map(state.transform());
  Rigid2f map_to_base_footprint = base_footprint_to_map.inverse();
  Eigen::Vector2f point(state.marker_mu(id));
  Eigen::Vector2f mu = map_to_base_footprint * point;
  return mu;
}

Eigen::MatrixXf SlamMeasurementModel::ComputeJacobian(const State& state, 
                                                      int id) {
  float cos_theta = cos(state.angle());
  float sin_theta = sin(state.angle());
  float tx = state.translation()(0);
  float ty = state.translation()(1);
  float mx = state.marker_mu(id)(0);
  float my = state.marker_mu(id)(1);
  Eigen::Matrix<float, 2, 3> h1;
  h1 << -cos_theta, -sin_theta, -sin_theta*(mx-tx) + cos_theta*(my-ty),
        sin_theta,  -cos_theta, -cos_theta*(mx-tx) - sin_theta*(my-ty);
  Eigen::Matrix2f h2;
  h2 << cos_theta, sin_theta,
        -sin_theta, cos_theta;
  Eigen::MatrixXf H = Eigen::MatrixXf::Zero(2, state.dim());
  H.block<2, 3>(0, 0) = h1;
  H.block<2, 2>(0, 3+2*id) = h2;
  return H;
}