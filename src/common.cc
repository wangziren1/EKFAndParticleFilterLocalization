#include "common.h"
#include <iostream>
using namespace std;

std::ostream& operator<<(std::ostream& os, const Pose& pose) {
  os << "pose mean: [" << pose.mu_.translation().transpose() << " " 
     << pose.mu_.rotation().angle() << "]\n";
  os << "pose covariance:\n" << pose.sigma_;
  return os;
}

std::ostream& operator<<(std::ostream& os, const PoseAnother& pose) {
  os << "pose mean: [" << pose.mu_.transpose() << "]\n";
  os << "pose covariance:\n" << pose.sigma_;
  return os;
}

PoseAnother ToPoseAnother(const Pose& pose) {
  PoseAnother pose_another;
  pose_another.mu_(0) = pose.mu_.translation()(0);
  pose_another.mu_(1) = pose.mu_.translation()(1);
  pose_another.mu_(2) = pose.mu_.rotation().angle();
  pose_another.sigma_ = pose.sigma_;
  return pose_another;
}

Pose FromPoseAnother(const PoseAnother& pose_another) {
  Pose pose;
  Rigid2f::Vector translation(pose_another.mu_(0), pose_another.mu_(1));
  float theta = pose_another.mu_(2);
  Rigid2f mu(translation, theta);
  pose.mu_ = mu;
  pose.sigma_ = pose_another.sigma_;
  return pose;
}

std::ostream& operator<<(std::ostream& os, 
                         const MarkerPosition& marker_position) {
  os << "id:" << marker_position.id_ << " [" 
     << marker_position.position_.transpose() << "] distance:" 
     << marker_position.position_.norm();
  return os;
}

std::ostream& operator<<(std::ostream& os, const Measurements& measurements) {
  os << "measurements:";
  for (auto& m : measurements) {
    os << "\n" << m.second;
  }
  return os;
}

std::ostream& operator<<(
    std::ostream& os, 
    const PredictMarkerPosition& predict_marker_position) {
  os << "id:" << predict_marker_position.id_ << " mean: [" 
     << predict_marker_position.mu_.transpose() << "] distance:" 
     << predict_marker_position.mu_.norm() << "\ncovariance:\n" 
     << predict_marker_position.sigma_;
  return os;
}

std::ostream& operator<<(std::ostream& os, 
                         const PredictMeasurements& predict_measurements) {
  os << "predict measurements:";
  for (auto& p : predict_measurements) {
    os << "\n" << p.second;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const State& state) {
  os << "mean: [" << state.mu().head(3).transpose() << "] covariance:\n" 
     << state.sigma().block<3, 3>(0, 0);
  return os;
}

std::ostream& operator<<(std::ostream& os, const MarkerState& marker_state) {
  os << "id: " << marker_state.id << " mean: [" << marker_state.mu.transpose()
     << "] covariance:\n" << marker_state.sigma;
  return os;
}