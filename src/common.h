#ifndef COMMON_H
#define COMMON_H

#include "rigid_transform.h"
#include <yaml-cpp/yaml.h>
#include <map>

struct Pose {
  Rigid2f mu_;
  Eigen::Matrix3f sigma_;
};
std::ostream& operator<<(std::ostream& os, const Pose& pose);

struct PoseAnother {
  PoseAnother() 
      : mu_(Eigen::Vector3f::Zero()), sigma_(Eigen::Matrix3f::Zero()) {}

  PoseAnother(const Eigen::Vector3f& mu, const Eigen::Matrix3f& sigma) 
      : mu_(mu), sigma_(sigma) {}

  PoseAnother(const PoseAnother& pose)
      : mu_(pose.mu_), sigma_(pose.sigma_) {}

  PoseAnother& operator=(const PoseAnother& pose) {
    mu_ = pose.mu_;
    sigma_ = pose.sigma_;
    return *this;
  }

  Eigen::Vector3f mu_;
  Eigen::Matrix3f sigma_;
};
std::ostream& operator<<(std::ostream& os, const PoseAnother& pose);

PoseAnother ToPoseAnother(const Pose& pose);
Pose FromPoseAnother(const PoseAnother& pose_another);

struct MarkerPosition {
  int id_;
  Eigen::Vector2f position_;
};
std::ostream& operator<<(std::ostream& os, 
                         const MarkerPosition& marker_position);

using Measurements = std::map<int, MarkerPosition>;
std::ostream& operator<<(std::ostream& os, const Measurements& measurements);

struct PredictMarkerPosition {
  int id_;
  Eigen::Vector2f mu_;
  Eigen::Matrix2f sigma_;
  Eigen::Matrix<float, 2, 3> H_;
};
std::ostream& operator<<(std::ostream& os, 
                         const PredictMarkerPosition& predict_marker_position);

using PredictMeasurements = std::map<int, PredictMarkerPosition>;
std::ostream& operator<<(std::ostream& os, 
                         const PredictMeasurements& predict_measurements);

#endif