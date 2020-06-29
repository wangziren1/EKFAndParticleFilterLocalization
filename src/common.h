#ifndef COMMON_H
#define COMMON_H

#include "rigid_transform.h"
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
  Eigen::MatrixXf H_;
};
std::ostream& operator<<(std::ostream& os, 
                         const PredictMarkerPosition& predict_marker_position);

using PredictMeasurements = std::map<int, PredictMarkerPosition>;
std::ostream& operator<<(std::ostream& os, 
                         const PredictMeasurements& predict_measurements);

class State {
 public:
  State(int dim) : dim_(dim) {
    mu_ = Eigen::VectorXf::Zero(dim_);
    sigma_ = Eigen::MatrixXf::Zero(dim_, dim_);
  }

  State(const Eigen::VectorXf& mu, const Eigen::MatrixXf& sigma) 
      : mu_(mu), sigma_(sigma), dim_(mu.size()) {}
  
  int dim() const {return dim_;}

  const Eigen::VectorXf& mu() const {return mu_;}
  const Eigen::MatrixXf& sigma() const {return sigma_;}

  Eigen::Vector2f translation() const {return mu_.head(2);}
  float angle() const {return mu_(2);}
  Rigid2f transform() const {return Rigid2f(translation(), angle());}
  Eigen::Matrix3f pose_sigma() const {return sigma_.block<3, 3>(0, 0);}

  Eigen::Vector2f marker_mu(int id) const {return mu_.segment<2>(3+2*id);}
  Eigen::Matrix2f marker_sigma(int id) const {
    return sigma_.block<2, 2>(3+2*id, 3+2*id);
  }
  void set_map_point(int id, const Eigen::Vector2f& point) {
    mu_.segment<2>(3+2*id) = point;
  }

  Eigen::VectorXf& mutable_mu() {return mu_;}
  Eigen::MatrixXf& mutable_sigma() {return sigma_;}

  State& operator=(const State& state) {
    dim_ = state.dim();
    mu_ = state.mu();
    sigma_ = state.sigma();
    return *this;
  }

 private:
  int dim_;
  Eigen::VectorXf mu_;
  Eigen::MatrixXf sigma_;
};
std::ostream& operator<<(std::ostream& os, const State& state);

struct MarkerState {
  int id;
  Eigen::Vector2f mu;
  Eigen::Matrix2f sigma;
};
std::ostream& operator<<(std::ostream& os, const MarkerState& marker_state);

#endif