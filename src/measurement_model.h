#ifndef MEASUREMENT_MODEL_H
#define MEASUREMENT_MODEL_H

#include "common.h"
#include "maps.h"
#include <yaml-cpp/yaml.h>

class MeasurementModel {
 public:
  MeasurementModel(const YAML::Node& node);
  
  PredictMeasurements Predict(const MapsData& maps, const Pose& pose);

 private:
  Eigen::Matrix<float, 2, 3> ComputeJacobian(
      const Pose& pose, const MarkerPosition& marker_position);
  
  Eigen::Matrix2f Q_;
};

class SlamMeasurementModel {
 public:
  SlamMeasurementModel(const YAML::Node& node);
  
  PredictMarkerPosition Predict(const State& state, int id);

 private:
  Eigen::Vector2f ComputeMean(const State& state, int id);

  Eigen::MatrixXf ComputeJacobian(const State& state, int id);
  
  Eigen::Matrix2f Q_;
};

#endif