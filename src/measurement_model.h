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

#endif