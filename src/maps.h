#ifndef MAPS_H
#define MAPS_H

#include "common.h"
#include <yaml-cpp/yaml.h>
#include <map>

using MapsData = std::map<int, MarkerPosition>;

class Maps {
 public:
  Maps(const YAML::Node& maps);
 
  Eigen::Vector2f position(int id) {
    return data_[id].position_;
  }

  MapsData Subset(const std::vector<int>& ids);

 private:
  MapsData data_;
};

#endif