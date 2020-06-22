#include "maps.h"
#include <iostream>
using namespace std;

Maps::Maps(const YAML::Node& maps) {
  for(YAML::const_iterator it=maps.begin(); it != maps.end(); ++it) {
    int id = it->first.as<int>();
    float x = it->second[0].as<float>();
    float y = it->second[1].as<float>();
    MarkerPosition marker_position{id, {x, y}};
    data_.insert({id, marker_position});
  }
  cout << "maps:" << endl;
  for (auto& p : data_) {
    cout << "id:" << p.first << " [" << p.second.position_(0) << " " 
         << p.second.position_(1) << "]" << endl;
  }
  cout << "------------------------------------------" << endl;
}

MapsData Maps::Subset(const std::vector<int>& ids) {
  MapsData subset_maps;
  for (auto& id : ids) {
    const auto& marker_position = data_[id];
    subset_maps.insert({id, marker_position});
  }
  return subset_maps;
}