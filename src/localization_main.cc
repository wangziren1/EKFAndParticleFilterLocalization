#include "node.h"
#include <gflags/gflags.h>

DEFINE_string(configuration_file, "config.yaml", "configuration file");
DEFINE_string(map_file, "map.yaml", "map file");

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "ekf_localization_node");
  YAML::Node config = YAML::LoadFile(FLAGS_configuration_file);
  YAML::Node map = YAML::LoadFile(FLAGS_map_file);
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tfListener(tf_buffer);
  Node node(config, map, &tf_buffer);
}