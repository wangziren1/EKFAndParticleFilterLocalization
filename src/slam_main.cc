#include "node.h"
#include <gflags/gflags.h>

DEFINE_string(configuration_file, "config.yaml", "configuration file");
DEFINE_string(map_file, "slam_map.yaml", "saved map file");

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "ekf_slam_node");
  YAML::Node config = YAML::LoadFile(FLAGS_configuration_file);
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tfListener(tf_buffer);
  SlamNode node(config, FLAGS_map_file, &tf_buffer);
}