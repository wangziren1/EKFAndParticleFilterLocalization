#ifndef NODE_H
#define NODE_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "ekf_localization.h"
#include "ekf_slam.h"

class Node {
 public:
  Node(const YAML::Node& config, const YAML::Node& map, 
       tf2_ros::Buffer* tf_buffer);

 private:
  void CallBack(
      const nav_msgs::Odometry::ConstPtr& odom_msg,
      const sensor_msgs::CompressedImageConstPtr& camera_msg);
      
  void SensorsTimeDiff(
    const nav_msgs::Odometry::ConstPtr& odom_msg,
    const sensor_msgs::CompressedImageConstPtr& camera_msg);

  void Visualize();

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, 
      sensor_msgs::CompressedImage> SyncPolicy;
  ros::NodeHandle nh_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::CompressedImage> camera_sub_;
  message_filters::Synchronizer<SyncPolicy> sync_;
  EKFLocalization ekf_localization_;

  ros::Publisher odometry_pub_;
  ros::Publisher pose_pub_;
  ros::WallTimer timer_;
};

class SlamNode {
 public:
  SlamNode(const YAML::Node& config, std::string map_file,
           tf2_ros::Buffer* tf_buffer);

 private:
  void CallBack(
      const nav_msgs::Odometry::ConstPtr& odom_msg,
      const sensor_msgs::CompressedImageConstPtr& camera_msg);

  void Visualize();

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, 
      sensor_msgs::CompressedImage> SyncPolicy;
  ros::NodeHandle nh_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::CompressedImage> camera_sub_;
  message_filters::Synchronizer<SyncPolicy> sync_;
  EKFSlam ekf_slam_;

  ros::Publisher pose_pub_;
  ros::Publisher markers_pub_;
};

#endif