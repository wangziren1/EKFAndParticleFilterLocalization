#include "node.h"
#include <iostream>
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
using namespace std;

Node::Node(const YAML::Node& config, const YAML::Node& map,
           tf2_ros::Buffer* tf_buffer)
      : odom_sub_(nh_, "odom", 1),
        camera_sub_(nh_, "/camera/rgb/image_raw/compressed", 1),
        sync_(SyncPolicy(10), odom_sub_, camera_sub_),
        ekf_localization_(config, map, tf_buffer) {
  sync_.registerCallback(boost::bind(&Node::CallBack, this, _1, _2));
  odometry_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "odom_trajectory", 50);
  pose_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "pose_trajectory", 50);
  ros::spin();
}

void Node::CallBack(
    const nav_msgs::Odometry::ConstPtr& odom_msg,
    const sensor_msgs::CompressedImageConstPtr& camera_msg) {
  // SensorsTimeDiff(odom_msg, camera_msg);
  ekf_localization_.AddSensorData(odom_msg, camera_msg);
  Visualize();
}

void Node::SensorsTimeDiff(
    const nav_msgs::Odometry::ConstPtr& odom_msg,
    const sensor_msgs::CompressedImageConstPtr& camera_msg) {
  cout << "odom seq: " << odom_msg->header.seq << " "
       << odom_msg->header.stamp.sec << " " << odom_msg->header.stamp.nsec 
       << "\n";
  cout << "image seq:" 
       << camera_msg->header.seq << " "
       << camera_msg->header.stamp.sec << " " << camera_msg->header.stamp.nsec 
       << "\n";
  int diff;
  if (odom_msg->header.stamp.sec == camera_msg->header.stamp.sec) {
    diff = odom_msg->header.stamp.nsec - camera_msg->header.stamp.nsec;
  } else if (odom_msg->header.stamp.sec > camera_msg->header.stamp.sec) {
    diff = int(1e9) + odom_msg->header.stamp.nsec - 
        camera_msg->header.stamp.nsec;
  } else {
    diff = int(1e9) + camera_msg->header.stamp.nsec - 
        odom_msg->header.stamp.nsec;
  }
  cout << "diff:" << std::abs(double(diff) / 1e9) << endl;
}

void Node::Visualize() {
	visualization_msgs::Marker odometry_line, pose_line;
	odometry_line.header.frame_id = pose_line.header.frame_id = "/odom";
	odometry_line.header.stamp = pose_line.header.stamp = ros::Time::now();
	odometry_line.action = pose_line.action = visualization_msgs::Marker::ADD;
	odometry_line.pose.orientation.w = pose_line.pose.orientation.w = 1.0;
	odometry_line.id = 0;
  pose_line.id = 1;

	odometry_line.type = visualization_msgs::Marker::LINE_STRIP;
  pose_line.type = visualization_msgs::Marker::LINE_STRIP;

	odometry_line.scale.x = 0.02;
  pose_line.scale.x = 0.02;

	odometry_line.color.r = 1;
  odometry_line.color.g = 0;
	odometry_line.color.b = 0;
	odometry_line.color.a = 1;

  pose_line.color.r = 0;
  pose_line.color.g = 1;
	pose_line.color.b = 0;
  pose_line.color.a = 1;

	for (auto& pose : ekf_localization_.AllOdometryData()) {
		geometry_msgs::Point p;
		p.x = pose.translation()(0);
		p.y = pose.translation()(1);
		p.z = 0;
		odometry_line.points.push_back(p);
	}

	for (auto& pose : ekf_localization_.AllPosteriorPoses()) {
		geometry_msgs::Point p;
		p.x = pose.mu_.translation()(0);
		p.y = pose.mu_.translation()(1);
		p.z = 0;
	  pose_line.points.push_back(p);
	}

	odometry_pub_.publish(odometry_line);
	pose_pub_.publish(pose_line);
}

SlamNode::SlamNode(const YAML::Node& config, std::string map_file,
                   tf2_ros::Buffer* tf_buffer)
      : odom_sub_(nh_, "odom", 1),
        camera_sub_(nh_, "/camera/rgb/image_raw/compressed", 1),
        sync_(SyncPolicy(10), odom_sub_, camera_sub_),
        ekf_slam_(config, tf_buffer) {
  sync_.registerCallback(boost::bind(&SlamNode::CallBack, this, _1, _2));
  pose_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "pose_trajectory", 50);
  markers_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "marker_position", 50);
  ros::spin();
}

void SlamNode::CallBack(
    const nav_msgs::Odometry::ConstPtr& odom_msg,
    const sensor_msgs::CompressedImageConstPtr& camera_msg) {
  ekf_slam_.AddSensorData(odom_msg, camera_msg);
  Visualize();
}

void SlamNode::Visualize() {
  visualization_msgs::Marker pose_line, markers_point;
	pose_line.header.frame_id = "/odom";
  markers_point.header.frame_id = "/odom";
	pose_line.header.stamp = ros::Time::now();
  markers_point.header.stamp = ros::Time::now();
	pose_line.action = visualization_msgs::Marker::ADD;
  markers_point.action = visualization_msgs::Marker::ADD;
	pose_line.pose.orientation.w = 1.0;
  markers_point.pose.orientation.w = 1.0;
	pose_line.id = 0;
  markers_point.id = 1;

  pose_line.type = visualization_msgs::Marker::LINE_STRIP;
  markers_point.type = visualization_msgs::Marker::POINTS;

  pose_line.scale.x = 0.02;
  markers_point.scale.x = 0.2;
  markers_point.scale.y = 0.2;

  pose_line.color.g = 1;
  pose_line.color.a = 1;
  markers_point.color.r = 1;
  markers_point.color.a = 1;

	for (auto& pose : ekf_slam_.AllPosesState()) {
		geometry_msgs::Point p;
		p.x = pose.mu_.translation()(0);
		p.y = pose.mu_.translation()(1);
		p.z = 0;
	  pose_line.points.push_back(p);
	}
  for (auto& marker_state : ekf_slam_.AllMarkersState()) {
    geometry_msgs::Point p;
    p.x = marker_state.mu(0);
		p.y = marker_state.mu(1);
		p.z = 0;
    markers_point.points.push_back(p);
  }
	pose_pub_.publish(pose_line);
  markers_pub_.publish(markers_point);
}