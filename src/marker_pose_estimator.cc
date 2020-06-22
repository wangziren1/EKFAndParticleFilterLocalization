#include "marker_pose_estimator.h"
#include <iostream>
#include <cmath>
#include <gflags/gflags.h>
using namespace std;

std::ostream& operator<<(std::ostream& os, const MarkerPose& markerpose) {
  Eigen::Vector2f position(markerpose.pose_.translation()(0),
                           markerpose.pose_.translation()(2));
  os << "id:" << markerpose.id_ << " [" << position.transpose() << "] "
     "distance:" << position.norm();
  return os;
}

MarkerPoseEstimator::MarkerPoseEstimator(const YAML::Node& config)
    : show_window_(config["show_image"].as<bool>()),
      parameter_(cv::aruco::DetectorParameters::create()),
      dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50)),
      dist_coeffs_(1, 5, CV_32F, cv::Scalar(0)) {
  K_ = (cv::Mat_<double>(3,3) << 1206.8897719532354, 0.0, 960.5, 
                                 0.0, 1206.8897719532354, 540.5, 
                                 0.0, 0.0, 1.0);
  if (show_window_) {
    cv::namedWindow("image", cv::WINDOW_NORMAL);
  }
}

MarkerPoseEstimator::~MarkerPoseEstimator() {
  if (show_window_) {
    cv::destroyWindow("image");
  }
}

MarkersPose MarkerPoseEstimator::ComputeMarkersPose(
    const sensor_msgs::CompressedImageConstPtr& camera_msg) {
  cv::Mat image = cv::imdecode(cv::Mat(camera_msg->data),1);
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f> > marker_corners;
  cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids,
      parameter_);                          
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.1778, K_, dist_coeffs_, 
                                       rvecs, tvecs);
  if (show_window_) {
    ShowImage(image, K_, dist_coeffs_, rvecs, tvecs, marker_ids, marker_corners);
  }
  MarkersPose makers_to_camera = CVPoseToEigenPose(marker_ids, rvecs, tvecs);
  return makers_to_camera;
}

MarkersPose MarkerPoseEstimator::CVPoseToEigenPose(
    const std::vector<int>& marker_ids,
    const std::vector<cv::Vec3d>& rvecs,
    const std::vector<cv::Vec3d>& tvecs) {
  MarkersPose markers_to_camera;
  for (int i = 0; i < marker_ids.size(); ++i) {
    MarkerPose marker_pose;
    marker_pose.id_ = marker_ids[i];
    Rigid3f::Vector t(tvecs[i][0], tvecs[i][1],tvecs[i][2]);
    Rigid3f::AngleAxis angle_axis = ToAngleAxis(rvecs[i]);
    marker_pose.pose_ = Rigid3f(t, angle_axis);
    markers_to_camera.push_back(marker_pose);
  }
  return markers_to_camera;
}

Rigid3f::AngleAxis MarkerPoseEstimator::ToAngleAxis(const cv::Vec3d& rvec) {
  Eigen::Vector3f rotation_vector(rvec[0], rvec[1], rvec[2]);
  float theta = rotation_vector.norm();
  Eigen::Vector3f axis(rotation_vector/theta);
  // Eigen::Vector3f axis1 = rotation_vector.normalized();
  // cout << "mannul:" << axis.transpose() << "\n" << "library:" 
  //      << axis1.transpose() << endl;
  return Rigid3f::AngleAxis(theta, axis);
}

void MarkerPoseEstimator::ShowImage(
    const cv::Mat image, const cv::Mat K, 
    const cv::Mat dist_coeffs,
    const std::vector<cv::Vec3d> rvecs, 
    const std::vector<cv::Vec3d> tvecs,
    const std::vector<int> marker_ids,
    const std::vector<std::vector<cv::Point2f> > marker_corners) {
  cv::Mat output_image = image.clone();
  for (int i = 0; i < rvecs.size(); ++i) {
    auto rvec = rvecs[i];
    auto tvec = tvecs[i];
    cv::aruco::drawAxis(output_image, K, dist_coeffs, rvec, tvec, 0.1);
  }
  cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);
  cv::imshow("image", output_image);
  cv::waitKey(10);
}

void MarkerPoseEstimator::Debug(const MarkersPose& markers_to_camera) {
  cout << "markers -> camera:" << endl;
  for (auto& marker_to_camera : markers_to_camera) {
    cout << marker_to_camera << endl;
  }
}