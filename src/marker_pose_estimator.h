#ifndef MARKER_POSE_H
#define MARKER_POSE_H

#include "rigid_transform.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <yaml-cpp/yaml.h>
#include <ostream>

struct MarkerPose {
  int id_;
  Rigid3f pose_;
};
std::ostream& operator<<(std::ostream& os, const MarkerPose& markerpose);
using MarkersPose = std::vector<MarkerPose>;

class MarkerPoseEstimator {
 public:
  MarkerPoseEstimator(const YAML::Node& config);
  ~MarkerPoseEstimator();

  MarkersPose ComputeMarkersPose(
      const sensor_msgs::CompressedImageConstPtr& camera_msg);

 private:
  MarkersPose CVPoseToEigenPose(const std::vector<int>& marker_ids,
                                const std::vector<cv::Vec3d>& rvecs,
                                const std::vector<cv::Vec3d>& tvecs);
  
  Rigid3f::AngleAxis ToAngleAxis(const cv::Vec3d& rvec);

  void ShowImage(const cv::Mat image, const cv::Mat K, 
                 const cv::Mat dist_coeffs,
                 const std::vector<cv::Vec3d> rvecs,
                 const std::vector<cv::Vec3d> tvecs,
                 const std::vector<int> marker_ids,
                 const std::vector<std::vector<cv::Point2f> > 
                 marker_corners);

  void Debug(const MarkersPose& markers_to_camera);

  bool show_window_;
  cv::Ptr<cv::aruco::DetectorParameters> parameter_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Mat K_;
  cv::Mat dist_coeffs_;
};

#endif