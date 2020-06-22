#ifndef EKF_LOCALIZATION_H
#define EKF_LOCALIZATION_H

#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <ostream>

#include "marker_pose_estimator.h"
#include "odometry.h"
#include "motion_model.h"
#include "measurement_model.h"
#include "maps.h"

class EKFLocalization {
 public:
  EKFLocalization(const YAML::Node& config, const YAML::Node& map,
                  tf2_ros::Buffer* const tf_buffer);

  void AddSensorData(
      const nav_msgs::Odometry::ConstPtr& odom_msg,
      const sensor_msgs::CompressedImageConstPtr& camera_msg);

  const std::vector<Rigid2f>& AllOdometryData() {
      return odometry_.AllOdometryData();
  }

  const std::vector<Pose>& AllPosteriorPoses() { 
      return all_pose_;
  }

 private:
  Rigid3f LookUpCameraToBaseFootprintTransform(const ros::Time time);

  Measurements GetMarkersPosition(
      const Rigid3f& camera_to_base_footprint, 
      const MarkersPose& markers_to_camera);

  Measurements GetMeasurements(
      const sensor_msgs::CompressedImageConstPtr& camera_msg);

  OdometryData GetOdometry(const ros::Time time);
                        
  std::vector<int> GetMeasurementsId(const Measurements& measurements);

  void CorrectionStep(const Measurements& measurements, const Pose& pose);

  tf2_ros::Buffer* const tf_buffer_;
  MarkerPoseEstimator marker_pose_estimator_;
  Odometry odometry_;
  Maps maps_;
  MotionModel motion_model_;
  MeasurementModel measurement_model_;
  
  // posterior pose at time t-1
  Pose pose_;
  std::vector<Pose> all_pose_;
  bool first_sensor_data_;
  Eigen::Matrix3f prior_pose_sigma_;
};
#endif