#ifndef EKF_SLAM_H
#define EKF_SLAM_H

#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <set>

#include "odometry.h"
#include "marker_pose_estimator.h"
#include "motion_model.h"
#include "measurement_model.h"

class EKFSlam {
 public:
  EKFSlam(const YAML::Node& config, tf2_ros::Buffer* const tf_buffer);

  void AddSensorData(
      const nav_msgs::Odometry::ConstPtr& odom_msg,
      const sensor_msgs::CompressedImageConstPtr& camera_msg);

  const std::vector<Pose>& AllPosesState() const {return all_poses_state_;}

  const std::vector<MarkerState>& AllMarkersState() const {
    return all_markers_state_;
  }

 private:
  void InitializeState();

  OdometryData GetOdometry(const ros::Time time);

  Measurements GetMeasurements(
      const sensor_msgs::CompressedImageConstPtr& camera_msg);
  
  Rigid3f LookUpCameraToBaseFootprintTransform(const ros::Time time);

  Measurements GetMarkersPosition(
      const Rigid3f& camera_to_base_footprint, 
      const MarkersPose& markers_to_camera);

  State CorrectionStep(const Measurements& measurements, 
                       const State& predict_state);

  void InitializeMarkerState(State& state, 
                             const MarkerPosition& marker_position);
  
  void CollectState();

  void DebugMarkersState();

  tf2_ros::Buffer* const tf_buffer_;
  Odometry odometry_;
  MarkerPoseEstimator marker_pose_estimator_;
  const int state_dim_;
  State state_;
  SlamMotionModel slam_motion_model_;
  SlamMeasurementModel slam_measurement_model_;
  std::set<int> seen_markers_;
  std::vector<Pose> all_poses_state_;
  std::vector<MarkerState> all_markers_state_;
};
#endif