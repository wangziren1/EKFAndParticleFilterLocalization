#include "ekf_localization.h"
#include <iostream>
using namespace std;


EKFLocalization::EKFLocalization(const YAML::Node& config, 
                                 const YAML::Node& maps,
                                 tf2_ros::Buffer* const tf_buffer)
    : tf_buffer_(tf_buffer), marker_pose_estimator_(config), maps_(maps),
      motion_model_(config), measurement_model_(config),
      first_sensor_data_(true) {
  prior_pose_sigma_ << 0.1, 0, 0,
                       0, 0.1, 0,
                       0, 0, 0.1;
  all_pose_.reserve(5000);
}

void EKFLocalization::AddSensorData(
      const nav_msgs::Odometry::ConstPtr& odom_msg,
      const sensor_msgs::CompressedImageConstPtr& camera_msg) {
  Measurements measurements = GetMeasurements(camera_msg);
  OdometryData odometry = GetOdometry(camera_msg->header.stamp);
  if (first_sensor_data_) {
    Pose prior_pose{odometry.current_, prior_pose_sigma_};
    CorrectionStep(measurements, prior_pose);
    first_sensor_data_ = false;
  } else {
    cout << "prior pose:\n" << pose_ << endl;
    Pose predict_pose = motion_model_.Predict(pose_, odometry);
    CorrectionStep(measurements, predict_pose);
  }
  cout << endl;
}

Rigid3f EKFLocalization::LookUpCameraToBaseFootprintTransform(
    const ros::Time time) {
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tf_buffer_->lookupTransform("base_footprint",
                                                   "camera_rgb_optical_frame",
                                                   time);
    geometry_msgs::Vector3 translation = transformStamped.transform.translation;
    geometry_msgs::Quaternion rotation = transformStamped.transform.rotation;
    Eigen::Quaternion<float> q(rotation.w, rotation.x, rotation.y, rotation.z);
    Eigen::Vector3f t(translation.x, translation.y, translation.z);
    Rigid3f transform(t, q);
    return transform;
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }
}

Measurements EKFLocalization::GetMarkersPosition(
    const Rigid3f& camera_to_base_footprint, 
    const MarkersPose& markers_to_camera) {
  Measurements measurements;
  for (auto& marker_to_camera : markers_to_camera) {
    Rigid3f marker_to_base_footprint = camera_to_base_footprint *
        marker_to_camera.pose_;
    MarkerPosition marker_position;
    marker_position.id_ = marker_to_camera.id_;
    marker_position.position_ = Eigen::Vector2f(
        marker_to_base_footprint.translation()(0), 
        marker_to_base_footprint.translation()(1));
    measurements.insert({marker_position.id_, marker_position});
  }
  // cout << measurements;
  return measurements;
}

Measurements EKFLocalization::GetMeasurements(
    const sensor_msgs::CompressedImageConstPtr& camera_msg) {
  Rigid3f camera_to_base_footprint = LookUpCameraToBaseFootprintTransform(
      camera_msg->header.stamp);
  MarkersPose markers_to_camera =
      marker_pose_estimator_.ComputeMarkersPose(camera_msg);
  Measurements measurements = GetMarkersPosition(
      camera_to_base_footprint, markers_to_camera);
  return measurements;
}

OdometryData EKFLocalization::GetOdometry(const ros::Time time) {
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tf_buffer_->lookupTransform("odom", "base_footprint", 
                                                   time);
    // cout << "odom transform time: " << transformStamped.header.stamp.sec << " "
    //      << transformStamped.header.stamp.nsec << "\n";
    geometry_msgs::Vector3 translation = transformStamped.transform.translation;
    geometry_msgs::Quaternion rotation = transformStamped.transform.rotation;
    Rigid3f::Quaternion q(rotation.w, rotation.x, rotation.y, rotation.z);
    Rigid3f::Vector t(translation.x, translation.y, translation.z);
    Rigid3f transform3d(t, q);
    // cout << "odometry 3d:" << transform3d.DebugString() << endl;
    Rigid2f transform2d = Project2D(transform3d);
    // cout << "project 2d:" << transform2d.DebugString() << endl;
    odometry_.Update(transform2d);
    return odometry_.Data();
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
  }
}

std::vector<int> EKFLocalization::GetMeasurementsId(
    const Measurements& measurements) {
  std::vector<int> ids;
  for (auto& m : measurements) {
    ids.push_back(m.first);
  }
  return ids;
}

void EKFLocalization::CorrectionStep(const Measurements& measurements,
                                     const Pose& pose) {
  auto map_subset = maps_.Subset(GetMeasurementsId(measurements));
  PredictMeasurements predict_measurements = measurement_model_.Predict(
      map_subset, pose);
  PoseAnother predict_pose(ToPoseAnother(pose));
  PoseAnother posterior_pose;
  cout << "predict pose:\n" << predict_pose << endl;
  for (auto& m : measurements) {
    int id = m.first;
    PredictMarkerPosition predict_m = predict_measurements[id];
    Eigen::Matrix<float, 3, 2> K = predict_pose.sigma_ * 
        predict_m.H_.transpose() * predict_m.sigma_.inverse();
    Eigen::Vector2f m_position = m.second.position_;
    Eigen::Vector3f posterior_mu = predict_pose.mu_ + K * (m_position - 
        predict_m.mu_);
    Eigen::Matrix3f posterior_sigma = (Eigen::Matrix3f::Identity() - 
        K * predict_m.H_) * predict_pose.sigma_;
    posterior_pose = PoseAnother(posterior_mu, posterior_sigma);
    predict_pose = posterior_pose;
    // cout << "id:" << id << "\n" << posterior_pose << endl;
  }
  cout << "posterior pose:\n" << posterior_pose << endl;
  pose_ = FromPoseAnother(posterior_pose);
  all_pose_.push_back(pose_);
}