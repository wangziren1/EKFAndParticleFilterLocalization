#include "ekf_slam.h"
#include <iostream>
using namespace std;

EKFSlam::EKFSlam(const YAML::Node& config, tf2_ros::Buffer* const tf_buffer)
    : tf_buffer_(tf_buffer), marker_pose_estimator_(config),
      state_dim_(3 + 2*config["marker_num"].as<int>()), state_(state_dim_),
      slam_motion_model_(config, state_dim_), slam_measurement_model_(config) {
  InitializeState();
}

void EKFSlam::AddSensorData(
      const nav_msgs::Odometry::ConstPtr& odom_msg,
      const sensor_msgs::CompressedImageConstPtr& camera_msg) {
  OdometryData odometry = GetOdometry(camera_msg->header.stamp);
  Measurements measurements = GetMeasurements(camera_msg);
  cout << "prior state:\n" << state_ << endl;
  State predict_state(slam_motion_model_.Predict(state_, odometry));
  cout << "predict state:\n" << predict_state << endl;
  state_ = CorrectionStep(measurements, predict_state);
  CollectState();
  cout << "posterior state:\n" << state_ << endl;
  cout << "---------------------------\n";
  DebugMarkersState();
  cout << endl;
}

void EKFSlam::CollectState() {
  all_poses_state_.push_back(Pose{state_.transform(), state_.pose_sigma()});
  all_markers_state_.clear();
  for (auto id : seen_markers_) {
    MarkerState marker_state{id, state_.marker_mu(id), 
                             state_.marker_sigma(id)};
    all_markers_state_.push_back(marker_state);
  }
}

void EKFSlam::DebugMarkersState() {
  for (int id = 0; id < 20; ++id) {
    MarkerState marker_state{id, state_.marker_mu(id), 
                             state_.marker_sigma(id)};
    cout << marker_state << endl;
  }
}

void EKFSlam::InitializeState() {
  state_.mutable_mu() = Eigen::VectorXf::Zero(state_dim_);
  Eigen::MatrixXf sigma = Eigen::MatrixXf::Identity(state_dim_, state_dim_);
  sigma.block(0, 0, 3, 3) = Eigen::MatrixXf::Zero(3, 3);
  // 100 Don't be too big, or it will be unstable.
  sigma = 100 * sigma;
  state_.mutable_sigma() = sigma;
  // cout << "Initialize state:\n" << "mu:\n" << state_.mu() << "\nsigma:\n"
  //      << state_.sigma() << endl;
}

OdometryData EKFSlam::GetOdometry(const ros::Time time) {
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

Measurements EKFSlam::GetMeasurements(
    const sensor_msgs::CompressedImageConstPtr& camera_msg) {
  Rigid3f camera_to_base_footprint = LookUpCameraToBaseFootprintTransform(
      camera_msg->header.stamp);
  MarkersPose markers_to_camera =
      marker_pose_estimator_.ComputeMarkersPose(camera_msg);
  Measurements measurements = GetMarkersPosition(
      camera_to_base_footprint, markers_to_camera);
  return measurements;
}

Rigid3f EKFSlam::LookUpCameraToBaseFootprintTransform(
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

Measurements EKFSlam::GetMarkersPosition(
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

State EKFSlam::CorrectionStep(const Measurements& measurements,
                              const State& predict_state) {
  State state(predict_state);
  for (auto& m : measurements) {
    int id = m.first;
    MarkerPosition marker_position = m.second;
    if (!seen_markers_.count(id)) {
      InitializeMarkerState(state, marker_position);
      seen_markers_.insert(id);
    }
    PredictMarkerPosition predict_marker = 
        slam_measurement_model_.Predict(state, id);
    Eigen::MatrixXf K = state.sigma()*predict_marker.H_.transpose()*
        predict_marker.sigma_.inverse();
    Eigen::VectorXf mu = state.mu() + K*(marker_position.position_ - 
        predict_marker.mu_);
    Eigen::MatrixXf sigma = (Eigen::MatrixXf::Identity(state_dim_, state_dim_)
        - K*predict_marker.H_)*state.sigma();
    state.mutable_mu() = mu;
    state.mutable_sigma() = sigma;
  }
  return state;
}

void EKFSlam::InitializeMarkerState(State& state,
                                    const MarkerPosition& marker_position) {
  Rigid2f base_footprint_to_map(state.transform());
  Eigen::Vector2f map_point = base_footprint_to_map * 
      marker_position.position_;
  state.set_map_point(marker_position.id_, map_point);
}