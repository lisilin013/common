// Copyright (c) 2020 NRSL HITsz. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-8-31.

#include "ros_tf_transform_broadcaster.h"

namespace ros_utils {

TfTransformBroadcaster::TfTransformBroadcaster(
    const std::string &frame_id, const std::string &child_frame_id)
    : frame_id_(frame_id), child_frame_id_(child_frame_id) {
  tf_tb_ = std::make_unique<tf::TransformBroadcaster>();
}

void TfTransformBroadcaster::Publish(
    const transform::TimestampedTransform &timestamped_transform) {
  tf::Transform transform;
  transform.setOrigin(
      tf::Vector3(timestamped_transform.transform.translation().x(),
                  timestamped_transform.transform.translation().y(),
                  timestamped_transform.transform.translation().z()));
  tf::Quaternion q(timestamped_transform.transform.rotation().x(),
                   timestamped_transform.transform.rotation().y(),
                   timestamped_transform.transform.rotation().z(),
                   timestamped_transform.transform.rotation().w());
  transform.setRotation(q);

  ros::Time timestamp;
  timestamp.fromNSec(
      common::ToUniversalNanoseconds(timestamped_transform.time));
  tf_tb_->sendTransform(
      tf::StampedTransform(transform, timestamp, frame_id_, child_frame_id_));
}

void TfTransformBroadcaster::Publish(const nav_msgs::Odometry &odometry) {
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(odometry.pose.pose.position.x,
                                  odometry.pose.pose.position.y,
                                  odometry.pose.pose.position.z));
  tf::Quaternion q(
      odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y,
      odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w);
  transform.setRotation(q);

  tf_tb_->sendTransform(tf::StampedTransform(transform, odometry.header.stamp,
                                             frame_id_, child_frame_id_));
}

void TfTransformBroadcaster::Publish(const tf::Transform &transform,
                                     const common::Time &stamp) {
  ros::Time timestamp;
  timestamp.fromNSec(common::ToUniversalNanoseconds(stamp));
  tf_tb_->sendTransform(
      tf::StampedTransform(transform, timestamp, frame_id_, child_frame_id_));
}

TfStaticTransformBroadcaster::TfStaticTransformBroadcaster(
    const transform::Rigid3d &transform, const std::string &frame_id,
    const std::string &child_frame_id)
    : transform_(transform), frame_id_(frame_id),
      child_frame_id_(child_frame_id) {
  static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>();
  static_transformStamped_ =
      std::make_unique<geometry_msgs::TransformStamped>();
}

void TfStaticTransformBroadcaster::Publish(const common::Time &time) {
  static_transformStamped_->header.stamp.fromNSec(
      common::ToUniversalNanoseconds(time));
  static_transformStamped_->header.frame_id = frame_id_;
  static_transformStamped_->child_frame_id = child_frame_id_;
  static_transformStamped_->transform.translation.x =
      transform_.translation().x();
  static_transformStamped_->transform.translation.y =
      transform_.translation().y();
  static_transformStamped_->transform.translation.z =
      transform_.translation().z();
  static_transformStamped_->transform.rotation.x = transform_.rotation().x();
  static_transformStamped_->transform.rotation.y = transform_.rotation().y();
  static_transformStamped_->transform.rotation.z = transform_.rotation().z();
  static_transformStamped_->transform.rotation.w = transform_.rotation().w();
  static_broadcaster_->sendTransform(*static_transformStamped_);
}

} // namespace ros_utils
