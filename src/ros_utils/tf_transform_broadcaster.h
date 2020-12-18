// Copyright (c) 2020 NRSL HITsz. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-8-31.

#pragma once

#include <memory>

#include "ros_ros_utils.h"
#include "transform/timestamped_transform.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace ros_utils {

class TfTransformBroadcaster {
public:
  TfTransformBroadcaster(const std::string &frame_id,
                         const std::string &child_frame_id);
  virtual ~TfTransformBroadcaster() = default;

  void Publish(const transform::TimestampedTransform &timestamped_transform);
  void Publish(const nav_msgs::Odometry &odometry);
  void Publish(const tf::Transform &transform, const common::Time &stamp);

private:
  std::string frame_id_;
  std::string child_frame_id_;
  std::unique_ptr<tf::TransformBroadcaster> tf_tb_;
};

class TfStaticTransformBroadcaster {
public:
  TfStaticTransformBroadcaster(const transform::Rigid3d &transform,
                               const std::string &frame_id,
                               const std::string &child_frame_id);
  virtual ~TfStaticTransformBroadcaster() = default;
  void Publish(const common::Time &time);
  //  typedef std::unique_ptr<TfStaticTransformBroadcaster> UniqPtr;

private:
  transform::Rigid3d transform_;
  std::string frame_id_;
  std::string child_frame_id_;

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::unique_ptr<geometry_msgs::TransformStamped> static_transformStamped_;
};
} // namespace ros_utils
