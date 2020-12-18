// Copyright (c) 2020 NRSL HITsz. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-3.

#pragma once

#include <opencv/cv.hpp>

#include <glog/logging.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>

#include "transform/timestamped_transform.h"

namespace ros_utils {

std::string GetCurrentPackagePath();
std::string GetDataDirectoryPath();
std::string GetMapDirectoryPath();
std::string GetConfigFilePath();
std::string GetRS80AngleFilePath();

common::Time FromRosTime(const ros::Time &stamp);
ros::Time ToRosTime(const common::Time &time);

template <typename PointType>
sensor_msgs::PointCloud2
ConvertToCloudMsg(const pcl::PointCloud<PointType> &cloud,
                  const std::string &frame_id, const common::Time &time) {
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  cloud_msg.header.stamp.fromNSec(common::ToUniversalNanoseconds(time));

  return cloud_msg;
}

template <typename PointT>
pcl::PointCloud<PointT> ConvertToPclMsg(const sensor_msgs::PointCloud2 &cloud) {
  pcl::PointCloud<PointT> cloud_msg;
  pcl::fromROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = cloud.header.frame_id;
  cloud_msg.header.stamp = cloud.header.stamp.toNSec();
  return cloud_msg;
}

pcl::PointCloud<pcl::PointXYZI>
ConvertToPclMsg(const sensor_msgs::PointCloud2 &cloud);

sensor_msgs::Image ConvertToImageMsg(
    const cv::Mat &image, const std::string &frame_id, const common::Time &time,
    const std::string &encoding = sensor_msgs::image_encodings::BGR8);

cv::Mat ConvertFromImageMsg(const sensor_msgs::Image &image,
                            const std::string &image_encoding);
nav_msgs::Odometry
ConvertToOdomMsg(const transform::TimestampedTransform &timestamped_transform,
                 const std::string &frame_id,
                 const std::string &child_frame_id);

} // namespace ros_utils
