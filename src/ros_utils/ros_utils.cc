// Copyright (c) 2020 NRSL HITsz. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-3.

#include "common/constants.h"
#include "common/file_path.h"
#include "ros_ros_utils.h"

namespace ros_utils {

std::string GetCurrentPackagePath() {
  return ros::package::getPath(kCurrentPackageName);
}

std::string GetDataDirectoryPath() {
  return common::PathJoin(ros_utils::GetCurrentPackagePath(),
                          kRelativeDataDirPath);
}

std::string GetMapDirectoryPath() {
  return common::PathJoin(ros_utils::GetCurrentPackagePath(),
                          kRelativeMapDirPath);
}

std::string GetConfigFilePath() {
  return common::PathJoin(ros_utils::GetCurrentPackagePath(),
                          kRelativeLongTermSlamParamsFilePath);
}

std::string GetRS80AngleFilePath() {
  return common::PathJoin(ros_utils::GetCurrentPackagePath(),
                          kRelativeRs80AngleFilePath);
}

common::Time FromRosTime(const ros::Time &stamp) {
  return common::Time(common::FromSeconds(stamp.toSec()));
}

ros::Time ToRosTime(const common::Time &time) {
  ros::Time stamp;
  stamp.fromNSec(common::ToUniversalNanoseconds(time));
  return stamp;
}

sensor_msgs::Image ConvertToImageMsg(const cv::Mat &image,
                                     const std::string &frame_id,
                                     const common::Time &time,
                                     const std::string &encoding) {
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp.fromNSec(common::ToUniversalNanoseconds(time));
  cv_bridge::CvImage cv_image(header, encoding, image);
  return *cv_image.toImageMsg();
}

cv::Mat ConvertFromImageMsg(const sensor_msgs::Image &image,
                            const std::string &image_encoding) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image, image_encoding);
  cv::Mat img = cv_ptr->image;
  return img;
}

nav_msgs::Odometry
ConvertToOdomMsg(const transform::TimestampedTransform &timestamped_transform,
                 const std::string &frame_id,
                 const std::string &child_frame_id) {
  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = frame_id;
  odom_msg.header.stamp.fromNSec(
      common::ToUniversalNanoseconds(timestamped_transform.time));
  odom_msg.child_frame_id = child_frame_id;

  odom_msg.pose.pose.position.x =
      timestamped_transform.transform.translation().x();
  odom_msg.pose.pose.position.y =
      timestamped_transform.transform.translation().y();
  odom_msg.pose.pose.position.z =
      timestamped_transform.transform.translation().z();
  odom_msg.pose.pose.orientation.x =
      timestamped_transform.transform.rotation().x();
  odom_msg.pose.pose.orientation.y =
      timestamped_transform.transform.rotation().y();
  odom_msg.pose.pose.orientation.z =
      timestamped_transform.transform.rotation().z();
  odom_msg.pose.pose.orientation.w =
      timestamped_transform.transform.rotation().w();

  return odom_msg;
}

} // namespace ros_utils
