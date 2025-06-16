/*
 * Copyright (c) 2025 Dapeng Feng
 * All rights reserved.
 */

#pragma once

#include <utility>

// clang-format off
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"


#include "luminous_msgs/msg/header.hpp"
#include "luminous_msgs/msg/pose_stamped.hpp"
#include "luminous_msgs/msg/transform_stamped.hpp"
#include "luminous_msgs/msg/odometry.hpp"
#include "luminous_msgs/msg/path.hpp"
#include "luminous_msgs/msg/camera_info.hpp"
#include "luminous_msgs/msg/compressed_image.hpp"
#include "luminous_msgs/msg/image.hpp"
#include "luminous_msgs/msg/imu.hpp"
#include "luminous_msgs/msg/point_cloud2.hpp"
// clang-format on

namespace luminous_msgs {

inline std_msgs::msg::Header toROS2(
    const luminous_msgs::msg::Header& msg) noexcept {
  std_msgs::msg::Header ros2_msg;
  ros2_msg.stamp = msg.stamp;
  ros2_msg.frame_id = std::move(msg.frame_id);
  return ros2_msg;
}

inline luminous_msgs::msg::Header fromROS2(
    const std_msgs::msg::Header& msg) noexcept {
  luminous_msgs::msg::Header luminous_msg;
  luminous_msg.stamp = msg.stamp;
  luminous_msg.frame_id = std::move(msg.frame_id);
  return luminous_msg;
}

inline geometry_msgs::msg::PoseStamped toROS2(
    const luminous_msgs::msg::PoseStamped& msg) noexcept {
  geometry_msgs::msg::PoseStamped ros2_msg;
  ros2_msg.header = toROS2(msg.header);
  ros2_msg.pose = msg.pose;
  return ros2_msg;
}

inline luminous_msgs::msg::PoseStamped fromROS2(
    const geometry_msgs::msg::PoseStamped& msg) noexcept {
  luminous_msgs::msg::PoseStamped luminous_msg;
  luminous_msg.header = fromROS2(msg.header);
  luminous_msg.pose = msg.pose;
  return luminous_msg;
}

inline geometry_msgs::msg::TransformStamped toROS2(
    const luminous_msgs::msg::TransformStamped& msg) noexcept {
  geometry_msgs::msg::TransformStamped ros2_msg;
  ros2_msg.header = toROS2(msg.header);
  ros2_msg.child_frame_id = std::move(msg.child_frame_id);
  ros2_msg.transform = msg.transform;
  return ros2_msg;
}

inline luminous_msgs::msg::TransformStamped fromROS2(
    const geometry_msgs::msg::TransformStamped& msg) noexcept {
  luminous_msgs::msg::TransformStamped luminous_msg;
  luminous_msg.header = fromROS2(msg.header);
  luminous_msg.child_frame_id = std::move(msg.child_frame_id);
  luminous_msg.transform = msg.transform;
  return luminous_msg;
}

inline sensor_msgs::msg::CameraInfo toROS2(
    const luminous_msgs::msg::CameraInfo& msg) noexcept {
  sensor_msgs::msg::CameraInfo ros2_msg;
  ros2_msg.header = toROS2(msg.header);
  ros2_msg.height = msg.height;
  ros2_msg.width = msg.width;
  ros2_msg.distortion_model = std::move(msg.distortion_model);
  ros2_msg.d = std::move(msg.d);
  ros2_msg.k = msg.k;
  ros2_msg.r = msg.r;
  ros2_msg.p = msg.p;
  ros2_msg.binning_x = msg.binning_x;
  ros2_msg.binning_y = msg.binning_y;
  ros2_msg.roi = msg.roi;
  return ros2_msg;
}

inline luminous_msgs::msg::CameraInfo fromROS2(
    const sensor_msgs::msg::CameraInfo& msg) noexcept {
  luminous_msgs::msg::CameraInfo luminous_msg;
  luminous_msg.header = fromROS2(msg.header);
  luminous_msg.height = msg.height;
  luminous_msg.width = msg.width;
  luminous_msg.distortion_model = std::move(msg.distortion_model);
  luminous_msg.d = std::move(msg.d);
  luminous_msg.k = msg.k;
  luminous_msg.r = msg.r;
  luminous_msg.p = msg.p;
  luminous_msg.binning_x = msg.binning_x;
  luminous_msg.binning_y = msg.binning_y;
  luminous_msg.roi = msg.roi;
  return luminous_msg;
}

inline sensor_msgs::msg::CompressedImage toROS2(
    const luminous_msgs::msg::CompressedImage& msg) noexcept {
  sensor_msgs::msg::CompressedImage ros2_msg;
  ros2_msg.header = toROS2(msg.header);
  ros2_msg.format = std::move(msg.format);
  ros2_msg.data = std::move(msg.data);
  return ros2_msg;
}

inline luminous_msgs::msg::CompressedImage fromROS2(
    const sensor_msgs::msg::CompressedImage& msg) noexcept {
  luminous_msgs::msg::CompressedImage luminous_msg;
  luminous_msg.header = fromROS2(msg.header);
  luminous_msg.format = std::move(msg.format);
  luminous_msg.data = std::move(msg.data);
  return luminous_msg;
}

inline sensor_msgs::msg::Image toROS2(
    const luminous_msgs::msg::Image& msg) noexcept {
  sensor_msgs::msg::Image ros2_msg;
  ros2_msg.header = toROS2(msg.header);
  ros2_msg.height = msg.height;
  ros2_msg.width = msg.width;
  ros2_msg.encoding = std::move(msg.encoding);
  ros2_msg.is_bigendian = msg.is_bigendian;
  ros2_msg.step = msg.step;
  ros2_msg.data = std::move(msg.data);
  return ros2_msg;
}

inline luminous_msgs::msg::Image fromROS2(
    const sensor_msgs::msg::Image& msg) noexcept {
  luminous_msgs::msg::Image luminous_msg;
  luminous_msg.header = fromROS2(msg.header);
  luminous_msg.height = msg.height;
  luminous_msg.width = msg.width;
  luminous_msg.encoding = std::move(msg.encoding);
  luminous_msg.is_bigendian = msg.is_bigendian;
  luminous_msg.step = msg.step;
  luminous_msg.data = std::move(msg.data);
  return luminous_msg;
}

inline sensor_msgs::msg::Imu toROS2(
    const luminous_msgs::msg::Imu& msg) noexcept {
  sensor_msgs::msg::Imu ros2_msg;
  ros2_msg.header = toROS2(msg.header);
  ros2_msg.orientation = msg.orientation;
  ros2_msg.orientation_covariance = msg.orientation_covariance;
  ros2_msg.angular_velocity = msg.angular_velocity;
  ros2_msg.angular_velocity_covariance = msg.angular_velocity_covariance;
  ros2_msg.linear_acceleration = msg.linear_acceleration;
  ros2_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance;
  return ros2_msg;
}

inline luminous_msgs::msg::Imu fromROS2(
    const sensor_msgs::msg::Imu& msg) noexcept {
  luminous_msgs::msg::Imu luminous_msg;
  luminous_msg.header = fromROS2(msg.header);
  luminous_msg.orientation = msg.orientation;
  luminous_msg.orientation_covariance = msg.orientation_covariance;
  luminous_msg.angular_velocity = msg.angular_velocity;
  luminous_msg.angular_velocity_covariance = msg.angular_velocity_covariance;
  luminous_msg.linear_acceleration = msg.linear_acceleration;
  luminous_msg.linear_acceleration_covariance =
      msg.linear_acceleration_covariance;
  return luminous_msg;
}

inline sensor_msgs::msg::PointCloud2 toROS2(
    const luminous_msgs::msg::PointCloud2& msg) noexcept {
  sensor_msgs::msg::PointCloud2 ros2_msg;
  ros2_msg.header = toROS2(msg.header);
  ros2_msg.height = msg.height;
  ros2_msg.width = msg.width;
  ros2_msg.fields = std::move(msg.fields);
  ros2_msg.is_bigendian = msg.is_bigendian;
  ros2_msg.point_step = msg.point_step;
  ros2_msg.row_step = msg.row_step;
  ros2_msg.data = std::move(msg.data);
  ros2_msg.is_dense = msg.is_dense;
  return ros2_msg;
}

inline luminous_msgs::msg::PointCloud2 fromROS2(
    const sensor_msgs::msg::PointCloud2& msg) noexcept {
  luminous_msgs::msg::PointCloud2 luminous_msgs;
  luminous_msgs.header = fromROS2(msg.header);
  luminous_msgs.height = msg.height;
  luminous_msgs.width = msg.width;
  luminous_msgs.fields = std::move(msg.fields);
  luminous_msgs.is_bigendian = msg.is_bigendian;
  luminous_msgs.point_step = msg.point_step;
  luminous_msgs.row_step = msg.row_step;
  luminous_msgs.data = std::move(msg.data);
  luminous_msgs.is_dense = msg.is_dense;
  return luminous_msgs;
}

inline nav_msgs::msg::Odometry toROS2(
    const luminous_msgs::msg::Odometry& msg) noexcept {
  nav_msgs::msg::Odometry ros2_msg;
  ros2_msg.header = toROS2(msg.header);
  ros2_msg.child_frame_id = std::move(msg.child_frame_id);
  ros2_msg.pose = msg.pose;
  ros2_msg.twist = msg.twist;
  return ros2_msg;
}

inline luminous_msgs::msg::Odometry fromROS2(
    const nav_msgs::msg::Odometry& msg) noexcept {
  luminous_msgs::msg::Odometry luminous_msg;
  luminous_msg.header = fromROS2(msg.header);
  luminous_msg.child_frame_id = std::move(msg.child_frame_id);
  luminous_msg.pose = msg.pose;
  luminous_msg.twist = msg.twist;
  return luminous_msg;
}

inline nav_msgs::msg::Path toROS2(
    const luminous_msgs::msg::Path& msg) noexcept {
  nav_msgs::msg::Path ros2_msg;
  ros2_msg.header = toROS2(msg.header);
  ros2_msg.poses.reserve(msg.poses.size());
  for (const auto& pose : msg.poses) {
    ros2_msg.poses.emplace_back(toROS2(pose));
  }
  return ros2_msg;
}

inline luminous_msgs::msg::Path fromROS2(
    const nav_msgs::msg::Path& msg) noexcept {
  luminous_msgs::msg::Path luminous_msg;
  luminous_msg.header = fromROS2(msg.header);
  luminous_msg.poses.reserve(msg.poses.size());
  for (const auto& pose : msg.poses) {
    luminous_msg.poses.emplace_back(fromROS2(pose));
  }
  return luminous_msg;
}

}  // namespace luminous_msgs
