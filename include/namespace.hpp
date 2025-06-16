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
#include "luminous_msgs/msg/camera_info.hpp"
#include "luminous_msgs/msg/compressed_image.hpp"
#include "luminous_msgs/msg/image.hpp"
#include "luminous_msgs/msg/imu.hpp"
#include "luminous_msgs/msg/point_cloud2.hpp"
#include "luminous_msgs/msg/odometry.hpp"
#include "luminous_msgs/msg/path.hpp"
// clang-format on

namespace luminous_msgs {

inline std_msgs::msg::Header toROS2(
    const luminous_msgs::msg::Header& msg) noexcept {
  std_msgs::msg::Header ros2_msg;
  ros2_msg.stamp = msg.stamp;
  ros2_msg.frame_id = msg.frame_id;
  return ros2_msg;
}

inline luminous_msgs::msg::Header fromROS2(
    const std_msgs::msg::Header& msg) noexcept {
  luminous_msgs::msg::Header luminous_msg;
  luminous_msg.stamp = msg.stamp;
  luminous_msg.frame_id = msg.frame_id;
  return luminous_msg;
}

inline std_msgs::msg::Header toROS2(
    luminous_msgs::msg::Header&& msg) noexcept {
  std_msgs::msg::Header ros2_msg;
  ros2_msg.stamp = std::move(msg.stamp);
  ros2_msg.frame_id = std::move(msg.frame_id);
  return ros2_msg;
}

inline luminous_msgs::msg::Header fromROS2(
    std_msgs::msg::Header&& msg) noexcept {
  luminous_msgs::msg::Header luminous_msg;
  luminous_msg.stamp = std::move(msg.stamp);
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

inline geometry_msgs::msg::PoseStamped toROS2(
    luminous_msgs::msg::PoseStamped&& msg) noexcept {
  geometry_msgs::msg::PoseStamped ros2_msg;
  ros2_msg.header = toROS2(std::move(msg.header));
  ros2_msg.pose = std::move(msg.pose);
  return ros2_msg;
}

inline luminous_msgs::msg::PoseStamped fromROS2(
    geometry_msgs::msg::PoseStamped&& msg) noexcept {
  luminous_msgs::msg::PoseStamped luminous_msg;
  luminous_msg.header = fromROS2(std::move(msg.header));
  luminous_msg.pose = std::move(msg.pose);
  return luminous_msg;
}

inline geometry_msgs::msg::TransformStamped toROS2(
    const luminous_msgs::msg::TransformStamped& msg) noexcept {
  geometry_msgs::msg::TransformStamped ros2_msg;
  ros2_msg.header = toROS2(msg.header);
  ros2_msg.child_frame_id = msg.child_frame_id;
  ros2_msg.transform = msg.transform;
  return ros2_msg;
}

inline luminous_msgs::msg::TransformStamped fromROS2(
    const geometry_msgs::msg::TransformStamped& msg) noexcept {
  luminous_msgs::msg::TransformStamped luminous_msg;
  luminous_msg.header = fromROS2(msg.header);
  luminous_msg.child_frame_id = msg.child_frame_id;
  luminous_msg.transform = msg.transform;
  return luminous_msg;
}

inline geometry_msgs::msg::TransformStamped toROS2(
    luminous_msgs::msg::TransformStamped&& msg) noexcept {
  geometry_msgs::msg::TransformStamped ros2_msg;
  ros2_msg.header = toROS2(std::move(msg.header));
  ros2_msg.child_frame_id = std::move(msg.child_frame_id);
  ros2_msg.transform = std::move(msg.transform);
  return ros2_msg;
}

inline luminous_msgs::msg::TransformStamped fromROS2(
    geometry_msgs::msg::TransformStamped&& msg) noexcept {
  luminous_msgs::msg::TransformStamped luminous_msg;
  luminous_msg.header = fromROS2(std::move(msg.header));
  luminous_msg.child_frame_id = std::move(msg.child_frame_id);
  luminous_msg.transform = std::move(msg.transform);
  return luminous_msg;
}

inline sensor_msgs::msg::CameraInfo toROS2(
    const luminous_msgs::msg::CameraInfo& msg) noexcept {
  sensor_msgs::msg::CameraInfo ros2_msg;
  ros2_msg.header = toROS2(msg.header);
  ros2_msg.height = msg.height;
  ros2_msg.width = msg.width;
  ros2_msg.distortion_model = msg.distortion_model;
  ros2_msg.d = msg.d;
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
  luminous_msg.distortion_model = msg.distortion_model;
  luminous_msg.d = msg.d;
  luminous_msg.k = msg.k;
  luminous_msg.r = msg.r;
  luminous_msg.p = msg.p;
  luminous_msg.binning_x = msg.binning_x;
  luminous_msg.binning_y = msg.binning_y;
  luminous_msg.roi = msg.roi;
  return luminous_msg;
}

inline sensor_msgs::msg::CameraInfo toROS2(
    luminous_msgs::msg::CameraInfo&& msg) noexcept {
  sensor_msgs::msg::CameraInfo ros2_msg;
  ros2_msg.header = toROS2(std::move(msg.header));
  ros2_msg.height = msg.height;
  ros2_msg.width = msg.width;
  ros2_msg.distortion_model = std::move(msg.distortion_model);
  ros2_msg.d = std::move(msg.d);
  ros2_msg.k = std::move(msg.k);
  ros2_msg.r = std::move(msg.r);
  ros2_msg.p = std::move(msg.p);
  ros2_msg.binning_x = msg.binning_x;
  ros2_msg.binning_y = msg.binning_y;
  ros2_msg.roi = std::move(msg.roi); // Struct, move if members are movable
  return ros2_msg;
}

inline luminous_msgs::msg::CameraInfo fromROS2(
    sensor_msgs::msg::CameraInfo&& msg) noexcept {
  luminous_msgs::msg::CameraInfo luminous_msg;
  luminous_msg.header = fromROS2(std::move(msg.header));
  luminous_msg.height = msg.height;
  luminous_msg.width = msg.width;
  luminous_msg.distortion_model = std::move(msg.distortion_model);
  luminous_msg.d = std::move(msg.d);
  luminous_msg.k = std::move(msg.k);
  luminous_msg.r = std::move(msg.r);
  luminous_msg.p = std::move(msg.p);
  luminous_msg.binning_x = msg.binning_x;
  luminous_msg.binning_y = msg.binning_y;
  luminous_msg.roi = std::move(msg.roi); // Struct, move if members are movable
  return luminous_msg;
}

inline sensor_msgs::msg::CompressedImage toROS2(
    const luminous_msgs::msg::CompressedImage& msg) noexcept {
  sensor_msgs::msg::CompressedImage ros2_msg;
  ros2_msg.header = toROS2(msg.header);
  ros2_msg.format = msg.format;
  ros2_msg.data = msg.data;
  return ros2_msg;
}

inline luminous_msgs::msg::CompressedImage fromROS2(
    const sensor_msgs::msg::CompressedImage& msg) noexcept {
  luminous_msgs::msg::CompressedImage luminous_msg;
  luminous_msg.header = fromROS2(msg.header);
  luminous_msg.format = msg.format;
  luminous_msg.data = msg.data;
  return luminous_msg;
}

inline sensor_msgs::msg::CompressedImage toROS2(
    luminous_msgs::msg::CompressedImage&& msg) noexcept {
  sensor_msgs::msg::CompressedImage ros2_msg;
  ros2_msg.header = toROS2(std::move(msg.header));
  ros2_msg.format = std::move(msg.format);
  ros2_msg.data = std::move(msg.data);
  return ros2_msg;
}

inline luminous_msgs::msg::CompressedImage fromROS2(
    sensor_msgs::msg::CompressedImage&& msg) noexcept {
  luminous_msgs::msg::CompressedImage luminous_msg;
  luminous_msg.header = fromROS2(std::move(msg.header));
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
  ros2_msg.encoding = msg.encoding;
  ros2_msg.is_bigendian = msg.is_bigendian;
  ros2_msg.step = msg.step;
  ros2_msg.data = msg.data;
  return ros2_msg;
}

inline luminous_msgs::msg::Image fromROS2(
    const sensor_msgs::msg::Image& msg) noexcept {
  luminous_msgs::msg::Image luminous_msg;
  luminous_msg.header = fromROS2(msg.header);
  luminous_msg.height = msg.height;
  luminous_msg.width = msg.width;
  luminous_msg.encoding = msg.encoding;
  luminous_msg.is_bigendian = msg.is_bigendian;
  luminous_msg.step = msg.step;
  luminous_msg.data = msg.data;
  return luminous_msg;
}

inline sensor_msgs::msg::Image toROS2(
    luminous_msgs::msg::Image&& msg) noexcept {
  sensor_msgs::msg::Image ros2_msg;
  ros2_msg.header = toROS2(std::move(msg.header));
  ros2_msg.height = msg.height;
  ros2_msg.width = msg.width;
  ros2_msg.encoding = std::move(msg.encoding);
  ros2_msg.is_bigendian = msg.is_bigendian;
  ros2_msg.step = msg.step;
  ros2_msg.data = std::move(msg.data);
  return ros2_msg;
}

inline luminous_msgs::msg::Image fromROS2(
    sensor_msgs::msg::Image&& msg) noexcept {
  luminous_msgs::msg::Image luminous_msg;
  luminous_msg.header = fromROS2(std::move(msg.header));
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

inline sensor_msgs::msg::Imu toROS2(
    luminous_msgs::msg::Imu&& msg) noexcept {
  sensor_msgs::msg::Imu ros2_msg;
  ros2_msg.header = toROS2(std::move(msg.header));
  ros2_msg.orientation = std::move(msg.orientation);
  ros2_msg.orientation_covariance = std::move(msg.orientation_covariance);
  ros2_msg.angular_velocity = std::move(msg.angular_velocity);
  ros2_msg.angular_velocity_covariance = std::move(msg.angular_velocity_covariance);
  ros2_msg.linear_acceleration = std::move(msg.linear_acceleration);
  ros2_msg.linear_acceleration_covariance = std::move(msg.linear_acceleration_covariance);
  return ros2_msg;
}

inline luminous_msgs::msg::Imu fromROS2(
    sensor_msgs::msg::Imu&& msg) noexcept {
  luminous_msgs::msg::Imu luminous_msg;
  luminous_msg.header = fromROS2(std::move(msg.header));
  luminous_msg.orientation = std::move(msg.orientation);
  luminous_msg.orientation_covariance = std::move(msg.orientation_covariance);
  luminous_msg.angular_velocity = std::move(msg.angular_velocity);
  luminous_msg.angular_velocity_covariance = std::move(msg.angular_velocity_covariance);
  luminous_msg.linear_acceleration = std::move(msg.linear_acceleration);
  luminous_msg.linear_acceleration_covariance =
      std::move(msg.linear_acceleration_covariance);
  return luminous_msg;
}

inline sensor_msgs::msg::PointCloud2 toROS2(
    const luminous_msgs::msg::PointCloud2& msg) noexcept {
  sensor_msgs::msg::PointCloud2 ros2_msg;
  ros2_msg.header = toROS2(msg.header);
  ros2_msg.height = msg.height;
  ros2_msg.width = msg.width;
  ros2_msg.fields = msg.fields;
  ros2_msg.is_bigendian = msg.is_bigendian;
  ros2_msg.point_step = msg.point_step;
  ros2_msg.row_step = msg.row_step;
  ros2_msg.data = msg.data;
  ros2_msg.is_dense = msg.is_dense;
  return ros2_msg;
}

inline luminous_msgs::msg::PointCloud2 fromROS2(
    const sensor_msgs::msg::PointCloud2& msg) noexcept {
  luminous_msgs::msg::PointCloud2 luminous_msg;
  luminous_msg.header = fromROS2(msg.header);
  luminous_msg.height = msg.height;
  luminous_msg.width = msg.width;
  luminous_msg.fields = msg.fields;
  luminous_msg.is_bigendian = msg.is_bigendian;
  luminous_msg.point_step = msg.point_step;
  luminous_msg.row_step = msg.row_step;
  luminous_msg.data = msg.data;
  luminous_msg.is_dense = msg.is_dense;
  return luminous_msg;
}

inline sensor_msgs::msg::PointCloud2 toROS2(
    luminous_msgs::msg::PointCloud2&& msg) noexcept {
  sensor_msgs::msg::PointCloud2 ros2_msg;
  ros2_msg.header = toROS2(std::move(msg.header));
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
    sensor_msgs::msg::PointCloud2&& msg) noexcept {
  luminous_msgs::msg::PointCloud2 luminous_msg;
  luminous_msg.header = fromROS2(std::move(msg.header));
  luminous_msg.height = msg.height;
  luminous_msg.width = msg.width;
  luminous_msg.fields = std::move(msg.fields);
  luminous_msg.is_bigendian = msg.is_bigendian;
  luminous_msg.point_step = msg.point_step;
  luminous_msg.row_step = msg.row_step;
  luminous_msg.data = std::move(msg.data);
  luminous_msg.is_dense = msg.is_dense;
  return luminous_msg;
}

inline nav_msgs::msg::Odometry toROS2(
    const luminous_msgs::msg::Odometry& msg) noexcept {
  nav_msgs::msg::Odometry ros2_msg;
  ros2_msg.header = toROS2(msg.header);
  ros2_msg.child_frame_id = msg.child_frame_id;
  ros2_msg.pose = msg.pose;
  ros2_msg.twist = msg.twist;
  return ros2_msg;
}

inline luminous_msgs::msg::Odometry fromROS2(
    const nav_msgs::msg::Odometry& msg) noexcept {
  luminous_msgs::msg::Odometry luminous_msg;
  luminous_msg.header = fromROS2(msg.header);
  luminous_msg.child_frame_id = msg.child_frame_id;
  luminous_msg.pose = msg.pose;
  luminous_msg.twist = msg.twist;
  return luminous_msg;
}

inline nav_msgs::msg::Odometry toROS2(
    luminous_msgs::msg::Odometry&& msg) noexcept {
  nav_msgs::msg::Odometry ros2_msg;
  ros2_msg.header = toROS2(std::move(msg.header));
  ros2_msg.child_frame_id = std::move(msg.child_frame_id);
  ros2_msg.pose = std::move(msg.pose);
  ros2_msg.twist = std::move(msg.twist);
  return ros2_msg;
}

inline luminous_msgs::msg::Odometry fromROS2(
    nav_msgs::msg::Odometry&& msg) noexcept {
  luminous_msgs::msg::Odometry luminous_msg;
  luminous_msg.header = fromROS2(std::move(msg.header));
  luminous_msg.child_frame_id = std::move(msg.child_frame_id);
  luminous_msg.pose = std::move(msg.pose);
  luminous_msg.twist = std::move(msg.twist);
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

inline nav_msgs::msg::Path toROS2(
    luminous_msgs::msg::Path&& msg) noexcept {
  nav_msgs::msg::Path ros2_msg;
  ros2_msg.header = toROS2(std::move(msg.header));
  ros2_msg.poses.reserve(msg.poses.size());
  for (auto&& pose : msg.poses) {
    ros2_msg.poses.emplace_back(toROS2(std::move(pose)));
  }
  // msg.poses is now a vector of moved-from luminous_msgs::msg::PoseStamped objects
  return ros2_msg;
}

inline luminous_msgs::msg::Path fromROS2(
    nav_msgs::msg::Path&& msg) noexcept {
  luminous_msgs::msg::Path luminous_msg;
  luminous_msg.header = fromROS2(std::move(msg.header));
  luminous_msg.poses.reserve(msg.poses.size());
  for (auto&& pose : msg.poses) {
    luminous_msg.poses.emplace_back(fromROS2(std::move(pose)));
  }
  // msg.poses is now a vector of moved-from geometry_msgs::msg::PoseStamped objects
  return luminous_msg;
}

}  // namespace luminous_msgs
