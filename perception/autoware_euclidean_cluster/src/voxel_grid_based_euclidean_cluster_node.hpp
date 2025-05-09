// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "autoware/euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp"

#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

namespace autoware::euclidean_cluster
{
class VoxelGridBasedEuclideanClusterNode : public rclcpp::Node
{
public:
  explicit VoxelGridBasedEuclideanClusterNode(const rclcpp::NodeOptions & options);

private:
  void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr cluster_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;

  std::shared_ptr<VoxelGridBasedEuclideanCluster> cluster_;
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_;


  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  void getDebugMarkerArray(
    const std_msgs::msg::Header & header,
    const std::vector<tier4_perception_msgs::msg::DetectedObjectWithFeature> & feature_objects,
    visualization_msgs::msg::MarkerArray & debug_marker_array);
};

}  // namespace autoware::euclidean_cluster
