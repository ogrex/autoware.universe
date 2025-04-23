// Copyright 2021 Apex.AI, Inc.
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "autoware_perception_rviz_plugin/object_detection/detected_objects_with_feature_display.hpp"


#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <rclcpp/duration.hpp>

#include <memory>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
  DetectedObjectsWithFeatureDisplay::DetectedObjectsWithFeatureDisplay() : ObjectPolygonDisplayBase("detected_objects")
{
}

void DetectedObjectsWithFeatureDisplay::processMessage(DetectedObjectsWithFeature::ConstSharedPtr msg)
{
  clear_markers();
  int id = 0;
  for (const auto & feature_object : msg->feature_objects) {
    const auto & cluster = feature_object.feature.cluster;

    // Create a marker to display the cluster point cloud

    auto pointcloud_marker_ptr = std::make_shared<Marker>();
    pointcloud_marker_ptr->header = msg->header;
    pointcloud_marker_ptr->ns = "cluster_point_cloud";
    pointcloud_marker_ptr->id = static_cast<int>(id);
    pointcloud_marker_ptr->type = visualization_msgs::msg::Marker::POINTS;
    pointcloud_marker_ptr->action = visualization_msgs::msg::Marker::ADD;
    pointcloud_marker_ptr->pose.position.x = 0.0;
    pointcloud_marker_ptr->pose.position.y = 0.0;
    pointcloud_marker_ptr->pose.position.z = 0.0;
    pointcloud_marker_ptr->pose.orientation.w = 1.0;
    pointcloud_marker_ptr->scale.x = 0.1;  // Point size
    pointcloud_marker_ptr->scale.y = 0.1;  // Point size
    pointcloud_marker_ptr->color.r = 0.0f;
    pointcloud_marker_ptr->color.g = 1.0f;
    pointcloud_marker_ptr->color.b = 0.0f;
    pointcloud_marker_ptr->color.a = 1.0f;
    pointcloud_marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
    pointcloud_marker_ptr->points.clear();
    pointcloud_marker_ptr->colors.clear();
    pointcloud_marker_ptr->points.reserve(cluster.width * cluster.height);
    pointcloud_marker_ptr->colors.reserve(cluster.width * cluster.height);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cluster, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cluster, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cluster, "z");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(cluster, "intensity");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
      geometry_msgs::msg::Point point;
      point.x = *iter_x;
      point.y = *iter_y;
      point.z = *iter_z;
      pointcloud_marker_ptr->points.push_back(point);

      std_msgs::msg::ColorRGBA color;
      color.r = static_cast<float>(*iter_intensity) / 255.0f;
      color.g = 0.0f;
      color.b = 0.0f;
      color.a = 1.0f;
      pointcloud_marker_ptr->colors.push_back(color);
    }
    add_marker(pointcloud_marker_ptr);
    id++;
  }
}

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  autoware::rviz_plugins::object_detection::DetectedObjectsWithFeatureDisplay, rviz_common::Display)
