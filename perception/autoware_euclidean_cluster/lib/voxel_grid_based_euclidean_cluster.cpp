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

#include "autoware/euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp"

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <cstddef>
#include <unordered_map>
#include <vector>
#include <chrono>

namespace autoware::euclidean_cluster
{
VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster()
{
}

VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster(
  bool use_height, int min_cluster_size, int max_cluster_size)
: EuclideanClusterInterface(use_height, min_cluster_size, max_cluster_size)
{
}

VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster(
  bool use_height, int min_cluster_size, int max_cluster_size, float tolerance,
  float voxel_leaf_size, int min_points_number_per_voxel)
: EuclideanClusterInterface(use_height, min_cluster_size, max_cluster_size),
  tolerance_(tolerance),
  voxel_leaf_size_(voxel_leaf_size),
  min_points_number_per_voxel_(min_points_number_per_voxel)
{
}
// TODO(badai-nguyen): remove this function when field copying also implemented for
// euclidean_cluster.cpp
bool VoxelGridBasedEuclideanCluster::cluster(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud,
  std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters)
{
  (void)pointcloud;
  (void)clusters;
  return false;
}

bool VoxelGridBasedEuclideanCluster::cluster(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg,
  tier4_perception_msgs::msg::DetectedObjectsWithFeature & objects)
{
  using Clock = std::chrono::steady_clock;
  using us = std::chrono::microseconds;

  // Persistent performance accumulators
  static size_t perf_calls = 0;
  static constexpr size_t STAGES = 7;
  static std::array<uint64_t, STAGES> sum_times = {};
  static std::array<uint64_t, STAGES> min_times;
  static std::array<uint64_t, STAGES> max_times;
  if (perf_calls == 0) {
    for (size_t i = 0; i < STAGES; ++i) {
      min_times[i] = std::numeric_limits<uint64_t>::max();
      max_times[i] = 0;
    }
  }

  std::array<uint64_t, STAGES> durations;
  auto t_start = Clock::now();


  // TODO(Saito) implement use_height is false version
  // 1) Convert ROS PointCloud2 to PCL cloud
  // create voxel
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  int point_step = pointcloud_msg->point_step;
  pcl::fromROSMsg(*pointcloud_msg, *pointcloud);
  auto t1 = Clock::now();
  durations[0] = std::chrono::duration_cast<us>(t1 - t_start).count();
  // 2) Voxel grid filtering
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, 100000.0);
  voxel_grid_.setMinimumPointsNumberPerVoxel(min_points_number_per_voxel_);
  voxel_grid_.setInputCloud(pointcloud);
  voxel_grid_.setSaveLeafLayout(true);
  voxel_grid_.filter(*voxel_map_ptr);
  auto t2 = Clock::now();
  durations[1] = std::chrono::duration_cast<us>(t2 - t1).count();


  // 3) Build 2D centroid cloud
  // voxel is pressed 2d
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2d_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto & point : voxel_map_ptr->points) {
    pcl::PointXYZ point2d;
    point2d.x = point.x;
    point2d.y = point.y;
    point2d.z = 0.0;
    pointcloud_2d_ptr->push_back(point2d);
  }
  auto t3 = Clock::now();
  durations[2] = std::chrono::duration_cast<us>(t3 - t2).count();

  // 4) KD-tree + clustering
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pointcloud_2d_ptr);

  // clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
  pcl_euclidean_cluster.setClusterTolerance(tolerance_);
  pcl_euclidean_cluster.setMinClusterSize(1);
  pcl_euclidean_cluster.setMaxClusterSize(600);
  pcl_euclidean_cluster.setSearchMethod(tree);
  pcl_euclidean_cluster.setInputCloud(pointcloud_2d_ptr);
  pcl_euclidean_cluster.extract(cluster_indices);
  auto t4 = Clock::now();
  durations[3] = std::chrono::duration_cast<us>(t4 - t3).count();

  size_t max_points_in_cluster = 0;

  for (const auto& indices : cluster_indices) {
    if (indices.indices.size() > max_points_in_cluster) {
      max_points_in_cluster = indices.indices.size();
    }
  }
  
  std::cout << "Maximum number of voxel(point) in a cluster: " << max_points_in_cluster << std::endl;
  


  // 5) Buffer preparation
  // create map to search cluster index from voxel grid index
  std::unordered_map</* voxel grid index */ int, /* cluster index */ int> map;
  std::vector<sensor_msgs::msg::PointCloud2> temporary_clusters;  // no check about cluster size
  std::vector<size_t> clusters_data_size;
  temporary_clusters.resize(cluster_indices.size());
  for (size_t cluster_idx = 0; cluster_idx < cluster_indices.size(); ++cluster_idx) {
    const auto & cluster = cluster_indices.at(cluster_idx);
    auto & temporary_cluster = temporary_clusters.at(cluster_idx);
    for (const auto & point_idx : cluster.indices) {
      map[point_idx] = cluster_idx;
    }
    temporary_cluster.height = pointcloud_msg->height;
    temporary_cluster.fields = pointcloud_msg->fields;
    temporary_cluster.point_step = point_step;
    temporary_cluster.data.resize(cluster.indices.size() * point_step);
    clusters_data_size.push_back(0);
  }
  auto t5 = Clock::now();
  durations[4] = std::chrono::duration_cast<us>(t5 - t4).count();


#define FILTER_VOXEL_POINTS

  // 6) Data copy
  // create vector of point cloud cluster. vector index is voxel grid index.

  constexpr int max_points_per_voxel_ = 5;
  std::unordered_map<int, std::unordered_map<int, int>> point_counts_per_voxel_per_cluster;
  for (size_t i = 0; i < pointcloud->points.size(); ++i) {
    const auto & point = pointcloud->points.at(i);
    const int voxel_index =
      voxel_grid_.getCentroidIndexAt(voxel_grid_.getGridCoordinates(point.x, point.y, point.z));
    auto map_it = map.find(voxel_index);

    if (map_it != map.end()) {
#ifdef FILTER_VOXEL_POINTS
      // Track point count per voxel per cluster
      int cluster_idx = map_it->second;
      
      int & voxel_point_count = point_counts_per_voxel_per_cluster[cluster_idx][voxel_index];
      if (voxel_point_count >= max_points_per_voxel_) {
        continue;  // Skip adding this point
      }
      voxel_point_count++;
#endif

      auto & cluster_data_size = clusters_data_size.at(map[voxel_index]);
      // if (
      //   cluster_data_size >
      //   static_cast<std::size_t>(max_cluster_size_) * static_cast<std::size_t>(point_step)) {
      //   continue;
      // }
      std::memcpy(
        &temporary_clusters.at(map[voxel_index]).data[cluster_data_size],
        &pointcloud_msg->data[i * point_step], point_step);
      cluster_data_size += point_step;
      if (cluster_data_size == temporary_clusters.at(map[voxel_index]).data.size()) {
        temporary_clusters.at(map[voxel_index])
          .data.resize(temporary_clusters.at(map[voxel_index]).data.size() * 2);
      }
    }
  }
  auto t6 = Clock::now();
  durations[5] = std::chrono::duration_cast<us>(t6 - t5).count();

  // // === Density estimation per cluster ===
  // std::vector<int> voxel_counts_per_cluster(cluster_indices.size(), 0);
  // std::vector<int> point_counts_per_cluster(cluster_indices.size(), 0);
  // std::unordered_map<int, std::unordered_set<int>> voxel_set_per_cluster;

  // for (size_t i = 0; i < pointcloud->points.size(); ++i) {
  //   const auto & point = pointcloud->points.at(i);
  //   const int voxel_index = voxel_grid_.getCentroidIndexAt(
  //     voxel_grid_.getGridCoordinates(point.x, point.y, point.z));
  //   auto it = map.find(voxel_index);
  //   if (it != map.end()) {
  //     int cluster_idx = it->second;
  //     point_counts_per_cluster[cluster_idx]++;
  //     voxel_set_per_cluster[cluster_idx].insert(voxel_index);
  //   }
  // }
  // // Output density info
  // for (size_t i = 0; i < cluster_indices.size(); ++i) {
  //   int point_count = point_counts_per_cluster[i];
  //   int voxel_count = static_cast<int>(voxel_set_per_cluster[i].size());
  //   if (voxel_count > 0) {
  //     float density = static_cast<float>(point_count) / voxel_count;
  //     std::cout << "[cluster " << i << "] Points: " << point_count
  //               << ", Voxels: " << voxel_count
  //               << ", Density (pts/voxel): " << density << std::endl;
  //   } else {
  //     std::cout << "[cluster " << i << "] No voxels found." << std::endl;
  //   }
  // }

  // // === Max point count per voxel in each cluster ===
  // std::unordered_map<int, std::unordered_map<int, int>> voxel_point_count_per_cluster;
  // // cluster_idx -> (voxel_index -> count)

  // for (size_t i = 0; i < pointcloud->points.size(); ++i) {
  //   const auto & point = pointcloud->points.at(i);
  //   const int voxel_index = voxel_grid_.getCentroidIndexAt(
  //     voxel_grid_.getGridCoordinates(point.x, point.y, point.z));
  //   auto it = map.find(voxel_index);
  //   if (it != map.end()) {
  //     int cluster_idx = it->second;
  //     voxel_point_count_per_cluster[cluster_idx][voxel_index]++;
  //   }
  // }

  // // Report max point count per voxel per cluster
  // for (const auto & [cluster_idx, voxel_map] : voxel_point_count_per_cluster) {
  //   int max_points_in_voxel = 0;
  //   for (const auto & [voxel_index, count] : voxel_map) {
  //     max_points_in_voxel = std::max(max_points_in_voxel, count);
  //   }
  //   std::cout << "[cluster " << cluster_idx << "] Max points in any voxel: "
  //             << max_points_in_voxel << std::endl;
  // }

  // 7) Output assembly
  // build output and check cluster size
  {
    for (size_t i = 0; i < temporary_clusters.size(); ++i) {
      auto & i_cluster_data_size = clusters_data_size.at(i);
      if (!(min_cluster_size_ <= static_cast<int>(i_cluster_data_size / point_step))) {
        continue;
      }
      const auto & cluster = temporary_clusters.at(i);
      tier4_perception_msgs::msg::DetectedObjectWithFeature feature_object;
      feature_object.feature.cluster = cluster;
      feature_object.feature.cluster.data.resize(i_cluster_data_size);
      feature_object.feature.cluster.header = pointcloud_msg->header;
      feature_object.feature.cluster.is_bigendian = pointcloud_msg->is_bigendian;
      feature_object.feature.cluster.is_dense = pointcloud_msg->is_dense;
      feature_object.feature.cluster.point_step = point_step;
      feature_object.feature.cluster.row_step = i_cluster_data_size / pointcloud_msg->height;
      feature_object.feature.cluster.width =
        i_cluster_data_size / point_step / pointcloud_msg->height;

      feature_object.object.kinematics.pose_with_covariance.pose.position =
        getCentroid(feature_object.feature.cluster);
      autoware_perception_msgs::msg::ObjectClassification classification;
      classification.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      classification.probability = 1.0f;
      feature_object.object.classification.emplace_back(classification);

      objects.feature_objects.push_back(feature_object);
    }
    objects.header = pointcloud_msg->header;
  }
  auto t7 = Clock::now();
  durations[6] = std::chrono::duration_cast<us>(t7 - t6).count();

  // Total
  uint64_t total = 0;
  for (auto d : durations) total += d;
  std::cout << "[perf] Total cluster() time: " << total << " us" << std::endl;

  // Update accumulators
  for (size_t i = 0; i < STAGES; ++i) {
    sum_times[i] += durations[i];
    min_times[i] = std::min(min_times[i], durations[i]);
    max_times[i] = std::max(max_times[i], durations[i]);
  }
  ++perf_calls;

  // Periodic summary every 100 calls
  if (perf_calls % 100 == 0) {
    static const char* names[STAGES] = {
      "Decode input", "Voxel filter", "Build centroids",
      "Clustering", "Buffer prep", "Data copy", "Output build"
    };
    std::cout << "[perf-summary after " << perf_calls << " calls]" << std::endl;
    for (size_t i = 0; i < STAGES; ++i) {
      uint64_t avg = sum_times[i] / perf_calls;
      std::cout << "  " << names[i]
                << ": avg=" << avg << " us"
                << ", min=" << min_times[i] << " us"
                << ", max=" << max_times[i] << " us"
                << std::endl;
    }
  }


  return true;
}

}  // namespace autoware::euclidean_cluster
