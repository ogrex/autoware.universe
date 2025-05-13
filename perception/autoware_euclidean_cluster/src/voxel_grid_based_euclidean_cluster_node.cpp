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

#include "voxel_grid_based_euclidean_cluster_node.hpp"

#include "autoware/euclidean_cluster/utils.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h>

#include <cstddef>
#include <memory>
#include <vector>
#include <filesystem> 
namespace autoware::euclidean_cluster
{
VoxelGridBasedEuclideanClusterNode::VoxelGridBasedEuclideanClusterNode(
  const rclcpp::NodeOptions & options)
: Node("voxel_grid_based_euclidean_cluster_node", options),
tf_buffer_(this->get_clock()),
tf_listener_(tf_buffer_)
{
  const bool use_height = this->declare_parameter("use_height", false);
  const int min_cluster_size = this->declare_parameter("min_cluster_size", 1);
  const int max_cluster_size = this->declare_parameter("max_cluster_size", 500);
  const float tolerance = this->declare_parameter("tolerance", 1.0);
  const float voxel_leaf_size = this->declare_parameter("voxel_leaf_size", 0.5);
  const int min_points_number_per_voxel = this->declare_parameter("min_points_number_per_voxel", 3);
  const int min_voxel_cluster_size_for_filtering =
    this->declare_parameter("min_voxel_cluster_size_for_filtering", 150);
  const int max_points_per_voxel_in_large_cluster =
    this->declare_parameter("max_points_per_voxel_in_large_cluster", 10);
  cluster_ = std::make_shared<VoxelGridBasedEuclideanCluster>(
    use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size,
    min_points_number_per_voxel, min_voxel_cluster_size_for_filtering,
    max_points_per_voxel_in_large_cluster);

  using std::placeholders::_1;
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&VoxelGridBasedEuclideanClusterNode::onPointCloud, this, _1));

  cluster_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
    "output", rclcpp::QoS{1});
  debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/clusters", 1);
  debug_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "debug/markers", rclcpp::QoS{1});
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  debug_publisher_ =
    std::make_unique<autoware_utils::DebugPublisher>(this, "voxel_grid_based_euclidean_cluster");
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");
}
/**
 * @brief Compute extended characteristics of a point cloud cluster for vapor filtering.
 * 
 * @param cluster Input point cloud cluster (e.g., from clustering stage).
 * @param point_count Number of points in the cluster.
 * @param intensity_average Mean intensity of all points.
 * @param intensity_stddev Standard deviation of intensity (useful for vapor reflectivity noise).
 * @param cluster_size Bounding box volume (Δx * Δy * Δz).
 * @param density Point density (points / volume).
 * @param elongation Shape descriptor: λ₂ / λ₁ (how stretched the cluster is).
 * @param flatness Shape descriptor: λ₃ / λ₁ (how flat the cluster is).
 * @param z_range Vertical height span (max_z - min_z).
 * @param centroid_z Z-value of the cluster centroid (helps detect floating vapor).
 * @param bounding_box_diag Diagonal length of the 3D bounding box.
 * @param min_distance Closest distance from sensor (vapor often appears nearby).
 * @param eigenvalue_ratio (λ₁ - λ₂) / λ₁: additional shape consistency metric.
 */
 void getClusterCharacteristicsExtended(
  const sensor_msgs::msg::PointCloud2 & cluster,
  size_t & point_count,
  double & intensity_average,
  double & intensity_stddev,
  double & cluster_size,
  double & density,
  double & elongation,
  double & flatness,
  double & z_range,
  double & centroid_z,
  double & bounding_box_diag,
  double & min_distance,
  double & eigenvalue_ratio)
{
  using IteratorF = sensor_msgs::PointCloud2ConstIterator<float>;
  using IteratorU8 = sensor_msgs::PointCloud2ConstIterator<uint8_t>;

  IteratorF iter_x(cluster, "x");
  IteratorF iter_y(cluster, "y");
  IteratorF iter_z(cluster, "z");
  IteratorU8 iter_intensity(cluster, "intensity");

  point_count = 0;
  double intensity_total = 0.0;
  double intensity_sq_total = 0.0;

  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  double min_z = std::numeric_limits<double>::max();
  double max_z = std::numeric_limits<double>::lowest();

  double min_dist = std::numeric_limits<double>::max();
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

  // First pass: gather stats
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
    double x = static_cast<double>(*iter_x);
    double y = static_cast<double>(*iter_y);
    double z = static_cast<double>(*iter_z);
    double d = std::sqrt(x * x + y * y + z * z);

    double intensity = static_cast<double>(*iter_intensity);
    intensity_total += intensity;
    intensity_sq_total += intensity * intensity;

    min_x = std::min(min_x, x); max_x = std::max(max_x, x);
    min_y = std::min(min_y, y); max_y = std::max(max_y, y);
    min_z = std::min(min_z, z); max_z = std::max(max_z, z);

    min_dist = std::min(min_dist, d);

    mean += Eigen::Vector3d(x, y, z);
    ++point_count;
  }

  if (point_count == 0) {
    intensity_average = 0.0;
    intensity_stddev = 0.0;
    cluster_size = 0.0;
    density = 0.0;
    elongation = 0.0;
    flatness = 0.0;
    z_range = 0.0;
    centroid_z = 0.0;
    bounding_box_diag = 0.0;
    min_distance = 0.0;
    eigenvalue_ratio = 0.0;
    return;
  }

  intensity_average = intensity_total / point_count;
  intensity_stddev = std::sqrt((intensity_sq_total / point_count) - (intensity_average * intensity_average));

  mean /= point_count;

  // Second pass: covariance matrix
  iter_x = IteratorF(cluster, "x");
  iter_y = IteratorF(cluster, "y");
  iter_z = IteratorF(cluster, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    Eigen::Vector3d p(*iter_x, *iter_y, *iter_z);
    Eigen::Vector3d diff = p - mean;
    covariance += diff * diff.transpose();
  }
  covariance /= point_count;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  Eigen::Vector3d eig = solver.eigenvalues();

  std::sort(eig.data(), eig.data() + 3, std::greater<double>());
  elongation = (eig[0] > 0.0) ? eig[1] / eig[0] : 0.0;
  flatness = (eig[0] > 0.0) ? eig[2] / eig[0] : 0.0;
  eigenvalue_ratio = (eig[0] > 0.0) ? (eig[0] - eig[1]) / eig[0] : 0.0;

  cluster_size = (max_x - min_x) * (max_y - min_y) * (max_z - min_z);
  density = (cluster_size > 0.0) ? point_count / cluster_size : 0.0;
  z_range = max_z - min_z;
  centroid_z = mean.z();
  bounding_box_diag = std::sqrt(
    (max_x - min_x) * (max_x - min_x) +
    (max_y - min_y) * (max_y - min_y) +
    (max_z - min_z) * (max_z - min_z));
  min_distance = min_dist;
}


void save_cluster_to_csv(
  const sensor_msgs::msg::PointCloud2 & cluster, const geometry_msgs::msg::Point & centroid, int id,
  const rclcpp::Time & msg_stamp,
  const geometry_msgs::msg::TransformStamped& transform,
  const std::string& file_path)
{
  std::ofstream file(file_path, std::ios::app);  // open in append mode

  if (!file.is_open()) {
      RCLCPP_ERROR(rclcpp::get_logger("ClusterSaver"), "Failed to open file: %s", file_path.c_str());
      return;
  }
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cluster, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cluster, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cluster, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(cluster, "intensity");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {

    geometry_msgs::msg::PointStamped point_in;
    point_in.header.frame_id = cluster.header.frame_id;
    point_in.header.stamp = cluster.header.stamp;
    point_in.point.x = *iter_x ;
    point_in.point.y = *iter_y ;
    point_in.point.z = *iter_z ;

    geometry_msgs::msg::PointStamped point_out;

    // Transform the point using tf2
    tf2::doTransform(point_in, point_out, transform);
    

    file << std::fixed << std::setprecision(6)
    << msg_stamp.seconds() << ","  // timestamp (float seconds)
    << id << ","
    << centroid.x << ","
    << centroid.y << ","
    << centroid.z << ","  
    << point_out.point.x << ","
    << point_out.point.y<< ","
    << point_out.point.z << ","
    << int(*iter_intensity)
    << "\n";
  }
  file.close();
}


void VoxelGridBasedEuclideanClusterNode::getDebugMarkerArray(
  const std_msgs::msg::Header & header,
  const std::vector<tier4_perception_msgs::msg::DetectedObjectWithFeature> & feature_objects,
  visualization_msgs::msg::MarkerArray & debug_marker_array)
{
  geometry_msgs::msg::TransformStamped transform_stamp_to_map;
  try {
     transform_stamp_to_map = tf_buffer_.lookupTransform(
      "map", header.frame_id, tf2_ros::fromMsg(header.stamp));
 
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "Could not transform %s to map: %s",
                header.frame_id.c_str(), ex.what());
    return;
  }
  // geometry_msgs::msg::TransformStamped transform_stamp_to_map = tf_buffer_.lookupTransform(
  //   "map", header.frame_id, tf2_ros::fromMsg(header.stamp));

  for (size_t i = 0; i < feature_objects.size(); ++i) {
    const auto & feature_object = feature_objects.at(i);
    const auto & cluster = feature_object.feature.cluster;

    // Create a text marker to display the cluster point number
    visualization_msgs::msg::Marker text_marker;
    text_marker.header = header;
    text_marker.ns = "cluster_point_numbers";
    text_marker.id = static_cast<int>(i);
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;


    size_t point_count = 0;
    double intensity_average = 0.0;
    double intensity_stddev = 0.0;
    double cluster_size = 0.0;
    double density = 0.0;
    double elongation = 0.0;
    double flatness = 0.0;
    double z_range = 0.0;
    double centroid_z = 0.0;
    double bounding_box_diag = 0.0;
    double min_distance = 0.0;
    double eigenvalue_ratio = 0.0;
    double existence_probability = feature_object.object.existence_probability;
    // Get the extended characteristics of the cluster
    getClusterCharacteristicsExtended(
      cluster, point_count, intensity_average, intensity_stddev, cluster_size, density, elongation,
      flatness, z_range, centroid_z, bounding_box_diag, min_distance, eigenvalue_ratio);


    

    // Set the position of the text marker to the centroid of the cluster
    geometry_msgs::msg::Point centroid;
    geometry_msgs::msg::Point centroid_in_absolute_coordinate;
    centroid.x = 0.0;
    centroid.y = 0.0;
    centroid.z = 0.0;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cluster, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cluster, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cluster, "z");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(cluster, "intensity");

    double intensity_total=0;
    double distance = 0;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
      centroid.x += *iter_x;
      centroid.y += *iter_y;
      centroid.z += *iter_z;
      intensity_total += *iter_intensity;
      //++point_count;
    }

    if (point_count > 0) {
      centroid.x /= point_count;
      centroid.y /= point_count;
      centroid.z /= point_count;
      intensity_total /= point_count;
      text_marker.color.r = static_cast<float>(intensity_total) / 255.0f;
      text_marker.color.g = 0.0f;
      text_marker.color.b = 0.0f;
      text_marker.color.a = 1.0f;
    }
    geometry_msgs::msg::PointStamped centroid_in, centroid_out;
    centroid_in.header.frame_id = header.frame_id;
    centroid_in.header.stamp = header.stamp;
    centroid_in.point = centroid;
    
    tf2::doTransform(centroid_in, centroid_out, transform_stamp_to_map);

    distance = std::sqrt(centroid.x * centroid.x + centroid.y * centroid.y + centroid.z * centroid.z);
    text_marker.pose.position = centroid;
    text_marker.pose.position.z += 1.0;  // Offset the text above the cluster
    text_marker.pose.orientation.w = 1.0;

    // Set the text to display the number of points in the cluster
    text_marker.text = "Cluster " + std::to_string(i) + ": " + std::to_string(point_count) +
                       " points\n" + "Distance: " + std::to_string(distance) + "\n" +
                       "Intensity: " + std::to_string(intensity_average) + "\n" +
                       "Centroid:" + std::to_string(centroid.x) + ", " +
                       std::to_string(centroid.y) + ", " +
                       "Centroid in absolute coordinate: " +
                       std::to_string(centroid_out.point.x) + ", " +
                       std::to_string(centroid_out.point.y) + ", " +
                       std::to_string(centroid_out.point.z) + "\n" +
      "Density: " + std::to_string(density) + "\n" + "Elongation: " + std::to_string(elongation) +
      "\n" + "Flatness: " + std::to_string(flatness) + "\n" +
      "Z Range: " + std::to_string(z_range) + "\n" + "Centroid Z: " + std::to_string(centroid_z) +
      "\n" + "Bounding Box Diag: " + std::to_string(bounding_box_diag) + "\n" +
      "Min Distance: " + std::to_string(min_distance) + "\n" +
      "Eigenvalue Ratio: " + std::to_string(eigenvalue_ratio)+ "\n" +
      "Existence Probability: " + std::to_string(existence_probability) ;
    // Set marker scale and color
    text_marker.scale.z = 0.2;  // Text height
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;

    debug_marker_array.markers.push_back(text_marker);

    // Check if centroid is inside bounding box
    if (centroid_out.point.x < 65670|| centroid_out.point.x > 65678 ||
      centroid_out.point.y < 678 || centroid_out.point.y > 682 ||
      centroid_out.point.z < 712 || centroid_out.point.z > 716) {
      continue;
    }

    std::cout << "centroid_out passed" << std::filesystem::current_path() / "cluster.csv" << std::endl; 

    save_cluster_to_csv(
      cluster, centroid_out.point, static_cast<int>(i), header.stamp,transform_stamp_to_map, "cluster.csv");


  }
}

void VoxelGridBasedEuclideanClusterNode::onPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  stop_watch_ptr_->toc("processing_time", true);

  // convert ros to pcl
  if (input_msg->data.empty()) {
    // NOTE: prevent pcl log spam
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Empty sensor points!");
  }
  // cluster and build output msg
  tier4_perception_msgs::msg::DetectedObjectsWithFeature output;

  cluster_->cluster(input_msg, output);
  cluster_pub_->publish(output);

  // build debug msg
  if (debug_pub_->get_subscription_count() >= 1) {
    sensor_msgs::msg::PointCloud2 debug;
    convertObjectMsg2SensorMsg(output, debug);
    debug_pub_->publish(debug);


    visualization_msgs::msg::MarkerArray debug_marker_array;
    getDebugMarkerArray(input_msg->header, output.feature_objects, debug_marker_array);
    debug_marker_pub_->publish(debug_marker_array);
  }
  if (debug_publisher_) {
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds((this->get_clock()->now() - output.header.stamp).nanoseconds()))
        .count();
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }
}

}  // namespace autoware::euclidean_cluster

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::euclidean_cluster::VoxelGridBasedEuclideanClusterNode)
