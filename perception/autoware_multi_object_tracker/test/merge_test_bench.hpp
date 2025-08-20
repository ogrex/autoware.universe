// Copyright 2025 TIER IV, inc.
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
#ifndef MERGE_TEST_BENCH_HPP_
#define MERGE_TEST_BENCH_HPP_
#include "test_bench.hpp"

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
class MergeTestBench : public TrackingTestBench
{
public:
  explicit MergeTestBench(const TrackingScenarioConfig & params) : TrackingTestBench(params)
  {
    initializeObjects(params);
  }

  void initializeObjects(const TrackingScenarioConfig & params) override
  {
    std::cout << " Calling MergeTestBench::initializeObjects" << std::endl;
    std::vector<float> lane_speeds_kmh;
    lane_speeds_kmh = {0.0f,  2.0f,  4.0f,  6.0f,  8.0f,  10.0f,
                       15.0f, 25.0f, 30.0f, 35.0f, 40.0f, 45.0f};
    std::vector<float> lane_speeds;
    for (float kmh : lane_speeds_kmh) {
      lane_speeds.push_back(kmh * 1000.0f / 3600.0f);  // convert km/h to m/s
    }

    for (int lane = 0; lane < static_cast<int>(lane_speeds.size()); ++lane) {
      float y = lane * params.lane_width;
      std::string car_id = "car_lane_" + std::to_string(lane);

      // Place car at start
      addNewCar(car_id, 0.0f - lane_speeds[lane] * 5, y, lane_speeds[lane]);

      // Attach unknown near this car
      std::string unk_id = "unk_lane_" + std::to_string(lane);
      addNewUnknownNearCar(car_id, unk_id, y);
    }
  }

  // Keep unknowns stuck near cars
  autoware::multi_object_tracker::types::DynamicObjectList generateDetections(
    const rclcpp::Time & stamp) override
  {
    auto detections = TrackingTestBench::generateDetections(stamp);

    std::cout << "Generating detections with " << car_states_.size() << " cars and "
              << unknown_states_.size() << " unknowns" << std::endl;
    // Update unknowns to follow their car (with no speed of their own)
    for (auto & [unk_id, state] : unknown_states_) {
      std::string car_id = unk_id_to_car_[unk_id];
      if (car_states_.count(car_id)) {
        const auto & car_pose = car_states_[car_id].pose;
        // Stick unknown near car (slightly offset in x or y)
        state.pose.position.x = car_pose.position.x + nearby_offset_x_[unk_id];
        state.pose.position.y = car_pose.position.y + nearby_offset_y_[unk_id];
      }
    }
    return detections;
  }

private:
  // Helper to spawn a car
  void addNewCar(const std::string & id, float x, float y, float speed)
  {
    ObjectState state;
    state.pose.position.x = x;
    state.pose.position.y = y;
    state.twist.linear.x = speed;
    state.shape = {3.5f, 2.0f};  // length, width
    car_states_[id] = state;
  }

  // Helper to spawn unknown near a car
  void addNewUnknownNearCar(const std::string & car_id, const std::string & unk_id, float lane_y)
  {
    UnknownObjectState state;
    state.pose.position.x = 0.5f;  // initial offset
    state.pose.position.y = lane_y + 1.0f;
    state.pose.position.z = 0.5f;    // fixed height
    state.pose.orientation.w = 1.0;  // no rotation

    state.is_moving = false;  // keep unknowns static
    state.twist.linear.x = 0.0f;
    state.twist.linear.y = 0.0f;

    state.z_dimension = 0.5f;  // fixed height
    state.previous_footprint = state.current_footprint;
    state.base_size = 2.0f;                                                 // fixed base size
    state.shape_type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;  // simple shape

    unknown_states_[unk_id] = state;
    std::cout << "Added unknown " << unk_id << " near car " << car_id << " at position ("
              << state.pose.position.x << ", " << state.pose.position.y << ")" << std::endl;
    std::cout << "Current unknown_states_ size: " << unknown_states_.size() << std::endl;

    // Track relationship car<->unk
    unk_id_to_car_[unk_id] = car_id;
    nearby_offset_x_[unk_id] = -4.5f / 2 - state.base_size / 2.0f - 1.5f;
    nearby_offset_y_[unk_id] = 0.0f;
  }

  std::unordered_map<std::string, std::string> unk_id_to_car_;
  std::unordered_map<std::string, float> nearby_offset_x_;
  std::unordered_map<std::string, float> nearby_offset_y_;
};

#endif  // MERGE_TEST_BENCH_HPP_
