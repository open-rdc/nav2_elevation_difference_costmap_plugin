// BSD 3-Clause License
//
// Copyright (c) 2026 Shuma Suzuki
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



#ifndef NAV2_ELEVATION_DIFFERENCE_COSTMAP_PLUGIN__ELEVATION_LAYER_HPP_
#define NAV2_ELEVATION_DIFFERENCE_COSTMAP_PLUGIN__ELEVATION_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <vector>
#include <string>

namespace nav2_elevation_difference_costmap_plugin
{

class ElevationLayer : public nav2_costmap_2d::Layer
{
public:
  ElevationLayer() = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x,
    double robot_y,
    double robot_yaw,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j) override;

  void reset() override;

  bool isClearable() override;

private:
  struct CellInfo
  {
    bool initialized{false};
    float z_min{0.0f};
    float z_max{0.0f};
    unsigned int point_count{0};
  };

  bool footprint_filter_enabled_{true};

  void pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void matchSize();

  std::string robot_base_frame_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    points_sub_;

  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

  std::mutex cloud_mutex_;

  std::vector<CellInfo> cells_;
  std::vector<unsigned int> touched_cells_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool enabled_ = true;

  static constexpr float kMinElevationDifference = 0.005f;
  static constexpr float kMaxElevationDifference = 0.01f;
  static constexpr unsigned char kMaxCost = 254;
};

}  // namespace nav2_elevation_difference_costmap_plugin

#endif  // NAV2_ELEVATION_DIFFERENCE_COSTMAP_PLUGIN__ELEVATION_LAYER_HPP_