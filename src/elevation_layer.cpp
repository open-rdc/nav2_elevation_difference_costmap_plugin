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

#include "nav2_elevation_difference_costmap_plugin/elevation_layer.hpp"

#include <algorithm>
#include <chrono>
#include <mutex>
#include <stdexcept>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace nav2_elevation_difference_costmap_plugin
{

namespace
{

struct TransformMatrix
{
  explicit TransformMatrix(
    const geometry_msgs::msg::Transform & transform)
  : translation(transform.translation)
  {
    const auto & q = transform.rotation;

    r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    r01 = 2.0 * (q.x * q.y - q.w * q.z);
    r02 = 2.0 * (q.x * q.z + q.w * q.y);

    r10 = 2.0 * (q.x * q.y + q.w * q.z);
    r11 = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
    r12 = 2.0 * (q.y * q.z - q.w * q.x);

    r20 = 2.0 * (q.x * q.z - q.w * q.y);
    r21 = 2.0 * (q.y * q.z + q.w * q.x);
    r22 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  }

  geometry_msgs::msg::Vector3 translation;

  double r00;
  double r01;
  double r02;

  double r10;
  double r11;
  double r12;

  double r20;
  double r21;
  double r22;
};

bool isInsideFootprint(
  double x,
  double y,
  const std::vector<geometry_msgs::msg::Point> & footprint)
{
  if (footprint.size() < 3 ||
      !std::isfinite(x) || !std::isfinite(y))
  {
    return false;
  }

  constexpr double epsilon = 1e-6;  // 辺上判定の許容距離[m]
  bool inside = false;

  for (size_t i = 0, j = footprint.size() - 1;
       i < footprint.size();
       j = i++)
  {
    const auto & a = footprint[j];
    const auto & b = footprint[i];

    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;

    // 点と辺の最短距離を求める
    double t = 0.0;
    if (length_squared > 0.0) {
      t = ((x - a.x) * dx + (y - a.y) * dy) / length_squared;
      t = std::max(0.0, std::min(1.0, t));
    }

    const double distance_x = x - (a.x + t * dx);
    const double distance_y = y - (a.y + t * dy);

    // 辺上の点もfootprint内として扱う
    if (distance_x * distance_x + distance_y * distance_y <=
        epsilon * epsilon)
    {
      return true;
    }

    // 点から右方向へ伸ばした半直線と辺の交差を判定
    if ((a.y > y) != (b.y > y)) {
      const double intersection_x =
        a.x + (y - a.y) * dx / dy;

      if (x < intersection_x) {
        inside = !inside;
      }
    }
  }

  return inside;
}

}  // namespace

void ElevationLayer::onInitialize()
{
  auto node = node_.lock();

  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));

  declareParameter(
    "topic",
    rclcpp::ParameterValue("/livox/lidar"));

  declareParameter(
    "footprint_filter.enabled",
    rclcpp::ParameterValue(true));

  node->get_parameter(
    name_ + ".enabled",
    enabled_);

  std::string topic;

  node->get_parameter(
    name_ + ".topic",
    topic);

  node->get_parameter(
    name_ + ".footprint_filter.enabled",
    footprint_filter_enabled_);

  // costmap本体に設定されたロボット基準フレームを取得
  node->get_parameter(
    "robot_base_frame",
    robot_base_frame_);

  tf_buffer_ =
    std::make_shared<tf2_ros::Buffer>(
      node->get_clock());

  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_);

  points_sub_ =
    node->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic,
      rclcpp::SensorDataQoS(),
      std::bind(
        &ElevationLayer::pointCloudCallback,
        this,
        std::placeholders::_1));

  matchSize();

  current_ = true;
}

void ElevationLayer::matchSize()
{
  auto * master =
    layered_costmap_->getCostmap();

  const size_t cell_count =
    static_cast<size_t>(
      master->getSizeInCellsX()) *
    master->getSizeInCellsY();

  cells_.assign(
    cell_count,
    CellInfo{});

  touched_cells_.clear();

  const size_t reserve_size =
    std::min(
      cell_count,
      static_cast<size_t>(20000));

  touched_cells_.reserve(reserve_size);
}

void ElevationLayer::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(
    cloud_mutex_);

  latest_cloud_ = msg;
}

void ElevationLayer::updateBounds(
  double /*robot_x*/,
  double /*robot_y*/,
  double /*robot_yaw*/,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  if (!enabled_) {
    return;
  }

  const auto * master = layered_costmap_->getCostmap();

  if (master->getSizeInCellsX() == 0 || master->getSizeInCellsY() == 0) {
    return;
  }

  // updateCosts writes directly into the master grid. Request its full extent
  // so that old costs are reset and all written cells are updated/published.
  // Read the geometry each cycle to follow resizing and rolling origins.
  const double origin_x = master->getOriginX();
  const double origin_y = master->getOriginY();
  const double end_x =
    origin_x + master->getSizeInCellsX() * master->getResolution();
  const double end_y =
    origin_y + master->getSizeInCellsY() * master->getResolution();

  // Like CostmapLayer::touch(), only expand bounds requested by other layers.
  *min_x = std::min(*min_x, origin_x);
  *min_y = std::min(*min_y, origin_y);
  *max_x = std::max(*max_x, end_x);
  *max_y = std::max(*max_y, end_y);
}

void ElevationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/,
  int /*min_j*/,
  int /*max_i*/,
  int /*max_j*/)
{
  if (!enabled_) {
    return;
  }

  const auto start_time =
    std::chrono::steady_clock::now();

  sensor_msgs::msg::PointCloud2::SharedPtr cloud;

  {
    std::lock_guard<std::mutex> lock(
      cloud_mutex_);

    cloud = latest_cloud_;
  }

  if (!cloud) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf;

  // 追加：点群座標系 → ロボット基準座標系のTF
  geometry_msgs::msg::TransformStamped cloud_to_base;

  try {
    tf = tf_buffer_->lookupTransform(
      layered_costmap_->getGlobalFrameID(),
      cloud->header.frame_id,
      tf2::TimePointZero);

    // 追加：footprint内外の判定に使うTF
    if (footprint_filter_enabled_) {
      cloud_to_base = tf_buffer_->lookupTransform(
        robot_base_frame_,
        cloud->header.frame_id,
        tf2::TimePointZero);
      }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_,
      "Failed to transform point cloud: %s",
      ex.what());

    return;
  }

  const size_t cell_count =
    static_cast<size_t>(
      master_grid.getSizeInCellsX()) *
    master_grid.getSizeInCellsY();

  if (cells_.size() != cell_count) {
    cells_.assign(
      cell_count,
      CellInfo{});

    touched_cells_.clear();

    const size_t point_count =
      static_cast<size_t>(
        cloud->width) *
      cloud->height;

    touched_cells_.reserve(
      std::min(
        cell_count,
        point_count));
  }

  const TransformMatrix transform(
    tf.transform);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(
    *cloud,
    "x");

  sensor_msgs::PointCloud2ConstIterator<float> iter_y(
    *cloud,
    "y");

  sensor_msgs::PointCloud2ConstIterator<float> iter_z(
    *cloud,
    "z");

  const auto footprint = layered_costmap_->getFootprint();

  for (;iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (footprint_filter_enabled_ &&
        isInsideFootprint(base_x, base_y, footprint))
    {
      continue;
    }

    const double x =
      transform.r00 * source_x +
      transform.r01 * source_y +
      transform.r02 * source_z +
      transform.translation.x;

    const double y =
      transform.r10 * source_x +
      transform.r11 * source_y +
      transform.r12 * source_z +
      transform.translation.y;

    const float z =
      static_cast<float>(
        transform.r20 * source_x +
        transform.r21 * source_y +
        transform.r22 * source_z +
        transform.translation.z);

    unsigned int mx;
    unsigned int my;

    if (!master_grid.worldToMap(
        x,
        y,
        mx,
        my))
    {
      continue;
    }

    const unsigned int index =
      master_grid.getIndex(
        mx,
        my);

    auto & cell =
      cells_[index];

    if (!cell.initialized) {
      cell.initialized = true;
      cell.z_min = z;
      cell.z_max = z;
      cell.point_count = 1;

      touched_cells_.push_back(index);
    } else {
      cell.z_min =
        std::min(
          cell.z_min,
          z);

      cell.z_max =
        std::max(
          cell.z_max,
          z);

      ++cell.point_count;
    }
  }

  unsigned char * costmap =
    master_grid.getCharMap();

  for (const unsigned int index : touched_cells_) {
    auto & cell =
      cells_[index];

    const float difference =
      cell.z_max - cell.z_min;

    if (difference < kMinElevationDifference) {
      costmap[index] = 0;
    } else if (
      difference >= kMaxElevationDifference)
    {
      costmap[index] = kMaxCost;
    } else {
      costmap[index] =
        static_cast<unsigned char>(
          (difference -
           kMinElevationDifference) *
          kMaxCost /
          (kMaxElevationDifference -
           kMinElevationDifference));
    }

    cell.initialized = false;
    cell.point_count = 0;
  }

  touched_cells_.clear();

  const auto elapsed =
    std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() -
      start_time);

  auto node = node_.lock();

  if (node) {
    // RCLCPP_INFO_THROTTLE(
    //   logger_,
    //   *node->get_clock(),
    //   1000,
    //   "updateCosts took %.3f ms",
    //   elapsed.count());

    RCLCPP_INFO(
      logger_,
      "updateCosts took %.3f ms",
      elapsed.count());
  }
}

void ElevationLayer::reset()
{
  std::lock_guard<std::mutex> lock(
    cloud_mutex_);

  latest_cloud_.reset();

  touched_cells_.clear();

  for (auto & cell : cells_) {
    cell.initialized = false;
    cell.point_count = 0;
  }

  current_ = true;
}

bool ElevationLayer::isClearable()
{
  return false;
}

}  // namespace nav2_elevation_difference_costmap_plugin

PLUGINLIB_EXPORT_CLASS(
  nav2_elevation_difference_costmap_plugin::ElevationLayer,
  nav2_costmap_2d::Layer)
