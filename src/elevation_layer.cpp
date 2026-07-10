#include "nav2_elevation_difference_costmap_plugin/elevation_layer.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <chrono>
#include <cmath>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace nav2_elevation_difference_costmap_plugin
{

ElevationLayer::ElevationLayer()
{
}

void ElevationLayer::onInitialize()
{
  auto node = node_.lock();

  points_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/surestar_points", rclcpp::SensorDataQoS(),
    std::bind(&ElevationLayer::pointCloudCallback, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ElevationLayer::updateBounds(
  double,
  double,
  double,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  *min_x = -100;
  *min_y = -100;
  *max_x = 100;
  *max_y = 100;
}


// circle wave //
void ElevationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/,
  int /*min_j*/,
  int /*max_i*/,
  int /*max_j*/)
{
  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud = latest_cloud_;
  }

  if (!cloud) {
    return;
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      layered_costmap_->getGlobalFrameID(),
      cloud->header.frame_id,
      tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {

    RCLCPP_WARN(
      logger_,
      "%s",
      ex.what());

    return;
  }

  const unsigned int size_x =
    master_grid.getSizeInCellsX();

  const unsigned int size_y =
    master_grid.getSizeInCellsY();

  const size_t cell_count = static_cast<size_t>(size_x) * size_y;
  if (cells_.size() != cell_count) {
    cells_.assign(cell_count, CellInfo{});
    touched_cells_.clear();
    const size_t point_count = static_cast<size_t>(cloud->width) * cloud->height;
    touched_cells_.reserve(std::min(cell_count, point_count));
  }

  // Build the rotation matrix once. Transforming XYZ directly avoids allocating
  // and copying an entire transformed PointCloud2 on every costmap update.
  const auto & q = tf.transform.rotation;
  const double xx = q.x * q.x;
  const double yy = q.y * q.y;
  const double zz = q.z * q.z;
  const double xy = q.x * q.y;
  const double xz = q.x * q.z;
  const double yz = q.y * q.z;
  const double wx = q.w * q.x;
  const double wy = q.w * q.y;
  const double wz = q.w * q.z;
  const double r00 = 1.0 - 2.0 * (yy + zz);
  const double r01 = 2.0 * (xy - wz);
  const double r02 = 2.0 * (xz + wy);
  const double r10 = 2.0 * (xy + wz);
  const double r11 = 1.0 - 2.0 * (xx + zz);
  const double r12 = 2.0 * (yz - wx);
  const double r20 = 2.0 * (xz - wy);
  const double r21 = 2.0 * (yz + wx);
  const double r22 = 1.0 - 2.0 * (xx + yy);
  const auto & translation = tf.transform.translation;

  sensor_msgs::PointCloud2ConstIterator<float>
  iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float>
  iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float>
  iter_z(*cloud, "z");

  //
  // 点群を走査して zmin / zmax を集計
  //
  for (;
    iter_x != iter_x.end();
    ++iter_x, ++iter_y, ++iter_z)
  {
    const float source_x = *iter_x;
    const float source_y = *iter_y;
    const float source_z = *iter_z;
    const double x = r00 * source_x + r01 * source_y + r02 * source_z + translation.x;
    const double y = r10 * source_x + r11 * source_y + r12 * source_z + translation.y;
    const float z = static_cast<float>(
      r20 * source_x + r21 * source_y + r22 * source_z + translation.z);

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
      master_grid.getIndex(mx, my);

    auto & cell = cells_[index];

    if (!cell.initialized) {

      cell.initialized = true;
      cell.z_min = z;
      cell.z_max = z;
      touched_cells_.push_back(index);

    } else {

      cell.z_min =
        std::min(cell.z_min, z);

      cell.z_max =
        std::max(cell.z_max, z);
    }
  }

  //
  // Δzからコスト生成
  //
  for (const unsigned int index : touched_cells_) {
    auto & cell = cells_[index];

    const float dz =
      cell.z_max - cell.z_min;

    unsigned char cost;

    if (dz >= 0.30f) {

      cost = 254;

    } else {

      cost = static_cast<unsigned char>(
        std::min(
          254.0f,
          dz / 0.30f * 254.0f));
    }

    master_grid.getCharMap()[index] = cost;
    cell.initialized = false;
  }
  touched_cells_.clear();

  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
  RCLCPP_INFO(
    logger_,
    "updateCosts took %f ms",
    elapsed.count());
}

// reset //
void ElevationLayer::reset()
{
}

bool ElevationLayer::isClearable()
{
  return false;
}

void ElevationLayer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  latest_cloud_ = msg;
}

PLUGINLIB_EXPORT_CLASS(
  nav2_elevation_difference_costmap_plugin::ElevationLayer,
  nav2_costmap_2d::Layer)

}
