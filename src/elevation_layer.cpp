#include "nav2_elevation_difference_costmap_plugin/elevation_layer.hpp"

#include "pluginlib/class_list_macros.hpp"

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
  if (!latest_cloud_) {
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud_odom;

  try {
    auto tf =
      tf_buffer_->lookupTransform(
      layered_costmap_->getGlobalFrameID(),
      latest_cloud_->header.frame_id,
      tf2::TimePointZero);

    tf2::doTransform(
      *latest_cloud_,
      cloud_odom,
      tf);

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

  struct CellInfo
  {
    bool initialized = false;
    float z_min = 0.0f;
    float z_max = 0.0f;
  };

  std::vector<CellInfo> cells(size_x * size_y);

  sensor_msgs::PointCloud2ConstIterator<float>
  iter_x(cloud_odom, "x");
  sensor_msgs::PointCloud2ConstIterator<float>
  iter_y(cloud_odom, "y");
  sensor_msgs::PointCloud2ConstIterator<float>
  iter_z(cloud_odom, "z");

  //
  // 点群を走査して zmin / zmax を集計
  //
  for (;
    iter_x != iter_x.end();
    ++iter_x, ++iter_y, ++iter_z)
  {
    const float x = *iter_x;
    const float y = *iter_y;
    const float z = *iter_z;

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

    auto & cell = cells[index];

    if (!cell.initialized) {

      cell.initialized = true;
      cell.z_min = z;
      cell.z_max = z;

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
  for (unsigned int my = 0;
    my < size_y;
    ++my)
  {
    for (unsigned int mx = 0;
      mx < size_x;
      ++mx)
    {
      const unsigned int index =
        master_grid.getIndex(mx, my);

      const auto & cell =
        cells[index];

      if (!cell.initialized) {
        continue;
      }

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

      master_grid.setCost(
        mx,
        my,
        cost);
    }
  }
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
  latest_cloud_ = msg;
}

PLUGINLIB_EXPORT_CLASS(
  nav2_elevation_difference_costmap_plugin::ElevationLayer,
  nav2_costmap_2d::Layer)

}
