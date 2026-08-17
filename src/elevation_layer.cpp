#include "nav2_elevation_difference_costmap_plugin/elevation_layer.hpp"

#include <algorithm>
#include <chrono>
#include <mutex>
#include <stdexcept>

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
    rclcpp::ParameterValue("/surestar_points"));

  declareParameter(
    "beam_filter.enabled",
    rclcpp::ParameterValue(true));

  declareParameter(
    "beam_filter.negative",
    rclcpp::ParameterValue(true));

  declareParameter(
    "beam_filter.xmin",
    rclcpp::ParameterValue(-0.70));

  declareParameter(
    "beam_filter.xmax",
    rclcpp::ParameterValue(0.70));

  declareParameter(
    "beam_filter.ymin",
    rclcpp::ParameterValue(-0.45));

  declareParameter(
    "beam_filter.ymax",
    rclcpp::ParameterValue(1.00));

  declareParameter(
    "beam_filter.zmin",
    rclcpp::ParameterValue(-2.0));

  declareParameter(
    "beam_filter.zmax",
    rclcpp::ParameterValue(2.0));

  node->get_parameter(
    name_ + ".enabled",
    enabled_);

  std::string topic;

  node->get_parameter(
    name_ + ".topic",
    topic);

  node->get_parameter(
    name_ + ".beam_filter.enabled",
    beam_filter_.enabled);

  node->get_parameter(
    name_ + ".beam_filter.negative",
    beam_filter_.negative);

  node->get_parameter(
    name_ + ".beam_filter.xmin",
    beam_filter_.xmin);

  node->get_parameter(
    name_ + ".beam_filter.xmax",
    beam_filter_.xmax);

  node->get_parameter(
    name_ + ".beam_filter.ymin",
    beam_filter_.ymin);

  node->get_parameter(
    name_ + ".beam_filter.ymax",
    beam_filter_.ymax);

  node->get_parameter(
    name_ + ".beam_filter.zmin",
    beam_filter_.zmin);

  node->get_parameter(
    name_ + ".beam_filter.zmax",
    beam_filter_.zmax);

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

  *min_x = -100.0;
  *min_y = -100.0;
  *max_x = 100.0;
  *max_y = 100.0;
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

  try {
    tf = tf_buffer_->lookupTransform(
      layered_costmap_->getGlobalFrameID(),
      cloud->header.frame_id,
      tf2::TimePointZero);
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

  for (;
    iter_x != iter_x.end();
    ++iter_x,
    ++iter_y,
    ++iter_z)
  {
    const float source_x = *iter_x;
    const float source_y = *iter_y;
    const float source_z = *iter_z;

    if (beam_filter_.enabled) {
      const bool inside =
        source_x >= beam_filter_.xmin &&
        source_x <= beam_filter_.xmax &&
        source_y >= beam_filter_.ymin &&
        source_y <= beam_filter_.ymax &&
        source_z >= beam_filter_.zmin &&
        source_z <= beam_filter_.zmax;

      if (beam_filter_.negative) {
        if (inside) {
          continue;
        }
      } else {
        if (!inside) {
          continue;
        }
      }
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
    RCLCPP_INFO_THROTTLE(
      logger_,
      *node->get_clock(),
      1000,
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
