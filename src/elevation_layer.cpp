#include "nav2_elevation_difference_costmap_plugin/elevation_layer.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <chrono>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace
{

constexpr float kMaxElevationDifference = 0.03; //0.01f; //0.30f;
constexpr unsigned char kMaxCost = 254;

struct TransformMatrix
{
  explicit TransformMatrix(const geometry_msgs::msg::Transform & transform)
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

  double x(float x, float y, float z) const
  {
    return r00 * x + r01 * y + r02 * z + translation.x;
  }

  double y(float x, float y, float z) const
  {
    return r10 * x + r11 * y + r12 * z + translation.y;
  }

  float z(float x, float y, float z) const
  {
    return static_cast<float>(r20 * x + r21 * y + r22 * z + translation.z);
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

namespace nav2_elevation_difference_costmap_plugin
{

void ElevationLayer::onInitialize()
{
  auto node = node_.lock();

  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  points_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/surestar_points", rclcpp::SensorDataQoS(),
    std::bind(&ElevationLayer::pointCloudCallback, this, std::placeholders::_1));

  cell_data_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/elevation_cell_data",
    rclcpp::QoS(1).reliable());

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

  const auto start_time = std::chrono::steady_clock::now();

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      layered_costmap_->getGlobalFrameID(),
      cloud->header.frame_id,
      tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(logger_, "%s", ex.what());
    return;
  }

  const size_t cell_count =
    static_cast<size_t>(master_grid.getSizeInCellsX()) * master_grid.getSizeInCellsY();
  if (cells_.size() != cell_count) {
    cells_.assign(cell_count, CellInfo{});
    touched_cells_.clear();
    const size_t point_count = static_cast<size_t>(cloud->width) * cloud->height;
    touched_cells_.reserve(std::min(cell_count, point_count));
  }

  // Transform XYZ directly instead of allocating a transformed PointCloud2.
  const TransformMatrix transform(tf.transform);

  sensor_msgs::PointCloud2ConstIterator<float>
  iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float>
  iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float>
  iter_z(*cloud, "z");

  for (;
    iter_x != iter_x.end();
    ++iter_x, ++iter_y, ++iter_z)
  {
    const float source_x = *iter_x;
    const float source_y = *iter_y;
    const float source_z = *iter_z;
    const double x = transform.x(source_x, source_y, source_z);
    const double y = transform.y(source_x, source_y, source_z);
    const float z = transform.z(source_x, source_y, source_z);

    unsigned int mx;
    unsigned int my;

    if (!master_grid.worldToMap(x, y, mx, my)) {
      continue;
    }

    const unsigned int index = master_grid.getIndex(mx, my);
    auto & cell = cells_[index];

    if (!cell.initialized) {
      cell.initialized = true;
      cell.z_min = z;
      cell.z_max = z;
      cell.point_count = 1;

      touched_cells_.push_back(index);
    } else {
      cell.z_min = std::min(cell.z_min, z);
      cell.z_max = std::max(cell.z_max, z);
      ++cell.point_count;
    }
  }

  auto * costmap = master_grid.getCharMap();

  // ==========================================
  // 追加1：1フレーム分の送信メッセージを作る
  // ==========================================
  std_msgs::msg::Float32MultiArray output_msg;
  // 1セルにつき6個の値を格納する
  output_msg.data.reserve(touched_cells_.size() * 6);
  // indexからmx, myを求めるために必要
  const unsigned int size_x = master_grid.getSizeInCellsX();

  for (const unsigned int index : touched_cells_) {
    auto & cell = cells_[index];
    const float difference = cell.z_max - cell.z_min;

    // ========================================
    // 追加2：Costmap上のセル番号を求める
    // ========================================
    const unsigned int mx = index % size_x;
    const unsigned int my = index / size_x;

    // ========================================
    // 追加3：セル番号をワールド座標へ変換
    // ========================================
    double wx = 0.0;
    double wy = 0.0;

    master_grid.mapToWorld(
      mx,
      my,
      wx,
      wy);

    // ========================================
    // 追加4：Pythonへ送る値を格納
    // ========================================
    output_msg.data.push_back(
      static_cast<float>(wx));

    output_msg.data.push_back(
      static_cast<float>(wy));

    output_msg.data.push_back(cell.z_min);
    output_msg.data.push_back(cell.z_max);
    output_msg.data.push_back(difference);

    output_msg.data.push_back(
      static_cast<float>(cell.point_count));

    costmap[index] = difference >= kMaxElevationDifference ?
      kMaxCost :
      static_cast<unsigned char>(difference * kMaxCost / kMaxElevationDifference);

    cell.initialized = false;
    cell.point_count = 0;
  }

  // ==========================================
  // 追加5：1フレーム分をトピックへ送信
  // ==========================================
  if (!output_msg.data.empty() && cell_data_pub_) {
    cell_data_pub_->publish(output_msg);
  }

  touched_cells_.clear();

  const auto elapsed = std::chrono::steady_clock::now() - start_time;
  RCLCPP_INFO(
    logger_, "updateCosts took %f ms",
    std::chrono::duration<double, std::milli>(elapsed).count());
}

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
