#ifndef NAV2_ELEVATION_DIFFERENCE_COSTMAP_PLUGIN__ELEVATION_LAYER_HPP_
#define NAV2_ELEVATION_DIFFERENCE_COSTMAP_PLUGIN__ELEVATION_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <mutex>
#include <vector>

#include <std_msgs/msg/float32_multi_array.hpp>

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

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  std::mutex cloud_mutex_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cell_data_pub_;

  // Keep only cells hit in the current update, avoiding a full-grid scan.
  std::vector<CellInfo> cells_;
  std::vector<unsigned int> touched_cells_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}

#endif  // NAV2_ELEVATION_DIFFERENCE_COSTMAP_PLUGIN__ELEVATION_LAYER_HPP_
