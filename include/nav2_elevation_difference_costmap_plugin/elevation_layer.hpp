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

  struct BeamFilter
  {
    bool enabled = true;
    bool negative = true;

    double xmin = -0.70;
    double xmax = 0.70;

    double ymin = -0.45;
    double ymax = 1.00;

    double zmin = -2.0;
    double zmax = 2.0;
  };

  void pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void matchSize();

  BeamFilter beam_filter_;

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