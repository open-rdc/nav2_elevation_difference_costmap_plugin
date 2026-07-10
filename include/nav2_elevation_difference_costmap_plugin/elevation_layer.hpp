#ifndef NAV2_ELEVATION_DIFFERENCE_COSTMAP_PLUGIN__ELEVATION_LAYER_HPP_
#define NAV2_ELEVATION_DIFFERENCE_COSTMAP_PLUGIN__ELEVATION_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <mutex>
#include <vector>

namespace nav2_elevation_difference_costmap_plugin
{

class ElevationLayer : public nav2_costmap_2d::Layer
{
public:
  ElevationLayer();

  virtual void onInitialize() override;
  virtual void updateBounds(
    double robot_x,
    double robot_y,
    double robot_yaw,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;

  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j) override;

  virtual void reset() override;
  virtual bool isClearable() override;

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  struct CellInfo
  {
    bool initialized{false};
    float z_min{0.0f};
    float z_max{0.0f};
  };

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  std::mutex cloud_mutex_;

  // Reused between updates to avoid allocating and clearing the whole costmap.
  std::vector<CellInfo> cells_;
  std::vector<unsigned int> touched_cells_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}

#endif  // NAV2_ELEVATION_DIFFERENCE_COSTMAP_PLUGIN__ELEVATION_LAYER_HPP_
