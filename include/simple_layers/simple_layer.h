/*#ifndef SIMPLE_LAYER_H
#define SIMPLE_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PointStamped.h>

namespace simple_layer_namespace {

class SimpleLayer : public costmap_2d::Layer {
public:
    SimpleLayer();
    virtual ~SimpleLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
    ros::Subscriber marker_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_{tf_buffer_};
    void lineMarkerCallback(const visualization_msgs::Marker::ConstPtr& marker);
    void addLethalObstacle(double x, double y, costmap_2d::Costmap2D& master_grid);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

};

} // namespace simple_layer_namespace

#endif // SIMPLE_LAYER_H
*/

#ifndef SIMPLE_LAYER_H_
#define SIMPLE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

namespace simple_layer_namespace
{

class SimpleLayer : public costmap_2d::Layer
{
public:
  SimpleLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  double mark_x_, mark_y_;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif

/*
#ifndef SIMPLE_LAYER_H_
#define SIMPLE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <utility> // For std::pair

namespace simple_layer_namespace
{

class SimpleLayer : public costmap_2d::Layer
{
public:
  SimpleLayer();
  
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  void lineMarkerCallback(const visualization_msgs::Marker::ConstPtr& marker);

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  std::vector<std::pair<double, double>> line_points_; // Store the coordinates of line points
  ros::Subscriber marker_sub_; // ROS subscriber to handle line marker messages
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif
*/
/*
#include<simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>
#include <cmath> // For std::cos, std::sin

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer() {}

void SimpleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  // Subscribe to the /line_marker topic
  marker_sub_ = nh.subscribe("/line_marker", 10, &SimpleLayer::lineMarkerCallback, this);
  ROS_INFO("Received marker");
}



void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}


void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_ || line_points_.empty())
    return;

  // Expand the bounding box to include the line points
  for (auto& pt : line_points_) {
      *min_x = std::min(*min_x, pt.first);
      *min_y = std::min(*min_y, pt.second);
      *max_x = std::max(*max_x, pt.first);
      *max_y = std::max(*max_y, pt.second);
  }
}

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  for (auto& pt : line_points_) {
    unsigned int mx;
    unsigned int my;
    if (master_grid.worldToMap(pt.first, pt.second, mx, my)) {
      if (mx >= unsigned(min_i) && mx <= unsigned(max_i) && my >= unsigned(min_j) && my <= unsigned(max_j)) {
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
    }
  }
}
void SimpleLayer::lineMarkerCallback(const visualization_msgs::Marker::ConstPtr& marker)
{
  if (marker->type != visualization_msgs::Marker::LINE_STRIP &&
      marker->type != visualization_msgs::Marker::LINE_LIST)
  {
    ROS_WARN("Received marker is not a line");
    return;
  }

  // Clear previous points
  line_points_.clear();
  // Get the costmap
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();

  // Convert each point to costmap coordinates and add to the list
  for (const auto& p : marker->points) {
    unsigned int mx, my;
     if (costmap->worldToMap(p.x, p.y, mx, my)) {
            line_points_.emplace_back(mx, my);
      } 
      else {
            ROS_WARN("Failed to convert world coordinates to map coordinates");
        }
  }
}







} // end namespace
*/