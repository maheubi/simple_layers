#include<simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>

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
}


void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  mark_x_ = robot_x + cos(robot_yaw);
  mark_y_ = robot_y + sin(robot_yaw);

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
  if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
}

} // end namespace
/*

#include<simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>
#include <cmath> // For std::cos, std::sin
#include <geometry_msgs/PointStamped.h> // Make sure to include this

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer() {}
SimpleLayer::~SimpleLayer() {}

void SimpleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  // Subscribe to the /line_marker topic
  marker_sub_ = nh.subscribe("/line_marker", 10, &SimpleLayer::lineMarkerCallback, this);
  ROS_INFO("Received marker");
  /*dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  
}



void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}


void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
 /* if (!enabled_ || line_points_.empty())
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
 /* if (!enabled_)
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
 if ((marker->type == visualization_msgs::Marker::LINE_STRIP || marker->type == visualization_msgs::Marker::LINE_LIST) &&
    marker->action == visualization_msgs::Marker::ADD) {
    ROS_WARN("Received marker is not a line");
    for (const auto& point : marker->points) 
    {
      geometry_msgs::PointStamped in, out;
      in.point = point;
      in.header = marker->header;
      try {
        tf2::doTransform(in, out, tf_buffer_.lookupTransform("map", marker->header.frame_id, ros::Time(0)));
        addLethalObstacle(out.point.x, out.point.y, *layered_costmap_->getCostmap());
        }  catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        }
          
    }
  }
}
void SimpleLayer::addLethalObstacle(double x, double y, costmap_2d::Costmap2D& master_grid) {
    unsigned int mx, my;
    if (master_grid.worldToMap(x, y, mx, my)) {
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
}


} // end namespace
*/