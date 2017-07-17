#ifndef GRID_LAYER_H_
#define GRID_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include "geometry_msgs/Polygon.h"
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>

# define PI           3.14159265358979323846

namespace simple_layer_namespace{
	class GridLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D{
		public:
		  GridLayer();
		  bool rolling_window_;
		  virtual void onInitialize();
		  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
				                     double* max_y);
		  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
		  bool isDiscretized()
		  {
			return true;
		  }
		  ros::NodeHandle n;
		  ros::Subscriber sub = n.subscribe("polygonPublisher", 1000, &GridLayer::msgSub, this);
		  ros::Subscriber osub = n.subscribe("/move_base_simple/goal", 1000, &GridLayer::goalSub, this);
		  ros::Subscriber rsub = n.subscribe("/move_base/result", 1000, &GridLayer::resultSub, this);
		  std::vector<float> xs;
		  std::vector<float> ys;
		  virtual void msgSub(const geometry_msgs::Polygon::ConstPtr& msg);
		  virtual void goalSub(const geometry_msgs::PoseStamped::ConstPtr& msg);
		  virtual void resultSub(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
		  virtual void matchSize();
		  double goalX, robotX, commandX;
		  bool poseSet;
		  bool goalStatus;

		private:
		  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
		  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
	};
}
#endif

