#pragma once

#include "TerrainMap.h"

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>

#include <rtabmap_ros/GetMap.h>

#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions.h>


#include <unistd.h>
#include <stdlib.h>
#include <vector>


class OctoTerrainMap : TerrainMap{
public:
    OctoTerrainMap(costmap_2d::Costmap2D *occ_grid);
    ~OctoTerrainMap();
    
    BekkerData getSoilDataAt(float x, float y) const override;
    float getAltitude(float x, float y, float z_guess) const override;
    float averageNeighbors(float x, float y, float z_guess) const;
    int isStateValid(float x, float y) const override;
    std::vector<Rectangle*> getObstacles() const override;    
  
    float getMapRes();
    void getBounds(float &max_x, float &min_x, float &max_y, float &min_y) const;
  
    static void get_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);

    unsigned rows_;
    unsigned cols_;
    float map_res_;
    float x_origin_;
    float y_origin_;
  
private:
    costmap_2d::Costmap2D *occ_grid_;
    octomap::OcTree* octomap_;
  
    
    float *occ_grid_blur_;
    float *elev_map_;
  
    static unsigned has_octomap_ground;
    static pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    static pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
};

