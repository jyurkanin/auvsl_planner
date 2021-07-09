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


#include <unistd.h>
#include <stdlib.h>
#include <vector>


class OctoTerrainMap : TerrainMap{
public:
    OctoTerrainMap(costmap_2d::Costmap2D *occ_grid);
    ~OctoTerrainMap();
    
    BekkerData getSoilDataAt(float x, float y) const override;
    float getAltitude(float x, float y, float z_guess) const override;
    int isStateValid(float x, float y) const override;
    std::vector<Rectangle*> getObstacles() const override;
    
    float getMapRes();
private:
    costmap_2d::Costmap2D *occ_grid_;
    octomap::OcTree* octomap_;
    
    unsigned rows_;
    unsigned cols_;
    float map_res_;
    unsigned char *oc_grid_;
    
    
    float Xmin, Xmax, Ymin, Ymax;
};

