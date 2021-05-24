#include "LocalPlanner.h"



LocalPlanner::LocalPlanner(){
  curr_waypoint_ = 0;
};

LocalPlanner::~LocalPlanner(){};

void LocalPlanner::setGlobalPath(const std::vector<Vector2d> &waypoints){
  waypoints_ = waypoints;
}

