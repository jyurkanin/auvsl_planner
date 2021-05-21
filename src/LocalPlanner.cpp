#include "LocalPlanner.h"



LocalPlanner::LocalPlanner(std::vector<Vector2d> waypoints) waypoints_(waypoints){
  curr_waypoint_ = 0;
};
LocalPlanner::~LocalPlanner(){};
