#include "local_planner_node.h"



LocalPlanner::LocalPlanner(std::vector<float> waypoints) waypoints_(waypoints){
  curr_waypoint_ = 0;
};
LocalPlanner::~LocalPlanner(){};
