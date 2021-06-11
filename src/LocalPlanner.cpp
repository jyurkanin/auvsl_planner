#include "LocalPlanner.h"



LocalPlanner::LocalPlanner(){
  curr_waypoint_ = 0;
};

LocalPlanner::~LocalPlanner(){};

void LocalPlanner::setGlobalPath(const std::vector<RigidBodyDynamics::Math::Vector2d> &waypoints){
    for(unsigned i = 0; i < waypoints.size(); i++){
        waypoints_.push_back(Vector2f(waypoints[i][0], waypoints[i][1]));
    }
}

