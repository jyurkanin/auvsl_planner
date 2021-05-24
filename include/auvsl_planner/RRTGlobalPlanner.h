#pragma once


#include "VehicleStateSpace.h"
#include "VehicleRRT.h"
#include "TerrainMap.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/config.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <stdio.h>



//    <!-- launch-prefix="gdb -ex run --args" -->




class RRTGlobalPlanner{
public:
  RRTGlobalPlanner(const TerrainMap *map);
  ~RRTGlobalPlanner();

  bool isStateValid(const ompl::base::State *state);
  void plan(std::vector<Vector2d> waypoints, float *vehicle_start_state, Vector2d goal_pos);
private:
  const TerrainMap *global_map_; //don't want to make changes to the terrain map in the global planner.

  ompl::control::SpaceInformationPtr si_;
  ompl::base::ProblemDefinitionPtr pdef_;
  ompl::control::VehicleRRT planner_;
  ompl::base::StateSpacePtr space_ptr_;
  float G_TOLERANCE_;
  ompl::base::PlannerTerminationCondition ptc_;
};
