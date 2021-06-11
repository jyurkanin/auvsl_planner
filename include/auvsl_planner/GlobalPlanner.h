#pragma once


#include "VehicleStateSpace.h"
#include "VehicleRRT.h"
#include "TerrainMap.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/config.h>

//#include <rbdl/Quaternion.h>
#include <rbdl/rbdl.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <stdio.h>
#include <vector>
#include <iostream>

//    <!-- launch-prefix="gdb -ex run --args" -->

//using namespace RigidBodyDynamics;
//using namespace RigidBodyDynamics::Math;


class GlobalPlanner{
public:
  GlobalPlanner(const TerrainMap *map);
  ~GlobalPlanner();

  static bool isStateValid(const ompl::base::State *state);
  int plan(std::vector<RigidBodyDynamics::Math::Vector2d> &waypoints, float *vehicle_start_state, RigidBodyDynamics::Math::Vector2d goal_pos, float goal_tol);
private:
  static const TerrainMap *global_map_; //don't want to make changes to the terrain map in the global planner.

  ompl::control::SpaceInformationPtr si_;
  ompl::base::ProblemDefinitionPtr pdef_;
  ompl::control::VehicleRRT *planner_;
  ompl::base::StateSpacePtr space_ptr_;
    //float G_TOLERANCE_;
};
