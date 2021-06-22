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
  void getWaypoints(std::vector<ompl::control::Control*> &controls, std::vector<double> &durations, std::vector<ompl::base::State*> states, std::vector<RigidBodyDynamics::Math::Vector2d> &waypoints, unsigned &num_waypoints);
private:
  static const TerrainMap *global_map_; //don't want to make changes to the terrain map in the global planner.

  ompl::control::SpaceInformationPtr si_;
  ompl::base::ProblemDefinitionPtr pdef_;
  ompl::base::PlannerPtr planner_;
  static ompl::base::StateSpacePtr space_ptr_; //needed in isStateValid
  //float G_TOLERANCE_;
};
