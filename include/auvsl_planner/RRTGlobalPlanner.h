#pragma once


#include <ompl/base/SpaceInformation.h>
#include "VehicleStateSpace.h"
#include "VehicleRRT.h"
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/config.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <stdio.h>


//    <!-- launch-prefix="gdb -ex run --args" -->



typedef struct{
  float x, y; //bottom left
  float width, height;
} Rectangle;

class RRTGlobalPlanner{
public:
  RRTGlobalPlanner();
  ~RRTGlobalPlanner();
  
  bool isStateValid(const ompl::base::State *state);
  void plan(std::vector<Vector2d> waypoints, float *vehicle_start_state, Vector2d goal_pos);
private:
  ompl::control::SpaceInformationPtr si_;
  ompl::base::ProblemDefinitionPtr pdef_;
  ompl::control::VehicleRRT planner_;
  ompl::base::StateSpacePtr space_ptr_;
  float G_TOLERANCE_;
  ompl::base::PlannerTerminationCondition ptc_;
};
