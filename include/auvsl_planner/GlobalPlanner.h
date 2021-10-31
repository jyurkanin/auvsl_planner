#pragma once


#include "VehicleStateSpace.h"
#include "VehicleRRT.h"
#include "TerrainMap.h"
#include "OctoTerrainMap.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/config.h>

//#include <rbdl/Quaternion.h>
#include <rbdl/rbdl.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <stdio.h>
#include <vector>
#include <iostream>

//    <!-- launch-prefix="gdb -ex run --args" -->

//using namespace RigidBodyDynamics;
//using namespace RigidBodyDynamics::Math;


namespace auvsl{
  
class GlobalPlanner : public nav_core::BaseGlobalPlanner{
public:
  GlobalPlanner();
  ~GlobalPlanner();

  static bool isStateValid(const ompl::base::State *state);
  int plan(std::vector<RigidBodyDynamics::Math::Vector2d> &waypoints, float *vehicle_start_state, RigidBodyDynamics::Math::Vector2d goal_pos, float goal_tol);
  void getWaypoints(std::vector<ompl::control::Control*> &controls, std::vector<double> &durations, std::vector<ompl::base::State*> states, std::vector<geometry_msgs::PoseStamped> &waypoints, unsigned &num_waypoints);
  void dynamicWindowPlan(std::vector<RigidBodyDynamics::Math::Vector2d> &waypoints);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
  bool makePlan(const geometry_msgs::PoseStamped& start,
                 const geometry_msgs::PoseStamped& goal,
                 std::vector<geometry_msgs::PoseStamped>& plan
                ) override;
    
private:
  static const TerrainMap *global_map_; //don't want to make changes to the terrain map in the global planner.

  ompl::control::SpaceInformationPtr si_;
  ompl::base::ProblemDefinitionPtr pdef_;
  ompl::base::PlannerPtr planner_;
  static ompl::base::StateSpacePtr space_ptr_; //needed in isStateValid
  ompl::control::StatePropagatorPtr dynamic_model_ptr_;
  //float G_TOLERANCE_;
};

}
