#include <iostream>
#include <thread>
#include <vector>

#include "auvsl_planner_node.h"
#include "GlobalParams.h"
#include "PlannerVisualizer.h"
#include "DStarPlanner.h"
#include "GlobalPlanner.h"
#include "JackalDynamicSolver.h"

//msgs
#include <auvsl_planner/LocalPathPlan.h>
//srvs
#include <auvsl_planner/GlobalPathPlan.h>

#include <ompl/util/RandomNumbers.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <rbdl/rbdl.h>

//#include <Eigen/Core>
//#include <rbdl/rbdl.h>

//launch-prefix="gdb -ex run --args

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace auvsl_planner;

SimpleTerrainMap *terrain_map;
GlobalPlanner *g_planner;
DStarPlanner *l_planner;


//This function is for later on when other nodes may want to request a global path plan
bool globalPlannerCallback(GlobalPathPlan::Request &req, GlobalPathPlan::Response &resp){
  std::vector<RigidBodyDynamics::Math::Vector2d> waypoints;
  float start_state[17];
  for(int i = 0; i < 17; i++){
    start_state[i] = req.start_state[i];
  }

  RigidBodyDynamics::Math::Vector2d goal_pos(req.goal_pos[0], req.goal_pos[1]);

  g_planner->plan(waypoints, start_state, goal_pos, req.goal_tol);

  for(unsigned i = 0; i < waypoints.size(); i++){
    resp.waypoint_x[i] = waypoints[i][0];
    resp.waypoint_y[i] = waypoints[i][1];
  }

  return true;
}




//1. Solve Global Path Plan
//2. Run Local Path Planner and try to follow global path plan
int main(int argc, char **argv){
  ROS_INFO("Starting up auvsl_planner_node\n");
  ompl::RNG::setSeed(GlobalParams::get_seed());

  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  ros::ServiceServer g_planner_client = nh.advertiseService<GlobalPathPlan::Request, GlobalPathPlan::Response>("global_planner", globalPlannerCallback);
  ros::Publisher l_planner_pub = nh.advertise<LocalPathPlan>("local_planner", 1);  //

  GlobalParams::load_params(&nh);
  ros::Rate loop_rate(10);

  terrain_map = new SimpleTerrainMap();
  terrain_map->generateObstacles();
  terrain_map->generateUnknownObstacles();

  JackalDynamicSolver::set_terrain_map(terrain_map);

  g_planner = new GlobalPlanner(terrain_map);
  l_planner = new DStarPlanner(terrain_map);

  std::vector<RigidBodyDynamics::Math::Vector2d> waypoints;
  float start_state[17] = {50,30,0, 0,0,0,1,  0,0,0,0,0,0,  0,0,0,0};
  RigidBodyDynamics::Math::Vector2d goal_pos(-53 ,30);

  ROS_INFO("Starting global planner.\n");
  waypoints.push_back(RigidBodyDynamics::Math::Vector2d(-90,-90));
  waypoints.push_back(RigidBodyDynamics::Math::Vector2d(-80,-90));
  g_planner->dynamicWindowPlan(waypoints);

  ROS_INFO("Done global planning. On to Local planning.\n");


  //waypoints.push_back(RigidBodyDynamics::Math::Vector2d(-90,-90));
  //waypoints.push_back(RigidBodyDynamics::Math::Vector2d(90,90));


  //l_planner->initWindow();
  //l_planner->setGlobalPath(waypoints);
  //l_planner->runPlanner();

  ROS_INFO("Local Planner Done");

  //while(ros::ok()){
  //ros::spinOnce();
  //loop_rate.sleep();
  //}

  //JackalDynamicSolver::del_model();

  delete terrain_map;
  delete l_planner;
  delete g_planner;

  return 0;
}
