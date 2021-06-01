#include <iostream>
#include <thread>
#include <vector>

#include "auvsl_planner_node.h"
#include "GlobalParams.h"
#include "PlannerVisualizer.h"
#include "LocalPlanner.h"
#include "GlobalPlanner.h"

//msgs
#include "LocalPathPlan.h"
//srvs
#include "GlobalPathPlan.h"

#include <ompl/util/RandomNumbers.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <rbdl/rbdl.h>

//launch-prefix="gdb -ex run --args


using namespace auvsl_planner;

SimpleTerrainMap *terrain_map;
GlobalPlanner *g_planner;
LocalPlanner *l_planner;


//This function is for later on when other nodes may want to request a global path plan
bool globalPlannerCallback(GlobalPlannerSrv.Request &req, GlobalPlannerSrv.Response &resp){
  std::vector<Vector2d> waypoints;
  float start_state[17];
  for(int i = 0; i < 17; i++){
    start_state[i] = req.start_state[i];
  }

  Vector2d goal_pos(req.goal_pos[0], req.goal_pos[1]);
  
  g_planner->plan(waypoints, start_state, goal_pos, req.goal_tol);
  
  for(unsigned i = 0; i < waypoints.size(); i++){
    resp.waypoint_x[i] = waypoints[i][0];
    resp.waypoint_y[i] = waypoints[i][1];
  }
}




//1. Solve Global Path Plan
//2. Run Local Path Planner and try to follow global path plan
int main(int argc, char **argv){
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  ros::ServiceServer g_planner_client = nh.advertiseService<GlobalPathPlan.Request, GlobalPathPlan.Response>("global_planner", globalPlannerCallback);
  ros::Publisher l_planner_pub = nh.advertise<LocalPathPlan>("local_planner", 1);  //

  GlobalParams::load_params(&nh);
  ros::Rate loop_rate(10);

  terrain_map = new TerrainMap();
  terrain_map->generateObstacles();
  terrain_map->generateUnknownObstacles();
  
  g_planner = new GlobalPlanner(terrain_map);
  l_planner = new LocalPlanner();


  std::vector<Vector2d> waypoints;
  float start_state[17] = {0,0,0, 0,0,0,1, 0,0,0,0,0,0, 0,0,0,0};
  Vector2d goal_pos(-90,-80);
  g_planner->plan(waypoints, start_state, goal_pos, .01);
  
  while(ros::ok()){
    ompl::RNG::setSeed(GlobalParams::get_seed());    
    
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
