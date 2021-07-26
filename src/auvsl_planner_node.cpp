#include <iostream>
#include <thread>
#include <vector>

#include "auvsl_planner_node.h"
#include "GlobalParams.h"
#include "PlannerVisualizer.h"
#include "DStarPlanner.h"
#include "GlobalPlanner.h"
#include "JackalDynamicSolver.h"
#include "OctoTerrainMap.h"


#include <std_srvs/Empty.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <ompl/util/RandomNumbers.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <rbdl/rbdl.h>



using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

//using namespace auvsl_planner;

//launch-prefix="gdb -ex run --args

//This function is for later on when other nodes may want to request a global path plan
//bool globalPlannerCallback(GlobalPathPlan::Request &req, GlobalPathPlan::Response &resp){
//This functionality is provided by the move_base/navigation packages.


unsigned got_init_pose = 0;
geometry_msgs::Pose initial_pose;

void get_pos_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  initial_pose = msg->pose.pose;
  got_init_pose = 1;
}


//1. Solve Global Path Plan
//2. Run Local Path Planner and try to follow global path plan
int main(int argc, char **argv){
  ROS_INFO("Starting up auvsl_planner_node\n");
  
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  
  GlobalParams::load_params(&nh);
  ompl::RNG::setSeed(GlobalParams::get_seed());
  
  ros::Rate loop_rate(10);

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  geometry_msgs::Twist msg;
  
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;
  
  cmd_vel_pub.publish(msg);
  
  ros::ServiceClient localization_srv = nh.serviceClient<std_srvs::Empty>("/rtabmap/set_mode_localization");
  std_srvs::Empty empty_srv;
  localization_srv.waitForExistence();
  if(localization_srv.call(empty_srv)){
      ROS_INFO("Localization mode set");
  }
  else{
      ROS_INFO("Failed to set Localization mode set");
  }
  
  
  ros::Subscriber initial_pose_sub = nh.subscribe("/rtabmap/localization_pose", 100, get_pos_callback);
  while(!got_init_pose){
    ros::spinOnce();
    loop_rate.sleep();
  }
  initial_pose_sub.shutdown();
  
  ros::ServiceClient g_planner_srv = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
  nav_msgs::GetPlan get_plan_srv;

  get_plan_srv.request.start.pose = initial_pose;
  get_plan_srv.request.start.header.frame_id = "map";
  
  
  get_plan_srv.request.goal.pose.position.x = 8;
  get_plan_srv.request.goal.pose.position.y = 0;
  get_plan_srv.request.goal.pose.position.z = .16;
  
  get_plan_srv.request.goal.pose.orientation.x = 0;
  get_plan_srv.request.goal.pose.orientation.y = 0;
  get_plan_srv.request.goal.pose.orientation.z = 0;
  get_plan_srv.request.goal.pose.orientation.w = 1;
  
  get_plan_srv.request.goal.header.frame_id = "map";
  
  get_plan_srv.request.tolerance = 1;
  
  ROS_INFO("Waiting for Global Planner to exist");
  g_planner_srv.waitForExistence();
  ROS_INFO("Calling global planner");
  if(g_planner_srv.call(get_plan_srv)){
      ROS_INFO("global plan got. Size %lu", get_plan_srv.response.plan.poses.size());
  }
  else{
      ROS_INFO("Failed to get global plan.");
  }
  
  return 0;
}
