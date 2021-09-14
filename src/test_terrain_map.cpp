#include <iostream>
#include <thread>
#include <vector>
#include <math.h>

#include "auvsl_planner_node.h"
#include "GlobalParams.h"
#include "PlannerVisualizer.h"
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

#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>


unsigned got_init_pose = 0;
geometry_msgs::Pose initial_pose;

unsigned got_grid_map = 0;
geometry_msgs::Pose origin;
float map_res;
unsigned height;
unsigned width;

void get_pos_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  initial_pose = msg->pose.pose;
  got_init_pose = 1;
}

int main(int argc, char **argv){
  
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;

  ROS_INFO("Starting up test_terrain_node\n");
  
  int plot_res;
  nh.getParam("/TerrainMap/plot_res", plot_res); 
  
  GlobalParams::load_params(&nh);
  ompl::RNG::setSeed(GlobalParams::get_seed());
  
  ros::Rate loop_rate(10);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf(tf_buffer);
  costmap_2d::Costmap2DROS costmap("my_costmap", tf_buffer); //This does nothing apparently. I think it needs a lot more work to set it to subscribe to everything and recieve an actual costmap. But I don't know how and I can't find out.
  
  ros::ServiceClient localization_srv = nh.serviceClient<std_srvs::Empty>("/rtabmap/set_mode_localization");
  std_srvs::Empty empty_srv;
  localization_srv.waitForExistence();
  if(localization_srv.call(empty_srv)){
      ROS_INFO("Localization mode set");
  }
  else{
      ROS_INFO("Failed to set Localization mode set");
  }
  
  
  ROS_INFO("Getting OctoTerrainMap");
  OctoTerrainMap terrain_map(costmap.getCostmap());
  ROS_INFO("Constructed terrain map");

  /*
  std::ofstream log_file;
  log_file.open("/home/justin/elev.csv", std::ofstream::out);
  log_file << "x,y,alt\n";
  
  //map_res = .1;
  height = 100;
  width = 100;
  
  float x;
  float y;
  float alt = 0;
  for(int j = 0; j < terrain_map.rows_; j++){
      for(int i = 0; i < terrain_map.cols_; i++){
          x = (i*terrain_map.map_res_) + terrain_map.x_origin_;
          y = (j*terrain_map.map_res_) + terrain_map.y_origin_;
        
          alt = terrain_map.getAltitude(x, y, alt);
          log_file << x << "," << y << "," << alt << "\n";
      }
  }
  
  log_file.close();
  */
  
  
  
  JackalDynamicSolver::set_terrain_map((TerrainMap*) &terrain_map);
  JackalDynamicSolver::init_model(2);

  
  JackalDynamicSolver solver;
  
  solver.simulateRealTrajectory("/home/justin/code/AUVSL_ROS/bags/odom.csv", "/home/justin/code/AUVSL_ROS/bags/joint_states_10hz.csv");
  
  JackalDynamicSolver::del_model();
  
  ROS_INFO("test_terrain is exiting");
}
