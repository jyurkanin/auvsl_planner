#include <iostream>
#include <thread>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "GlobalParams.h"
#include "JackalDynamicSolver.h"
#include "TerrainMap.h"

#include <std_srvs/Empty.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <ompl/util/RandomNumbers.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <rbdl/rbdl.h>
#include <tf/tf.h>

#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <string>
#include <fenv.h>



unsigned got_init_pose = 0;
geometry_msgs::Pose initial_pose;

unsigned got_grid_map = 0;
geometry_msgs::Pose origin;
float map_res;
unsigned height;
unsigned width;

SimpleTerrainMap simple_terrain_map;

typedef struct{
  double qd1;
  double qd2;
  double qd3;
  double qd4;
} XOUT_LINE;

std::vector<XOUT_LINE> xout_vec;

void readXoutFile(std::ifstream &xout_file, XOUT_LINE &xout_line){
  char comma;
  double ignore;
  double dist_left;  //ignore
  double dist_right; //ignore

  for(int i = 0; i < 17; i++){
    xout_file >> ignore >> comma;
  }
  
  xout_file >> xout_line.qd1 >> comma;
  xout_file >> xout_line.qd2 >> comma;
  xout_file >> xout_line.qd3 >> comma;
  xout_file >> xout_line.qd4;
}


void load_files(){
  std::ifstream xout_file("/home/justin/code/AUVSL_ROS/src/auvsl_planner/src/test/simulink_p_test.csv");
  
  XOUT_LINE xout_line;
  
  xout_vec.clear();

  std::string line;
  std::getline(xout_file, line);
  
  while(xout_file.peek() != EOF){
    readXoutFile(xout_file, xout_line);
    xout_vec.push_back(xout_line);
  }
  
  ROS_INFO("xout vec size %lu", xout_vec.size());
  xout_file.close();
}


void simulatePeriod(){
  float Xn[21];
  float Xn1[21];
  for(int i = 0; i < 21; i++){
    Xn[i] = 0;
  }
  
  Xn[10] = 1;
  
  float dx;
  float dy;
  
  double roll, pitch, yaw, yaw_next;
  
  JackalDynamicSolver solver;
  //solver.stabilize_sinkage(Xn, Xn);

  //solver.solve(Xn, Xn1, 1.0f);
  for(int i = 0; i < 21; i++){
    //Xn[i] = Xn1[i];
  }
  
  for(int i = 0; i < xout_vec.size(); i++){
    Xn[17] = xout_vec[i].qd1;
    Xn[18] = xout_vec[i].qd2;
    Xn[19] = xout_vec[i].qd3;
    Xn[20] = xout_vec[i].qd4;
    
    //    if((fabs(Xn[17]) + fabs(Xn[18]) + fabs(Xn[19]) + fabs(Xn[20])) < 1e-3f){
    //continue;
    //}
    
    solver.log_xout(Xn);
    solver.euler_method_unit(Xn, Xn1);
    //solver.step(Xn, Xn1, xout_vec[i].qd3, xout_vec[i].qd1);
    
    for(int i = 0; i < 17; i++){
      Xn[i] = Xn1[i];
    } 
  }
}




int main(int argc, char **argv){
  feenableexcept(FE_INVALID | FE_OVERFLOW);
  
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  
  ROS_INFO("Starting up test_terrain_node\n");
  
  int plot_res;
  nh.getParam("/TerrainMap/plot_res", plot_res); 
  
  GlobalParams::load_params(&nh);
  ompl::RNG::setSeed(GlobalParams::get_seed());
  srand(GlobalParams::get_seed());
  
  ros::Rate loop_rate(10);
  
  JackalDynamicSolver::set_terrain_map((TerrainMap*) &simple_terrain_map);
  JackalDynamicSolver::init_model(2);
  
  simple_terrain_map.test_bekker_data_ = lookup_soil_table(0);
  
  load_files();
    
  simulatePeriod();
  
  JackalDynamicSolver::del_model();
  
  return 0;
}
