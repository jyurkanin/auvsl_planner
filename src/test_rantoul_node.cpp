#include <iostream>
#include <thread>
#include <vector>
#include <math.h>
#include <stdlib.h>

#include "auvsl_planner_node.h"
#include "GlobalParams.h"
#include "JackalDynamicSolver.h"
#include "OctoTerrainMap.h"


#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <rbdl/rbdl.h>

#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <string>





void simulateRealTrajectory(const char *odom_fn, const char *joint_state_fn, float *X_final){
  JackalDynamicSolver solver;
  
  //================== Step 1. get starting position and orientation ============================
  std::ifstream odom_file(odom_fn);
  const int num_odom_cols = 90;
  std::string line;
  std::getline(odom_file, line); //skip first line
  
  float Xn[21];
  float Xn1[21];
  for(int i = 0; i < 21; i++){
    Xn[i] = 0;
  }
  
  double ignoreme;
  
  char comma;
  
  odom_file >> ignoreme >> comma; //index.
  
  odom_file >> Xn[0] >> comma;
  odom_file >> Xn[1] >> comma;
  odom_file >> Xn[2] >> comma;
  
  odom_file >> Xn[3] >> comma;
  odom_file >> Xn[4] >> comma;
  odom_file >> Xn[5] >> comma;
  odom_file >> Xn[10] >> comma;
  
  //==================== Step 2. Allow vehicle to come to rest/reach equillibrium sinkage before moving =================
  
  ROS_INFO("Starting z position: %f %f %f", Xn[0], Xn[1], Xn[2]);
  
  solver.stabilize_sinkage(Xn, Xn1);
  //solve(Xn, Xn1, 10.0f);
  for(int i = 0; i < 21; i++){
    Xn[i] = Xn1[i];
  }
  
  odom_file.close();

  //================== Step 3. Do the simulation bro  ============================
  std::ifstream joint_file(joint_state_fn);
  for(int i = 0; i < 1; i++){
    std::getline(joint_file, line);
  }
  
  float time;
  float front_left_vel;
  float front_right_vel;
  float back_left_vel;
  float back_right_vel;
  
  while(joint_file.peek() != EOF){
    joint_file >> time >> comma;
    joint_file >> front_left_vel >> comma;
    joint_file >> front_right_vel >> comma;
    joint_file >> back_left_vel >> comma;
    joint_file >> back_right_vel;
    
    Xn[17] = front_right_vel + 1e-5;
    Xn[18] = back_right_vel + 1e-5;
    Xn[19] = front_left_vel + 1e-5;
    Xn[20] = back_left_vel + 1e-5;
    
    
    solver.solve(Xn, Xn1, .02f);
    for(int i = 0; i < 21; i++){
      Xn[i] = Xn1[i];
    }
    
    if(!solver.terrain_map_->isStateValid(Xn[0], Xn[1])){
      ROS_INFO("State is equal to LIBERAL BULLSHIT");
      break;
    }
  }
  
  joint_file.close();
  
  for(unsigned i = 0; i < 21; i++){
    X_final[i] = Xn1[i]; 
  }
  
  return;
}







int main(int argc, char **argv){
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  
  ROS_INFO("Starting up test_terrain_node\n");
  
  int plot_res;
  nh.getParam("/TerrainMap/plot_res", plot_res); 
  
  GlobalParams::load_params(&nh);
  srand(GlobalParams::get_seed());

  ros::Rate loop_rate(10);
  
  SimpleTerrainMap simple_terrain_map;
  
  char ignore;
  float X_final[21];
  float X_start[21];
  
  JackalDynamicSolver::set_terrain_map((TerrainMap*) &simple_terrain_map);
  JackalDynamicSolver::init_model(2);
  simple_terrain_map.test_bekker_data_ = lookup_soil_table(3);
  
  
  //simulateRealTrajectory("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/0_odom_t265.csv", "/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/0_joint_states.csv", X_final);
  //simulateRealTrajectory("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/1_odom_t265.csv", "/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/1_joint_states.csv", X_final);
  //simulateRealTrajectory("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/2_odom_t265.csv", "/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/2_joint_states.csv", X_final);
  simulateRealTrajectory("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/3_odom_t265.csv", "/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/3_joint_states.csv", X_final);
  /*simulateRealTrajectory("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/4_odom_t265.csv", "/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/4_joint_states.csv", X_final);
  simulateRealTrajectory("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/5_odom_t265.csv", "/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/5_joint_states.csv", X_final);
  simulateRealTrajectory("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/6_odom_t265.csv", "/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/6_joint_states.csv", X_final);
  simulateRealTrajectory("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/7_odom_t265.csv", "/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/7_joint_states.csv", X_final);
  simulateRealTrajectory("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/8_odom_t265.csv", "/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/8_joint_states.csv", X_final);
  simulateRealTrajectory("/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/9_odom_t265.csv", "/home/justin/code/AUVSL_ROS/bags/rantoul4/test_1/9_joint_states.csv", X_final);
  */  
  ROS_INFO("Final Position %f %f", X_final[0], X_final[1]);
  
  //searchSoilParams();
  //record_altitude_under_path((TerrainMap*) terrain_map);
  
  JackalDynamicSolver::del_model();
  ROS_INFO("test_terrain is exiting");
  
  return 0;
}
