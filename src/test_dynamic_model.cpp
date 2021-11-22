#include <iostream>
#include <thread>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "GlobalParams.h"
#include "JackalDynamicSolver.h"
#include "TerrainMap.h"
#include "utils.h"

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



#define EPS 1e-2



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


int test_simulink(){
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



int test_tire_velocities(float pos_x, float pos_y, float pos_z, float qx, float qy, float qz, float qw){  
  float tire_thickness = .05;
  float tire_radius = .1;
  
  Vector3d base_size = Vector3d(.428, .323, .180);
  Vector3d wheel_offset(base_size[0]*.4, (base_size[1] + tire_thickness)/2.0f, .06f + tire_radius); //plus tire_radius because get_tire_sinkages_and_cpts_simple returns the point of contact that is one tire radius below the tire.
  Vector3d tire_trans[4];
  tire_trans[0] = Vector3d(wheel_offset[0], -wheel_offset[1], -wheel_offset[2]);  //front left
  tire_trans[1] = Vector3d(-wheel_offset[0], -wheel_offset[1], -wheel_offset[2]); //back left
  tire_trans[2] = Vector3d(wheel_offset[0], wheel_offset[1], -wheel_offset[2]);   //front right
  tire_trans[3] = Vector3d(-wheel_offset[0], wheel_offset[1], -wheel_offset[2]);  //back right
  
  ROS_INFO("Tire_Trans[0]   %.4f   %.4f   %.4f", tire_trans[0][0], tire_trans[0][1], tire_trans[0][2]);
  
  Quaternion quat(qx,qy,qz,qw);
  Matrix3d rot_w_to_v = quat.toMatrix();
  
  int passed;
  
  JackalDynamicSolver solver;
  
  float expected_vx;
  float expected_vy;
  float expected_vz;
  float expected_wx;
  float expected_wy;
  float expected_wz;
  
  float temp_value;
  
  float sinkages[4];
  Vector3d tire_vels[4];
  SpatialTransform cpt_X[4];
  SpatialTransform tire_X[4];
  
  float Xn[21];
  for(int i = 0; i < 21; i++){
    Xn[i] = 0;
  }
  Xn[0] = pos_x;
  Xn[1] = pos_y;
  Xn[2] = pos_z;
  Xn[3] = qx;
  Xn[4] = qy;
  Xn[5] = qz;
  Xn[10] = qw;
  
  Xn[11] = 10;
  Xn[12] = 0;
  Xn[13] = 0;
  Xn[14] = 0;
  Xn[15] = 0;
  Xn[16] = 2;

  Vector3d ang_vel(Xn[14], Xn[15], Xn[16]);
  Vector3d lin_vel(Xn[11], Xn[12], Xn[13]);
  Vector3d temp_vec;
  Vector3d r_vec;
  Matrix3d skew;
  
  solver.get_tire_sinkages_and_cpts_simple(Xn, sinkages, cpt_X, tire_X);
  solver.get_tire_vels(Xn, tire_vels, cpt_X);
  
  passed = 1;
  ROS_INFO("Test 1 conditions:   vx = 10 wz = 2");
  
  for(int jj = 0; jj < 4; jj++){
    r_vec[0] = tire_trans[jj][0];
    r_vec[1] = tire_trans[jj][1];
    r_vec[2] = tire_trans[jj][2];
    
    skew = get_skew_sym(r_vec);
    
    temp_vec = rot_w_to_v*lin_vel - skew*ang_vel;
    expected_vx = temp_vec[0];
    expected_vy = temp_vec[1];
    expected_vz = temp_vec[2];
    
    ROS_INFO("Expected Velocity %f    %f    %f", expected_vx,      expected_vy,      expected_vz);
    ROS_INFO("Tire Velocity     %f    %f    %f", tire_vels[jj][0], tire_vels[jj][1], tire_vels[jj][2]);
    
    if(fabs(tire_vels[jj][0] - expected_vx) > EPS ||
       fabs(tire_vels[jj][1] - expected_vy) > EPS ||
       fabs(tire_vels[jj][2] - expected_vz) > EPS){
      passed = 0;
    }
  }
  
  if(passed){
    ROS_INFO("Passed Test 1\n");
  }
  else{
    ROS_INFO("Failed Test 1\n");
  }


  
  for(int i = 0; i < 21; i++){
    Xn[i] = 0;
  }
  Xn[0] = pos_x;
  Xn[1] = pos_y;
  Xn[2] = pos_z;
  Xn[3] = qx;
  Xn[4] = qy;
  Xn[5] = qz;
  Xn[10] = qw;
  
  Xn[11] = 10;
  Xn[12] = 7;
  Xn[13] = 4;
  Xn[14] = 5;
  Xn[15] = 9;
  Xn[16] = 2;

  ang_vel = Vector3d(Xn[14], Xn[15], Xn[16]);
  lin_vel = Vector3d(Xn[11], Xn[12], Xn[13]);
  
  solver.get_tire_sinkages_and_cpts_simple(Xn, sinkages, cpt_X, tire_X);
  solver.get_tire_vels(Xn, tire_vels, cpt_X);
  
  passed = 1;
  ROS_INFO("Test 2 conditions:   vx = 10   vy = 7   vz = 4   wx = 5   wy = 9   wz = 2");
  
  for(int jj = 0; jj < 4; jj++){
    r_vec[0] = tire_trans[jj][0];
    r_vec[1] = tire_trans[jj][1];
    r_vec[2] = tire_trans[jj][2];
    
    skew = get_skew_sym(r_vec);
    
    temp_vec = rot_w_to_v*lin_vel - skew*ang_vel;
    expected_vx = temp_vec[0];
    expected_vy = temp_vec[1];
    expected_vz = temp_vec[2];
    
    ROS_INFO("Expected Velocity %f    %f    %f", expected_vx,      expected_vy,      expected_vz);
    ROS_INFO("Tire Velocity     %f    %f    %f", tire_vels[jj][0], tire_vels[jj][1], tire_vels[jj][2]);
    
    if(fabs(tire_vels[jj][0] - expected_vx) > EPS ||
       fabs(tire_vels[jj][1] - expected_vy) > EPS ||
       fabs(tire_vels[jj][2] - expected_vz) > EPS){
      passed = 0;
    }
  }
  
  if(passed){
    ROS_INFO("Passed Test 2\n");
  }
  else{
    ROS_INFO("Failed Test 2\n");
  }
    
  
  
  
  
    
  return 0;
}

//this passes.
void test_function_get_tire_vels(){
  for(int x = 0; x < 2; x++){
    for(int y = 0; y < 2; y++){
      for(int z = 0; z < 2; z++){
        ROS_INFO("Running tests at pos %d %d %d", x, y, z);
        test_tire_velocities((float)x, (float)y, (float)z, 0,0,0,1);
      }
    }
  }
  //test with a random rotation.
  ROS_INFO("Running tests at orientation 0.220556, 0.0021202, 0.4420248, 0.8694623");
  for(int x = 0; x < 2; x++){
    for(int y = 0; y < 2; y++){
      for(int z = 0; z < 2; z++){
        ROS_INFO("Running tests at pos %d %d %d", x, y, z);
        test_tire_velocities((float)x, (float)y, (float)z, 0.220556, 0.0021202, 0.4420248, 0.8694623);
      }
    }
  }
  
  ROS_INFO("Done with test"); 
}



int main(int argc, char **argv){
  feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  
  ROS_INFO("Starting up test_terrain_node\n");
  
  int plot_res;
  nh.getParam("/TerrainMap/plot_res", plot_res); 
  
  GlobalParams::load_params(&nh);
  srand(GlobalParams::get_seed());
  
  ros::Rate loop_rate(10);
  
  JackalDynamicSolver::set_terrain_map((TerrainMap*) &simple_terrain_map);
  JackalDynamicSolver::init_model(2);
  
  simple_terrain_map.test_bekker_data_ = lookup_soil_table(0);

  test_function_get_tire_vels();
  //test_tire_velocities(0,0,0,  0, 0.7071081, 0, 0.7071055);
    
  JackalDynamicSolver::del_model();
  
  return 0;
}
