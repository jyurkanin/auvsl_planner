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



#define SIM_LEN 6.0f





unsigned got_init_pose = 0;
geometry_msgs::Pose initial_pose;

unsigned got_grid_map = 0;
geometry_msgs::Pose origin;
float map_res;
unsigned height;
unsigned width;

SimpleTerrainMap simple_terrain_map;

typedef struct{
  double vl;
  double vr;
  double ts;
} ODOM_LINE;

typedef struct{
  double ts;
  float x;
  float y;
  float yaw;
} GT_LINE;


std::vector<ODOM_LINE> odom_vec;
std::vector<GT_LINE> gt_vec;


int readOdomFile(std::ifstream &odom_file, ODOM_LINE &odom_line){
  char comma;
  int cmd_mode;    //ignore
  double dist_left;  //ignore
  double dist_right; //ignore
  
  odom_file >> cmd_mode >> comma;
  odom_file >> odom_line.vl >> comma; //velocity left
  odom_file >> dist_left >> comma;
  odom_file >> odom_line.vr >> comma; //velocity right
  odom_file >> dist_right >> comma;
  odom_file >> odom_line.ts; //time (s)
  return odom_file.peek() != EOF;
}

int readGTFile(std::ifstream &gt_file, GT_LINE &gt_line){
  char comma;
  gt_file >> gt_line.ts >> comma;
  gt_file >> gt_line.x >> comma;
  gt_file >> gt_line.y >> comma;
  gt_file >> gt_line.yaw;
  return gt_file.peek() != EOF;
}





void load_files(const char *odom_fn, const char *gt_fn){
  std::ifstream odom_file(odom_fn);
  std::ifstream gt_file(gt_fn);
  
  ODOM_LINE odom_line;
  GT_LINE gt_line;
  
  odom_vec.clear();
  gt_vec.clear();
  
  while(readOdomFile(odom_file, odom_line)){
    odom_vec.push_back(odom_line);
  }
  
  while(readGTFile(gt_file, gt_line)){
    gt_vec.push_back(gt_line);
  }
  
  ROS_INFO("gt vec size %lu", gt_vec.size());
  ROS_INFO("odom vec size %lu", odom_vec.size());

  odom_file.close();
  gt_file.close();
}

void getDisplacement(float &total_len, float &total_ang){
  float dx, dy, dyaw;
  
  total_len = 0;
  total_ang = 0;

  double start_time = gt_vec[0].ts;
  ROS_INFO("Total Duration of File %f", gt_vec[gt_vec.size()-1].ts - start_time);
  
  for(int i = 0; (gt_vec[i].ts - start_time) < SIM_LEN; i++){
    dx = gt_vec[i+1].x - gt_vec[i].x;
    dy = gt_vec[i+1].y - gt_vec[i].y;
    dyaw = gt_vec[i+1].yaw - gt_vec[i].yaw;
    
    total_len += sqrtf((dx*dx) + (dy*dy));
    total_ang += fabs(dyaw);
  }
}

void simulatePeriod(double start_time, float *X_start, float *X_end){
  double dur;
  float Xn[21];
  float Xn1[21];
  for(int i = 0; i < 21; i++){
    Xn[i] = X_start[i];
  }
  
  float dx;
  float dy;
  
  double roll, pitch, yaw, yaw_next;
  
  JackalDynamicSolver solver;
  solver.stabilize_sinkage(Xn, Xn);
  
  for(unsigned idx = 0; (odom_vec[idx].ts - start_time) < SIM_LEN; idx++){
    Xn[17] = std::max(1e-3d, odom_vec[idx].vr);
    Xn[18] = std::max(1e-3d, odom_vec[idx].vr);
    Xn[19] = std::max(1e-3d, odom_vec[idx].vl);
    Xn[20] = std::max(1e-3d, odom_vec[idx].vl);
    
    dur = odom_vec[idx+1].ts - odom_vec[idx].ts;
    solver.solve(Xn, Xn1, dur);
    
    for(int i = 0; i < 21; i++){
      Xn[i] = Xn1[i];
    }
  }
  
  for(int i = 0; i < 21; i++){
    X_end[i] = Xn[i];
  }
}



//odometry filename, ground truth filename
void simulateFiles(float &rel_lin_err, float &rel_ang_err){
  float Xn[21];
  float Xn1[21];
  for(int i = 0; i < 21; i++){
    Xn[i] = 0;
  }
  
  tf::Quaternion initial_quat;
  tf::Quaternion temp_quat;
  
  float dx, dy, dx_gt;
  
  double dt;
  double trans_sum = 0;
  double ang_sum = 0;
  double ang_err;
  
  float total_len, total_ang;
  
  double time = gt_vec[0].ts;      
  Xn[0] = gt_vec[0].x;
  Xn[1] = gt_vec[0].y;
  Xn[2] = .16;
  
  initial_quat.setRPY(0,0,gt_vec[0].yaw);
  initial_quat = initial_quat.normalize();
  
  Xn[3] = initial_quat.getX();
  Xn[4] = initial_quat.getY();
  Xn[5] = initial_quat.getZ();
  Xn[10] = initial_quat.getW();
  
  getDisplacement(total_len, total_ang);
  simulatePeriod(time, Xn, Xn1);
  
  unsigned j;
  for(j = 0; (gt_vec[j].ts - time) < SIM_LEN; j++);
  
  dx = Xn1[0] - gt_vec[j].x;
  dy = Xn1[1] - gt_vec[j].y;
  
  temp_quat = tf::Quaternion(Xn1[3],Xn1[4],Xn1[5],Xn1[10]);
  tf::Matrix3x3 m(temp_quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
      
  ang_err = fabs(yaw - gt_vec[j].yaw);
  ang_err = ang_err > M_PI ?  ang_err - (2*M_PI) : ang_err;
  
  ROS_INFO("Translation Error %f / %f      Heading Error %f / %f", sqrt(dx*dx + dy*dy), total_len, ang_err, total_ang);
  
  rel_lin_err = sqrtf((dx*dx + dy*dy)) / total_len;
  rel_ang_err = ang_err / total_ang;
  
  ROS_INFO("rel_lin_err %f        rel_ang_err %f\n", rel_lin_err, rel_ang_err);
  
  return;
}





int main(int argc, char **argv){
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  
  ROS_INFO("Starting up test_terrain_node\n");
  
  int plot_res;
  nh.getParam("/TerrainMap/plot_res", plot_res); 
  
  GlobalParams::load_params(&nh);
  ompl::RNG::setSeed(GlobalParams::get_seed());
  srand(GlobalParams::get_seed());

  ros::Rate loop_rate(10);

  float total_lin_err = 0;
  float total_ang_err = 0;
  
  float rel_lin_err;
  float rel_ang_err;
  
  char odom_fn[100];
  char gt_fn[100];
  
  int count = 0;
  
  JackalDynamicSolver::set_terrain_map((TerrainMap*) &simple_terrain_map);
  JackalDynamicSolver::init_model(2);
  
  simple_terrain_map.test_bekker_data_ = lookup_soil_table(4);
  //144
  for(int i = 144; i <= 144; i++){
    memset(odom_fn, 0, 100);
    sprintf(odom_fn, "/home/justin/Downloads/CV3/extracted_data/odometry/%04d_odom_data.txt", i);
    ROS_INFO("Reading Odom File %s", odom_fn);
    
    memset(gt_fn, 0, 100);
    sprintf(gt_fn, "/home/justin/Downloads/CV3/localization_ground_truth/%04d_CV_grass_GT.txt", i);
    ROS_INFO("Readin GT File %s", gt_fn);
    
    load_files(odom_fn, gt_fn);
    
    simulateFiles(rel_lin_err, rel_ang_err);
    
    total_lin_err += rel_lin_err*rel_lin_err; //rmsre, square relative error
    total_ang_err += rel_ang_err*rel_ang_err;
    
    count++;
  }
  
  ROS_INFO("RMRSE lin: %f     ang: %f", sqrtf(total_lin_err/(float)count), sqrtf(total_ang_err/(float)count));
  
  JackalDynamicSolver::del_model();
  
  return 0;
}
