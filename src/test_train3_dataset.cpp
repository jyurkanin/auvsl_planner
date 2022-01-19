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


#define SIM_LEN 6




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

void readOdomFile(std::ifstream &odom_file, ODOM_LINE &odom_line){
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
}

void readGTFile(std::ifstream &gt_file, GT_LINE &gt_line){
  char comma;
  gt_file >> gt_line.ts >> comma;
  gt_file >> gt_line.x >> comma;
  gt_file >> gt_line.y >> comma;
  gt_file >> gt_line.yaw;
}

void load_files(const char *odom_fn, const char *gt_fn){
  std::ifstream odom_file(odom_fn);
  std::ifstream gt_file(gt_fn);
  
  ODOM_LINE odom_line;
  GT_LINE gt_line;
  
  odom_vec.clear();
  gt_vec.clear();
  
  while(odom_file.peek() != EOF){
    readOdomFile(odom_file, odom_line);
    odom_vec.push_back(odom_line);
  }
  
  while(gt_file.peek() != EOF){
    readGTFile(gt_file, gt_line);
    gt_vec.push_back(gt_line);
  }
  
  //ROS_INFO("gt vec size %lu", gt_vec.size());
  //ROS_INFO("odom vec size %lu", odom_vec.size());

  odom_file.close();
  gt_file.close();
}

void getDisplacement(float &total_len, float &total_ang){
  float dx, dy, dyaw;
  
  total_len = 0;
  total_ang = 0;

  double stop_time = SIM_LEN + odom_vec[0].ts;
  for(int i = 0; gt_vec[i].ts < stop_time; i++){
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
  
  double stop_time = SIM_LEN + odom_vec[0].ts;
  for(unsigned idx = 0; odom_vec[idx].ts < stop_time; idx++){
    Xn[17] = std::max(1e-4d, odom_vec[idx].vr);
    Xn[18] = std::max(1e-4d, odom_vec[idx].vr);
    Xn[19] = std::max(1e-4d, odom_vec[idx].vl);
    Xn[20] = std::max(1e-4d, odom_vec[idx].vl);
    
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
      
  //ROS_INFO("Velocity:   %f %f %f   %f %f %f", Xn[11], Xn[12], Xn[13],   Xn[14], Xn[15], Xn[16]);
  getDisplacement(total_len, total_ang);
  simulatePeriod(time, Xn, Xn1);

  double stop_time = SIM_LEN + odom_vec[0].ts;
  int j;
  for(j = 0; gt_vec[j].ts < stop_time; j++){}
      
  dx = Xn1[0] - gt_vec[j].x;
  dy = Xn1[1] - gt_vec[j].y;
  
  temp_quat = tf::Quaternion(Xn1[3],Xn1[4],Xn1[5],Xn1[10]);
  tf::Matrix3x3 m(temp_quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
      
  ang_err = yaw - gt_vec[j].yaw;
  ang_err = ang_err > M_PI ?  ang_err - (2*M_PI) : ang_err;

  //ROS_INFO("Final yaw %f   actual final yaw %f", yaw, gt_vec[j].yaw);
  //ROS_INFO("Translation Error %f / %f      Heading Error %f / %f\n", sqrt(dx*dx + dy*dy), total_len, ang_err, total_ang);
  
  rel_lin_err = sqrtf((dx*dx + dy*dy)) / total_len;
  rel_ang_err = ang_err / total_ang;
  
  //if(fabs(dyaw_gt) > .01){
  //}
    
  return;
}




void simulateFileSet(float &total_lin_err, float &total_ang_err){
  total_lin_err = 0;
  total_ang_err = 0;
  
  float rel_lin_err, rel_ang_err;
  int count = 0;
  
  char odom_fn[100];
  char gt_fn[100];  
  
  for(int i = 1; i <= 1; i++){
    memset(odom_fn, 0, 100);
    sprintf(odom_fn, "/home/justin/Downloads/Train3/extracted_data/odometry/%04d_odom_data.txt", i);
      
    memset(gt_fn, 0, 100);
    sprintf(gt_fn, "/home/justin/Downloads/Train3/localization_ground_truth/%04d_Tr_grass_GT.txt", i);
    
    load_files(odom_fn, gt_fn);
    simulateFiles(rel_lin_err, rel_ang_err);
    
    total_lin_err += rel_lin_err*rel_lin_err; //rmsre, square relative error
    total_ang_err += rel_ang_err*rel_ang_err;
      
    count++;
  }

  total_lin_err = sqrtf(total_lin_err/(float)count);
  total_ang_err = sqrtf(total_ang_err/(float)count);
}






void followGradientScalars(){
  const float EPS = 1e-2f;
  float actual_value[4];
  float total_lin_err, total_ang_err;
  float loss[4];
  float fx;
  float lr = 1.0f;


  actual_value[1] = JackalDynamicSolver::force_scalars[1];
  actual_value[2] = JackalDynamicSolver::force_scalars[2];
  
  
  simulateFileSet(total_lin_err, total_ang_err);
  fx = total_lin_err; // + (total_ang_err*.5); //comptue f(x)
  ROS_INFO("Current Performance %f", fx);
  
  
  JackalDynamicSolver::force_scalars[1] += EPS;
  simulateFileSet(total_lin_err, total_ang_err);
  loss[1] = total_lin_err;// + (total_ang_err*.1); //compute f(x+h)
  
  
  JackalDynamicSolver::force_scalars[1] = actual_value[1];
  JackalDynamicSolver::force_scalars[2] += EPS;
  simulateFileSet(total_lin_err, total_ang_err);
  loss[2] = total_lin_err;// + (total_ang_err*.1); //compute f(x+h)
  
  
  ROS_INFO("%f   %f", actual_value[1], actual_value[2]);
  
  
  JackalDynamicSolver::force_scalars[1] = actual_value[1] + (fx - loss[1])*lr;
  JackalDynamicSolver::force_scalars[2] = actual_value[2] + (fx - loss[2])*lr;
}







void followGradientSoil(BekkerData &soil_params){
  const float EPS = 1e-2f;
  BekkerData soil_params_pde;
  BekkerData temp_soil_params;
  float rel_lin_err, rel_ang_err;
  float loss;
  float fx;
  float lr = 1.0f;
  
  simple_terrain_map.test_bekker_data_ = soil_params;
  simulateFileSet(rel_lin_err, rel_ang_err);
  loss = rel_lin_err + (rel_ang_err*.5);
  fx = loss;
  ROS_INFO("Current Performance %f", rel_lin_err);
  
  /*
  simple_terrain_map.test_bekker_data_ = soil_params;
  simple_terrain_map.test_bekker_data_.kphi += 1;
  simulateFileSet(rel_lin_err, rel_ang_err);
  loss = rel_lin_err + (rel_ang_err*.5);
  soil_params_pde.kphi = fx - loss;
  
  simple_terrain_map.test_bekker_data_ = soil_params;
  simple_terrain_map.test_bekker_data_.phi += .01;
  simulateFileSet(rel_lin_err, rel_ang_err);
  loss = rel_lin_err + (rel_ang_err*.5);
  soil_params_pde.phi = fx - loss;
  

  simple_terrain_map.test_bekker_data_ = soil_params;
  simple_terrain_map.test_bekker_data_.kc += .1;
  simulateFileSet(rel_lin_err, rel_ang_err);
  loss = rel_lin_err + (rel_ang_err*.5);
  soil_params_pde.kc = fx - loss;
  */
  
  simple_terrain_map.test_bekker_data_ = soil_params;
  simple_terrain_map.test_bekker_data_.n0 += .01;
  simulateFileSet(rel_lin_err, rel_ang_err);
  loss = rel_lin_err + (rel_ang_err*.5);
  soil_params_pde.n0 = fx - loss;

  
  
  ROS_INFO("dkc %f kc %f    dphi %f phi %f    dkphi %f kphi %f    dn0 %f n0 %f", soil_params_pde.kc, soil_params.kc, soil_params_pde.phi, soil_params.phi,   soil_params_pde.kphi, soil_params.kphi,  soil_params_pde.n0, soil_params.n0);
  
  //soil_params.kphi += soil_params_pde.kphi*lr;
  //soil_params.phi += soil_params_pde.phi*lr;
  //soil_params.kc += soil_params_pde.kc*lr;
  soil_params.n0 += soil_params_pde.n0*lr;
}




//gradient descent for soil parameters
int main_stupid(int argc, char **argv){
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  
  ROS_INFO("Starting up test_terrain_node\n");
  
  int plot_res;
  nh.getParam("/TerrainMap/plot_res", plot_res); 
  
  GlobalParams::load_params(&nh);
  ompl::RNG::setSeed(GlobalParams::get_seed());
  srand(GlobalParams::get_seed());

  ros::Rate loop_rate(10);

  float rel_lin_err;
  float rel_ang_err;
  
  char odom_fn[100];
  char gt_fn[100];

  int count = 0;

  JackalDynamicSolver::set_terrain_map((TerrainMap*) &simple_terrain_map);
  JackalDynamicSolver::init_model(2);
  
  simple_terrain_map.test_bekker_data_ = lookup_soil_table(0);

  BekkerData soil_params = lookup_soil_table(0);
  for(int j = 0; j < 100; j++){
    followGradientSoil(soil_params);
  }

  JackalDynamicSolver::del_model();
  
  return 0;
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
  //for(float kphi = 1000; kphi < 2000; kphi += 100.0f){
  //for(float n0 = .65; n0 < .75; n0 += .01f){
    //simple_terrain_map.test_bekker_data_.kphi = kphi;
    //simple_terrain_map.test_bekker_data_.n0 = n0;
    
    //ROS_INFO("n0 %f", n0);

  std::ofstream param_graph_file("/home/justin/param_graph.csv");
  param_graph_file << "param,lin_err,ang_err\n";
  
  float test_value;

  simple_terrain_map.test_bekker_data_ = lookup_soil_table(0);
  for(float kphi = 1800; kphi < 2200; kphi += 50.0f){
    total_lin_err = 0;
    total_ang_err = 0;
    count = 0;
    
    simple_terrain_map.test_bekker_data_.n0 = .695;
    simple_terrain_map.test_bekker_data_.kc = 26.0f;
    simple_terrain_map.test_bekker_data_.kphi = kphi;
    
    for(int i = 1; i <= 1; i++){
      memset(odom_fn, 0, 100);
      sprintf(odom_fn, "/home/justin/Downloads/Train3/extracted_data/odometry/%04d_odom_data.txt", i);
      
      memset(gt_fn, 0, 100);
      sprintf(gt_fn, "/home/justin/Downloads/Train3/localization_ground_truth/%04d_Tr_grass_GT.txt", i);
      
      load_files(odom_fn, gt_fn);
      
      simulateFiles(rel_lin_err, rel_ang_err);
      
      total_lin_err += rel_lin_err*rel_lin_err; //rmsre, square relative error
      total_ang_err += rel_ang_err*rel_ang_err;
      
      count++;
    }

    ROS_INFO("kphi %f", kphi);
    ROS_INFO("RMRSE lin: %f     ang: %f", sqrtf(total_lin_err/(float)count), sqrtf(total_ang_err/(float)count));
  }
  
  param_graph_file.close();
  
  
  JackalDynamicSolver::del_model();
  
  return 0;
}
