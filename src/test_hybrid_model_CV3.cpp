#include "JackalDynamicSolver.h"
#include "HybridModel.h"
#include "GlobalParams.h"
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <fenv.h>
#include <tf/tf.h>


#define SIM_LEN 6.0f



//parse from residual_data/train3/data_x and data_y
//ts,vx,vy,wz,qd1,qd3
//nvx,nvy,nwz

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

HybridModel *g_hybrid_model;

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
  
  odom_file.close();
  gt_file.close();
}


void getDisplacement(float &total_len, float &total_ang){
  float dx, dy, dyaw;
  
  total_len = 0;
  total_ang = 0;

  double start_time = gt_vec[0].ts;
  //ROS_INFO("Total Duration of File %f", gt_vec[gt_vec.size()-1].ts - start_time);
  
  for(int i = 0; (gt_vec[i].ts - start_time) < SIM_LEN; i++){
    dx = gt_vec[i+1].x - gt_vec[i].x;
    dy = gt_vec[i+1].y - gt_vec[i].y;
    dyaw = gt_vec[i+1].yaw - gt_vec[i].yaw;
    dyaw = fabs(dyaw);
    
    dyaw = dyaw > M_PI ?  dyaw - (2*M_PI) : dyaw;
    
    total_len += sqrtf((dx*dx) + (dy*dy));
    total_ang += fabs(dyaw);
  }
}


void simulatePeriod(double start_time, float *X_start, float *X_end){
  g_hybrid_model->init_state(X_start);
  g_hybrid_model->settle();
  
  float vl, vr, dur;
  for(unsigned idx = 0; (odom_vec[idx].ts - start_time) < SIM_LEN; idx++){
    dur = odom_vec[idx+1].ts - odom_vec[idx].ts; //dur approximately equals .05s, which is the timestep of the hybrid model.
    
    vr = std::max(1e-4d, odom_vec[idx].vr);
    vl = std::max(1e-4d, odom_vec[idx].vl);
    
    g_hybrid_model->step(vl, vr);
    g_hybrid_model->bekker_model.log_xout(g_hybrid_model->vehicle_state);
  }
  
  for(int i = 0; i < 21; i++){
    X_end[i] = g_hybrid_model->vehicle_state[i];
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
  
  //Calculates the smallest angle between two angles.
  ang_err = fabs(yaw - gt_vec[j].yaw);
  ang_err = ang_err > M_PI ?  ang_err - (2*M_PI) : ang_err;
  
  ROS_INFO("Translation Error %f / %f      Heading Error %f / %f", sqrt(dx*dx + dy*dy), total_len, ang_err, total_ang);
  
  rel_lin_err = sqrtf((dx*dx + dy*dy)) / total_len;
  rel_ang_err = ang_err / total_ang;
  
  ROS_INFO("rel_lin_err %f        rel_ang_err %f\n", rel_lin_err, rel_ang_err);
  
  return;
}


//this matches the same result from python.
//Which validates the conversion of the neural network from python to C++
void test_nn(NNJackalModel &nn_model){
  Eigen::Matrix<float,NNJackalModel::num_in_features,1> features;
  Eigen::Matrix<float,NNJackalModel::num_out_features,1> nn_prediction;
  
  for(int i = 0; i < 16; i++){
    features[i] = 1.0f+i;
  }
  
  nn_prediction = nn_model.forward(features);
  
  //ROS_INFO("Vx %f", nn_prediction[0]);
  //ROS_INFO("Vy %f", nn_prediction[1]);
  //ROS_INFO("Wz %f", nn_prediction[2]);
}


//test accuracy by following the cv3 paths for the first 6 seconds.
void test_CV3_paths(){
  float total_lin_err = 0;
  float total_ang_err = 0;
  
  float rel_lin_err;
  float rel_ang_err;
  
  char odom_fn[100];
  char gt_fn[100];
  
  int count = 0;
  
  //this needs to start at 1.
  for(int jj = 1; jj <= 144; jj++){
    memset(odom_fn, 0, 100);
    sprintf(odom_fn, "/home/justin/Downloads/CV3/extracted_data/odometry/%04d_odom_data.txt", jj);
    ROS_INFO("Reading Odom File %s", odom_fn);
    
    memset(gt_fn, 0, 100);
    sprintf(gt_fn, "/home/justin/Downloads/CV3/localization_ground_truth/%04d_CV_grass_GT.txt", jj);
    //ROS_INFO("Readin GT File %s", gt_fn);
    
    load_files(odom_fn, gt_fn);
    
    simulateFiles(rel_lin_err, rel_ang_err);
    
    total_lin_err += rel_lin_err*rel_lin_err; //rmsre, square relative error
    total_ang_err += rel_ang_err*rel_ang_err;
    
    count++;
  }
  
  ROS_INFO("CV3 RMRSE lin: %f     ang: %f", sqrtf(total_lin_err/(float)count), sqrtf(total_ang_err/(float)count));

  /*
  for(int i = 0; i < odom_vec.size(); i++){
      g_hybrid_model->step(odom_vec[i].vl, odom_vec[i].vr);
      g_hybrid_model->bekker_model.log_xout(g_hybrid_model->vehicle_state);
  }
  */
  
}




int main(int argc, char **argv){
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "gen_data");
  ros::NodeHandle nh;
  GlobalParams::load_params(&nh);

  TerrainMap *terrain_map = new SimpleTerrainMap();
  JackalDynamicSolver::set_terrain_map(terrain_map);
  HybridModel::init();

  g_hybrid_model = new HybridModel();
  
  g_hybrid_model->init_state(); //set start pos to 0,0,.16 and orientation to 0,0,0,1
  g_hybrid_model->settle();     //allow the 3d vehicle to come to rest and reach steady state, equillibrium sinkage for tires.
  
  test_CV3_paths();
  
  JackalDynamicSolver::del_model();
}
