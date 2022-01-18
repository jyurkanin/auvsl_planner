#include "JackalDynamicSolver.h"
#include "GlobalParams.h"
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>


//parse from residual_data/train3/data_x and data_y
//ts,vx,vy,wz,qd1,qd3
//nvx,nvy,nwz

typedef struct{
  double ts;
  float vx;
  float vy;
  float wz;
  float qd1;
  float qd3;
  float nvx;
  float nvy;
  float nwz;
} RES_LINE;

std::vector<RES_LINE> res_vec;



int readGTFile(std::ifstream &res_file, RES_LINE &res_line){
  char comma;
  res_file >> res_line.ts >> comma;
  res_file >> res_line.vx >> comma;
  res_file >> res_line.vy >> comma;
  res_file >> res_line.wz >> comma;
  res_file >> res_line.qd1 >> comma;
  res_file >> res_line.qd3 >> comma;
  res_file >> res_line.nvx >> comma;
  res_file >> res_line.nvy >> comma;
  res_file >> res_line.nwz;
  
  return res_file.peek() != EOF;
}

void load_file(const char *res_fn){
  std::ifstream res_file(res_fn);
  RES_LINE res_line;
  res_vec.clear();
  
  std::string line;
  std::getline(res_file, line); //this discards the header.
  
  while(readGTFile(res_file, res_line)){
    res_vec.push_back(res_line);
  }
  
  ROS_INFO("gt vec size %lu", res_vec.size());
  res_file.close();
}

/*
void log_features(std::ofstream &feature_file, float *start_state, RES_LINE gt_vel, float *end_state){
  float e_vx = gt_vel.nvx - end_state[11];
  float e_vy = gt_vel.nvy - end_state[12];
  float e_wz = gt_vel.nwz - end_state[16];
  
  feature_file << e_vx << ',';
  feature_file << e_vy << ',';
  feature_file << e_wz << ',';

  
  feature_file << start_state[3] << ',';
  feature_file << start_state[4] << ',';
  feature_file << start_state[5] << ',';
  for(int i = 10; i < 17; i++){
    feature_file << start_state[i] << ',';
  }
  feature_file << start_state[17] << ',';
  feature_file << start_state[19] << '\n';
}
*/

int main(int argc, char **argv){
  ros::init(argc, argv, "gen_data");
  ros::NodeHandle nh;
  GlobalParams::load_params(&nh);
  
  
  load_file("/home/justin/code/AUVSL_ROS/src/auvsl_planner/data/residual_data/train3/train.csv");
  //std::ofstream feature_file("/home/justin/code/AUVSL_ROS/src/auvsl_planner/data/residual_data/features.csv");
  //feature_file << "e_vx,e_vy,e_wz,qx,qy,qz,qw,vx,vy,vz,ax,ay,az,qd1,qd3\n";
  
  
  TerrainMap *terrain_map = new SimpleTerrainMap();
  JackalDynamicSolver::set_terrain_map(terrain_map);
  JackalDynamicSolver::init_model(2);
  
  JackalDynamicSolver solver;
  
  float start_state[21];
  float temp_state[21];
  float end_state[21];
  float Xd[21];
  
  start_state[0] = 10;
  start_state[1] = 0;
  start_state[2] = .15;
  
  start_state[3] = 0;
  start_state[4] = 0;
  start_state[5] = 0;
  start_state[10] = 1;
  
  start_state[6] = 0;
  start_state[7] = 0;
  start_state[8] = 0;
  start_state[9] = 0;
  
  for(unsigned i = 11; i < 21; i++){
    start_state[i] = 0;
  }
  
  start_state[17] = 1e-3f;
  start_state[18] = 1e-3f;
  start_state[19] = 1e-3f;
  start_state[20] = 1e-3f;
  
  solver.solve(start_state, temp_state, 5);
  
  for(unsigned i = 0; i < 21; i++){
    start_state[i] = temp_state[i]; //stabilize in z direction and orientation
    ROS_INFO("%f ", start_state[i]);
  }  
  
  float Vf;
  float Wz;
  float base_width = .323;
  float gt_vel[3];
  
  ROS_INFO("Starting Sim"); //res_vec.size()
  for(unsigned i = 0; i < res_vec.size(); i++){
    //although res_vec is goign to be expressed in body frame,
    //dont need to convert to world frame here because
    //start_state is set so that body and world frames are coincident.
      start_state[11] = res_vec[i].vx;
      start_state[12] = res_vec[i].vy;
      start_state[16] = res_vec[i].wz;
      
      start_state[17] = res_vec[i].qd1;
      start_state[18] = res_vec[i].qd1;
      start_state[19] = res_vec[i].qd3;
      start_state[20] = res_vec[i].qd3;

      gt_vel[0] = res_vec[i].nvx;
      gt_vel[1] = res_vec[i].nvy;
      gt_vel[2] = res_vec[i].nwz;
      
      solver.solve(start_state, temp_state, .05);
      solver.log_features(start_state, gt_vel, temp_state, 0,0);
      
      if(i%100 == 0){
          ROS_INFO("Iteration %d", i);
      }
      
      for(unsigned j = 0; j < 21; j++){
          start_state[j] = temp_state[j];
      }
      
      RigidBodyDynamics::Math::Quaternion quat(temp_state[3], temp_state[4], temp_state[5], temp_state[10]);
      RigidBodyDynamics::Math::Vector3d vec = quat.rotate(RigidBodyDynamics::Math::Vector3d(0,0,1));
      if(vec[2] < 0){
          ROS_INFO("INVALID STATE: ROllOVER");
          break;
      }
      
  }
  ROS_INFO("Sim is done");
  
//  ROS_INFO("Final position: %f %f %f", end_state[0], end_state[1], end_state[2]);


  JackalDynamicSolver::del_model();

    
}
