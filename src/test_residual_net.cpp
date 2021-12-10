#include "JackalDynamicSolver.h"
#include "solver_res_nn.h"
#include "GlobalParams.h"
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <fenv.h>


//parse from residual_data/train3/data_x and data_y
//ts,vx,vy,wz,qd1,qd3
//nvx,nvy,nwz


typedef struct{
  double vl;
  double vr;
  double ts;
} ODOM_LINE;

std::vector<ODOM_LINE> odom_vec;

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

void load_file(const char *odom_fn){
  std::ifstream odom_file(odom_fn);
  ODOM_LINE odom_line;
  odom_vec.clear();
  
  while(readOdomFile(odom_file, odom_line)){
    odom_vec.push_back(odom_line);
  }
  ROS_INFO("odom vec size %lu", odom_vec.size());

  odom_file.close();
}




void resnet_forward(NNResModel &nn_model, float *start_state, float *residual_est){
  Quaternion rbdl_quat(start_state[3],start_state[4],start_state[5],start_state[10]);
  Vector3d body_lin_vel = rbdl_quat.toMatrix() * Vector3d(start_state[11],start_state[12],start_state[13]);
  
  Eigen::Matrix<float,NNResModel::num_in_features,1> features;
  Eigen::Matrix<float,NNResModel::num_out_features,1> res_prediction;
  
  features[0] = body_lin_vel[0];
  features[1] = body_lin_vel[1];
  features[2] = start_state[16]; //body_lin_vel[2];
  features[3] = start_state[17];
  features[4] = start_state[19];
  
  res_prediction = nn_model.forward(features);
  
  residual_est[0] = res_prediction[0];
  residual_est[1] = res_prediction[1];
  residual_est[2] = res_prediction[2];
  
}

void apply_correction(float *predicted_state, float *residual_est, float *corrected_state){
  for(int i = 0; i < 21; i++){
    corrected_state[i] = predicted_state[i];
  }
  
  Quaternion rbdl_quat(predicted_state[3],predicted_state[4],predicted_state[5],predicted_state[10]);
  Vector3d body_lin_vel = rbdl_quat.toMatrix() * Vector3d(predicted_state[11],predicted_state[12],predicted_state[13]);
  Vector3d corrected_body_vel;
  
  corrected_body_vel[0] = residual_est[0] + predicted_state[11];
  corrected_body_vel[1] = residual_est[1] + predicted_state[12];
  corrected_body_vel[2] = residual_est[2] + predicted_state[16];

  Vector3d corrected_world_vel = rbdl_quat.toMatrix().transpose() * corrected_body_vel;
  corrected_state[11] = corrected_world_vel[0];
  corrected_state[12] = corrected_world_vel[1];
  corrected_state[16] = corrected_world_vel[2];
}

int main(int argc, char **argv){
  feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "gen_data");
  ros::NodeHandle nh;
  GlobalParams::load_params(&nh);
  
  load_file("/home/justin/Downloads/CV3/extracted_data/odometry/0100_odom_data.txt");
  
  TerrainMap *terrain_map = new SimpleTerrainMap();
  JackalDynamicSolver::set_terrain_map(terrain_map);
  JackalDynamicSolver::init_model(2);
  
  JackalDynamicSolver solver;
  
  float start_state[21];
  float temp_state[21];
  float end_state[21];
  float Xd[21];
  
  start_state[0] = 0;
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

  NNResModel nn_res_model;
  nn_res_model.load_nn_model(); //plz remember to do this
  
  float residuals[3];
  float corrected_state[21];
  
  ROS_INFO("Starting Sim");
  for(unsigned i = 0; i < odom_vec.size(); i++){
      start_state[17] = odom_vec[i].vr + 1e-3f;
      start_state[18] = odom_vec[i].vr + 1e-3f;
      start_state[19] = odom_vec[i].vl + 1e-3f;
      start_state[20] = odom_vec[i].vl + 1e-3f;
      
      solver.solve(start_state, temp_state, .05);
      resnet_forward(nn_res_model, start_state, residuals);
      apply_correction(temp_state, residuals, corrected_state);
            
      for(unsigned j = 0; j < 21; j++){
          start_state[j] = corrected_state[j];
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
