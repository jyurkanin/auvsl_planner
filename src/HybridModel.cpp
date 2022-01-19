#include "HybridModel.h"


//const float HybridModel::timestep; //The rate that nn_model operates at.

void HybridModel::init(){
  NNJackalModel::load_nn_model();
  JackalDynamicSolver::init_model(2);
}

HybridModel::HybridModel(){
  //nothing I think
}

void HybridModel::init_state(){
  //                       x,y,z,   qx,qy,qz,  q1,q2,q3,q4,  qw,  vx,vy,vz,  ax,ay,az,  qd1,qd2,qd3,qd4
  float start_state[21] = {0,0,.16, 0, 0, 0,   0, 0, 0, 0,   1,   0, 0, 0,   0, 0, 0,   0,  0,  0,  0};
  init_state(start_state);
}

void HybridModel::init_state(float *start_state){
  for(int i = 0; i < 21; i++){
    vehicle_state[i] = start_state[i];
  }
  feature_vec = Eigen::Matrix<float,NNJackalModel::num_in_features,1>::Zero();
}

//this is going to step .05s
void HybridModel::step(float vl, float vr){
  Eigen::Matrix<float,NNJackalModel::num_out_features,1> nn_prediction_vec;
  
  //added current features to feature_vec like a queue
  for(int i = 7; i > 0; i--){
    int idx = i*2;
    int prev_idx = (i-1)*2;
    feature_vec[idx] = feature_vec[prev_idx];
    feature_vec[idx+1] = feature_vec[prev_idx+1];
  }
  
  feature_vec[0] = vl;
  feature_vec[1] = vr;
  
  nn_prediction_vec = nn_model.forward(feature_vec);
  
  float next_state[21];
  float fused_state[21];
  int substeps = roundf(timestep / .001f);
  for(int i = 0; i < substeps; i++){
    vehicle_state[17] = vehicle_state[18] = vr;
    vehicle_state[19] = vehicle_state[20] = vl;
    
    bekker_model.runge_kutta_method(vehicle_state, next_state);
    
    fuse_predictions(nn_prediction_vec, next_state, fused_state);
    
    for(int j = 0; j < 17; j++){
      vehicle_state[j] = fused_state[j];
    }
  }
  
}

void HybridModel::fuse_predictions(Eigen::Matrix<float,NNJackalModel::num_out_features,1> nn_vel, float *bekker_state, float *fused_state){
  for(int i = 0; i < 21; i++){
    fused_state[i] = bekker_state[i];
  }
  
  Quaternion rbdl_quat(bekker_state[3],bekker_state[4],bekker_state[5],bekker_state[10]);
  Vector3d body_lin_vel = rbdl_quat.toMatrix() * Vector3d(bekker_state[11],bekker_state[12],bekker_state[13]);
  Vector3d fused_body_vel;
  
  //0 mean use bekker model. 1 is nn.
  const float mix = 1.0f;
  
  //perform mixing in body coords
  fused_body_vel[0] = (mix*nn_vel[0]) + ((1.0f - mix)*body_lin_vel[0]);  //Vx
  fused_body_vel[1] = (mix*nn_vel[1]) + ((1.0f - mix)*body_lin_vel[1]);  //Vy
  fused_body_vel[2] = body_lin_vel[2]; //Vz. Dont modify.
  
  Vector3d fused_world_vel = rbdl_quat.toMatrix().transpose() * fused_body_vel;
  fused_state[11] = fused_world_vel[0];
  fused_state[12] = fused_world_vel[1];
  fused_state[16] = (mix*nn_vel[2]) + ((1.0f-mix)*bekker_state[16]); //Mix Wz
}

//only modifies bekker model
//achieve a vehicle orientation that is steady state
//only allow vehicle to translate up and down, roll and pitch
//DO not yaw or move in x and y
//Initial vehicle state is not at steady state because of tire sinkage.
//This function gives the vehicle 1 second at a slow timestep to achieve
//an equillibrium sinkage for all tires. Reaches a corresponding steady
//state vehicle orientation due to the small amount of tire sinkage.
void HybridModel::settle(){
  bekker_model.stabilize_sinkage(vehicle_state, vehicle_state);
}
