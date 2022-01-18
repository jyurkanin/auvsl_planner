#include "LinearModel.h"

LinearModel::LinearModel(){
  log_file.open("/home/justin/xout_file.csv", std::ofstream::out);
  log_file << "x,y,yaw\n";
}

LinearModel::~LinearModel(){
  log_file.close();
}

void LinearModel::init_state(){
  float start_state[3] = {0,0,0};
  init_state(start_state);
}

void LinearModel::init_state(float *start_state){
  for(int i = 0; i < 3; i++){
    vehicle_state[i] = start_state[i];
  }
}

void LinearModel::log_xout(){
  log_file << vehicle_state[0] << ",";
  log_file << vehicle_state[1] << ",";
  log_file << vehicle_state[2] << "\n";
}


//this is going to step .05s
//substeps at .005s just to maintain integration accuracy
void LinearModel::step(float vl, float vr){  
  float Xd[3];
  
  for(int i = 0; i < 10; i++){
    float vx = (vl*0.0519) + (vr*0.0352);
    float vy = (vl*0.0038) + (vr*-0.0034);
    float wz = (vl*-0.1747) + (vr*0.1744);
    
    Xd[0] = vx*cosf(vehicle_state[2]) - vy*sinf(vehicle_state[2]);
    Xd[1] = vx*sinf(vehicle_state[2]) + vy*cosf(vehicle_state[2]);
    Xd[2] = wz;
    
    vehicle_state[0] += Xd[0]*.005;
    vehicle_state[1] += Xd[1]*.005;
    vehicle_state[2] += Xd[2]*.005;
  }
  
}
