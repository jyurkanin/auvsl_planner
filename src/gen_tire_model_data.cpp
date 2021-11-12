#include "JackalDynamicSolver.h"
#include "TerrainMap.h"
#include <math.h>
#include <stdlib.h>



float rand_float(float max, float min){
  return ((max - min)*((float)rand()/RAND_MAX)) + min;
}


void test_rand(){
  for(int i = 0; i < 1000000; i++){
    float temp = rand_float(10,5);
    if(temp < 5){
      ROS_INFO("Failed 1: less than 5: %f", temp);
    }
    if(temp > 10){
      ROS_INFO("Failed 1: greater than 10: %f", temp);
    }
  }


  for(int i = 0; i < 1000000; i++){
    float temp = rand_float(10,-5);
    if(temp < -5){
      ROS_INFO("Failed 2: less than -5: %f", temp);
    }
    if(temp > 10){
      ROS_INFO("Failed 2: greater than 10: %f", temp);
    }
  }
}


//slip_ratio and Fy
int main_slip_ratio(){
  test_rand();
  
  SimpleTerrainMap simple_map;
  JackalDynamicSolver::init_model(2);
  JackalDynamicSolver::set_terrain_map(&simple_map);
  
  JackalDynamicSolver solver;
  
  Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> features;
  
  BekkerData soil_params = lookup_soil_table(0);
  
  features[0] = .003;   //sinkage
  features[1] = 1.398;      //slip_ratio
  features[2] = 0;      //slip_angle
  
  features[3] = soil_params.kc;
  features[4] = soil_params.kphi;
  features[5] = soil_params.n0;
  features[6] = soil_params.n1;
  features[7] = soil_params.phi;
  
  SpatialVector bk;
  SpatialVector nn;
  
  std::ofstream force_log;
  force_log.open("/home/justin/code/AUVSL_ROS/src/auvsl_planner/data/f_tire_train.csv", std::ofstream::out);
  force_log << "slip_angle,Fy\n";
  
  for(unsigned i = 0; i < 10000; i++){
    features[2] = ((M_PI - .1)*i/10000.0f) - (M_PI_2 - .05);
    
    bk = solver.tire_model_bekker(features);
    //nn = solver.tire_model_nn(features);
    
    force_log << features[2] << ',';    
    force_log << bk[4] << '\n';
    
    if(i % 1000){
      ROS_INFO("Number: %d", i);
    }
  }
  
  force_log.close();

  JackalDynamicSolver::del_model();

  return 0;
}



int main(){
  test_rand();
  
  SimpleTerrainMap simple_map;
  JackalDynamicSolver::init_model(2);
  JackalDynamicSolver::set_terrain_map(&simple_map);
  
  JackalDynamicSolver solver;
  
  Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> features;
  
  BekkerData soil_params = lookup_soil_table(0);
  
  features[0] = .003;   //sinkage
  features[1] = 0;      //slip_ratio
  features[2] = .5;      //slip_angle
  
  features[3] = soil_params.kc;
  features[4] = soil_params.kphi;
  features[5] = soil_params.n0;
  features[6] = soil_params.n1;
  features[7] = soil_params.phi;
  
  SpatialVector bk;
  SpatialVector nn;
  
  std::ofstream force_log;
  force_log.open("/home/justin/code/AUVSL_ROS/src/auvsl_planner/data/f_tire_train.csv", std::ofstream::out);
  force_log << "zr,slip_ratio,slip_angle,kc,kphi,n0,n1,phi,Fx,Fy,Fz,Ty\n";
  
  for(unsigned i = 0; i < 1000; i++){
    //features[3] = (70.0f*i/1000.0f) + 30;           //kc
    //features[4] = (3000.0f*i/1000.0f) + 2000;         //kphi
    //features[5] = rand_float(1.3,.3);           //n0
    //features[6] = rand_float(.2,0);             //n1
    features[7] = (.4f*i/1000.0f) + .2f; //phi
    
    bk = solver.tire_model_bekker(features);
    //nn = solver.tire_model_nn(features);
    
    for(int j = 0; j < 8; j++){
      force_log << features[j] << ',';
    }
    
    force_log << bk[3] << ',';
    force_log << bk[4] << ',';
    force_log << bk[5] << ',';
    force_log << bk[1] << '\n';
  }

  
  force_log.close();

  JackalDynamicSolver::del_model();
}


