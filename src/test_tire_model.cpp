#include "JackalDynamicSolver.h"
#include "TerrainMap.h"
#include <math.h>

//integral of cosine is sine.
float f_cos(float x){
  return cosf(x);
}

//integral is .5x^2
float f_x(float x){
  return x;
}

//integral is x
float f_c(float x){
  return 1;
}

int num_steps = 100;

float integrate(float (*func)(float), float upper_b, float lower_b){
  float dtheta = (upper_b - lower_b) / num_steps;
  float eps = dtheta*.1; //adaptive machine epsilon
    
  //trapezoidal rule.
  float sum = 0;
  for(float theta = lower_b; theta < (upper_b - eps); theta += dtheta){
    sum += .5*dtheta*(func(theta + dtheta) + func(theta));
  }
  return sum;
}

//test trapeziod integration rule.
//passes this test.
void test_integrator(){
  float upper = 10.0f;
  float lower = -10.0f;
  
  float expected;
  float test_val = integrate(f_c, upper, lower);
  expected = upper-lower;
  ROS_INFO("Test 1     %f     %f", test_val, expected);

  test_val = integrate(f_x, upper, lower);
  expected = .5*upper*upper - .5*lower*lower;
  ROS_INFO("Test 2     %f     %f", test_val, expected);
  
  test_val = integrate(f_cos, upper, lower);
  expected = sinf(upper) - sinf(lower);
  ROS_INFO("Test 3     %f     %f", test_val, expected);
  
}


struct TireSoilLine{
  float zr,slip_ratio,slip_angle,kc,kphi,n0,n1,phi,Fx,Fy,Fz,Ty;
};

void readTireSoilLine(std::ifstream &ts_file, TireSoilLine &line){
  char comma;
  ts_file >> line.zr >> comma;
  ts_file >> line.slip_ratio >> comma;
  ts_file >> line.slip_angle >> comma;
  
  ts_file >> line.kc >> comma;
  ts_file >> line.kphi >> comma;
  ts_file >> line.n0 >> comma;
  ts_file >> line.n1 >> comma;
  ts_file >> line.phi >> comma;
  
  ts_file >> line.Fx >> comma;
  ts_file >> line.Fy >> comma;
  ts_file >> line.Fz >> comma;
  ts_file >> line.Ty;// >> comma;
}

//test C++ bekker tire-soil against matlab bekker tire-soil
void test_tire_soil_model(){
  std::ifstream ts_file("/home/justin/code/AUVSL_ROS/src/auvsl_planner/src/test/f_tire_no_outliers.csv");
  std::string line;
  std::getline(ts_file, line);
  
  JackalDynamicSolver solver;
  Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> features;
  TireSoilLine ts_line;
  SpatialVector wrench;
  
  //while(ts_file.peek() != EOF){
  for(int i = 0; i < 20; i++){
    readTireSoilLine(ts_file, ts_line);
    
    //due to bug in the matlab script related to n = n0*|s|
    if(ts_line.slip_ratio < 0){ 
      continue;
    }
    //Fy won't match due to bug in matlab code used to generate test data.
    
    features[0] = ts_line.zr;
    features[1] = ts_line.slip_ratio;
    features[2] = ts_line.slip_angle;
    
    features[3] = ts_line.kc;
    features[4] = ts_line.kphi;
    features[5] = ts_line.n0;
    features[6] = ts_line.n1;
    features[7] = ts_line.phi;
    
    wrench = solver.tire_model_bekker(features);

    ROS_INFO("Features  %f   %f   %f", ts_line.zr, ts_line.slip_ratio, ts_line.slip_angle);
    ROS_INFO("C++    Fx %f     Fy %f     Fz %f     Ty %f", wrench[3], wrench[4],  wrench[5], wrench[1]);
    ROS_INFO("Matlab Fx %f     Fy %f     Fz %f     Ty %f", ts_line.Fx, ts_line.Fy, ts_line.Fz, ts_line.Ty);
    ROS_INFO(" ");
  }
}



int main(){  
  SimpleTerrainMap simple_map;
  JackalDynamicSolver::init_model(2);
  JackalDynamicSolver::set_terrain_map(&simple_map);
  
  //test_integrator();
  //test_tire_soil_model();
  
  
  JackalDynamicSolver solver;

  Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> features;
  
  BekkerData soil_params = lookup_soil_table(0);
  
  features[0] = .003;   //sinkage
  features[1] = 0;      //slip_ratio
  features[2] = 0;      //slip_angle
  
  features[3] = soil_params.kc;
  features[4] = soil_params.kphi;
  features[5] = soil_params.n0;
  features[6] = soil_params.n1;
  features[7] = soil_params.phi;
  
  SpatialVector bk;
  SpatialVector nn;
  
  std::ofstream force_log;
  force_log.open("/home/justin/force_log.csv", std::ofstream::out);
  force_log << "slip_angle,bk,nn\n";
  
  for(float vy = -1; vy < 1; vy += .001){
    features[2] = atanf(vy / -1);
    bk = solver.tire_model_bekker(features);
    nn = solver.tire_model_nn(features);
    
    force_log << features[2] << ',' << bk[4] << ',' << nn[4] << '\n';
  }
  
  force_log.close();

  JackalDynamicSolver::del_model();
}
