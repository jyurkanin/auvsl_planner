#include "JackalDynamicSolver.h"
#include "GlobalParams.h"

#include <stdlib.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "gen_data");
  ros::NodeHandle nh;
  GlobalParams::load_params(&nh);

  TerrainMap *terrain_map = new SimpleTerrainMap();
  JackalDynamicSolver::set_terrain_map(terrain_map);
  JackalDynamicSolver::init_model(0);

  JackalDynamicSolver solver;
  
  float start_state[21];
  float temp_state[21];
  float end_state[21];
  
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
    start_state[i] = temp_state[i];
  }  
  
  float Vf;
  float Wz;
  float base_width = .323;
  
  Vf = 2.0f*rand()/RAND_MAX;
  Wz = 2.0f*rand()/RAND_MAX - 1.0f;
  
  ROS_INFO("Starting Sim");
  for(int i = 0; i < 1000000; i++){
      if((1.0f*rand()/RAND_MAX) < .05){
          Vf = 2.0f*rand()/RAND_MAX;
          Wz = 2.0f*rand()/RAND_MAX - 1.0f;
      }
      
      float vl = (Vf - Wz*(base_width/2.0))/.1f;
      float vr = (Vf + Wz*(base_width/2.0))/.1f;

      if((1.0f*rand()/RAND_MAX) < .2){
          vr = (19.0f*rand()/RAND_MAX) + 1.0f;
          vl = (19.0f*rand()/RAND_MAX) + 1.0f;
      }
      
      start_state[17] = vr;
      start_state[18] = vr;
      start_state[19] = vl;
      start_state[20] = vl;
      
      solver.solve(start_state, temp_state, .01);
      solver.log_features(start_state, temp_state, vl, vr);

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
