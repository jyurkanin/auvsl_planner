#include <fstream>
#include <eigen3/Eigen/Dense>

#include "NNJackalModel.h"
#include "JackalDynamicSolver.h"

class HybridModel{
public:
  HybridModel();
  
  static void init();
  
  void init_state();
  void init_state(float *start_state);
  void step(float vl, float vr);
  void fuse_predictions(Eigen::Matrix<float,NNJackalModel::num_out_features,1> nn_vel, float *bekker_state, float *combined_state);
  void settle();
  
  Eigen::Matrix<float,NNJackalModel::num_in_features,1> feature_vec;
  float vehicle_state[21];
  static constexpr float timestep = .05; //The rate that nn_model operates at.
  NNJackalModel nn_model;
  JackalDynamicSolver bekker_model;
};

