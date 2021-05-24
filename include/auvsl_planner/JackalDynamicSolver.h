#include <rbdl/rbdl.h>
#include "utils.h"
#include <eigen3/Eigen/Dense>
#include <fstream>
#include "GlobalParams.h"
#include "TerrainMap.h"

#pragma once


#define NUM_HACK .0001
//#define LOG_DATA 0

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
//using namespace Eigen; //oh boy plz no name collisions.





class PIDController{
public:
  PIDController(float P, float I, float D, float step);
  PIDController(){}
  ~PIDController();

  void reset();
  float step(float err);
private:
  float P_gain;
  float I_gain;
  float D_gain;

  float timestep;

  float err_sum;
  float prev_err;
};




class JackalDynamicSolver{
 public:
  static const int num_hidden_nodes = 20;
  static const int num_in_features = 8;
  static const int num_out_features = 3;

  static std::ofstream log_file;
  static std::ofstream temp_log;

  JackalDynamicSolver();
  ~JackalDynamicSolver();

  static void init_model(int debug);
  static void del_model();
  static void load_nn_gc_model();
  
  void get_tire_sinkages_and_cpts(float *X, float *tire_sinkages, SpatialTransform *cpt_X); //in body frame.
  void get_tire_f_ext(float *X); //in body frame.
  void get_tire_vels(float *X, Vector3d *tire_vels, SpatialTransform *cpt_X);
  
  static Eigen::Matrix<float,num_out_features,1> scale_output(Eigen::Matrix<float,num_out_features,1> labels);
  static Eigen::Matrix<float,num_in_features,1> scale_input(Eigen::Matrix<float,num_in_features,1> features);

  void reset();
  void step(float *X_now, float *X_next, float Vl, float Vr);

  void solve(float *x_init, float *x_end, float vl, float vr, float sim_time);

  void euler_method(float *X, float *Xt1);
  void runge_kutta_method(float *X, float *Xt1);
  float get_timestep();
  void normalize_quat(float *q);
  void ode(float *X, float *Xd);
  void apply_force(SpatialVector wrench, int body);
  void log_xout(float *Xout);

  // private:

  PIDController internal_controller[2];

  static Vector3d base_size;
  static int debug_level;
  static Model *model;
  static float stepsize;
  int timestep;
  VectorNd tau;
  std::vector<SpatialVector> f_ext;

  static float tire_radius;

  static Eigen::Matrix<float,num_hidden_nodes,num_in_features> weight0;
  static Eigen::Matrix<float,num_hidden_nodes,1> bias0;
  static Eigen::Matrix<float,num_hidden_nodes,num_hidden_nodes> weight2;
  static Eigen::Matrix<float,num_hidden_nodes,1> bias2;
  //Eigen::Matrix<float,num_hidden_nodes,num_hidden_nodes> weight4;
  //Eigen::Matrix<float,num_hidden_nodes,1> bias4;
  static Eigen::Matrix<float,num_out_features,num_hidden_nodes> weight4;
  static Eigen::Matrix<float,num_out_features,1> bias4;

  static Eigen::Matrix<float,num_out_features,1> out_mean;
  static Eigen::Matrix<float,num_out_features,1> out_std;

  static Eigen::Matrix<float,num_in_features,1> in_mean;
  static Eigen::Matrix<float,num_in_features,1> in_std;


};
