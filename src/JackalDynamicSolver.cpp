#include "JackalDynamicSolver.h"
//#include "auvsl_planner_node.h"
#include <unistd.h>
#include <math.h>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <stdlib.h>
#include <fstream>
#include <cfenv>

#include <tf/tf.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


//nuclear option for debugging floats
//#pragma STDC FENV_ACCESS ON
void check_fp_exceptions(){
  if(std::fetestexcept(FE_DIVBYZERO)){
    ROS_INFO("divide by zero");
  }
  if(std::fetestexcept(FE_INEXACT)){
    ROS_INFO("inexact");
  }
  if(std::fetestexcept(FE_INVALID)){
    ROS_INFO("invalid");
  }
  if(std::fetestexcept(FE_OVERFLOW)){
    ROS_INFO("overflow");
  }
  if(std::fetestexcept(FE_UNDERFLOW)){
    ROS_INFO("underflow");
  }
}


void break_on_me(){}
    

struct BekkerModel{
public:
  static constexpr float k0 = 2195;
  static constexpr float Au = 192400;
  static constexpr float K = .0254;
  static constexpr float density = 1681;
  static constexpr float c = 8.62;
  static constexpr float pg = 100; //15psi
  
  static float zr;
  static float slip_ratio;
  static float slip_angle;
  
  static float kc;
  static float kphi;
  static float n0;
  static float n1;
  static float phi;

  static float n;
  static float pgc;
  static float R;
  static float b;
  static float ze;
  static float zu;
  static float ku;
  
  static float theta_f;
  static float theta_r;
  static float theta_c;
  
  static const int num_steps = 100;
  
  static float sigma_x_cf(float theta){
    //in some cases, cosf - cosf can return negative due to theta != theta_f, but very close.
    float diff = cosf(theta) - cosf(theta_f);//std::max(0.0f,cosf(theta) - cosf(theta_f));
    float temp = ((kc/b) + kphi)*std::pow(R*(diff), n);
    return temp;
  }
  static float sigma_x_cc(float theta){return ((kc/b)+kphi)*std::pow(ze, n); }
  static float sigma_x_cr(float theta){
    float diff = cosf(theta) - cosf(theta_r);
    float temp = ku*std::pow(R*(diff), n);
    return temp;
  }
  
  static float jx_cf(float theta){
    float temp = R*((theta - theta_f) - (1 - slip_ratio)*(sinf(theta)-sinf(theta_f)));
    return temp;
  }
  static float jx_cc(float theta){return R*((theta_c - theta_f) + (1 - slip_ratio)*sinf(theta_f) - sinf(theta_c) + (slip_ratio*cosf(theta_c)*tanf(theta))); }
  static float jx_cr(float theta){return R*((2*theta_c + theta - theta_f) - (1 - slip_ratio)*(sinf(theta) - sinf(theta_f)) - 2*sinf(theta_c)); }
  
  static float jy_cf(float theta){
    return R*(1-slip_ratio) * tanf(slip_angle) * (theta_f - theta);
  }
  static float jy_cc(float theta){return R*(1-slip_ratio) * tanf(slip_angle) * (theta_f - theta + sinf(theta_c) - cosf(theta_c)*tanf(theta)); }
  static float jy_cr(float theta){return R*(1-slip_ratio) * tanf(slip_angle) * (theta_f - theta - 2*theta_c + 2*sinf(theta_c)); }
  
  static float j_cf(float theta){
    float temp = sqrtf(std::pow(jy_cf(theta), 2) + std::pow(jx_cf(theta), 2));
    return temp;
  }
  static float j_cc(float theta){return sqrtf(std::pow(jy_cc(theta), 2) + std::pow(jx_cc(theta), 2)); }
  static float j_cr(float theta){return sqrtf(std::pow(jy_cr(theta), 2) + std::pow(jx_cr(theta), 2)); }
  
  static float tau_x_cf(float theta){
    //smart divide
    float j_cf_theta = j_cf(theta);
    float temp = j_cf_theta == 0 ? 0 : jx_cf(theta)/j_cf_theta;
    return (-temp) * (c + (sigma_x_cf(theta)*tanf(phi))) * (1 - (exp(-j_cf_theta/K)));
  }
  static float tau_x_cc(float theta){
    float j_cc_theta = j_cc(theta);
    float temp = j_cc_theta == 0 ? 0 : jx_cc(theta)/j_cc_theta;
    return (-temp) * (c + (sigma_x_cc(theta)*tanf(phi))) * (1 - (exp(-j_cc_theta/K)));
  } 
  static float tau_x_cr(float theta){
    float j_cr_theta = j_cr(theta);
    float temp = j_cr_theta == 0 ? 0 : jx_cr(theta)/j_cr_theta;
    return (-temp) * (c + (sigma_x_cr(theta)*tanf(phi))) * (1 - (exp(-j_cr_theta/K)));
  }
  
  static float tau_y_cf(float theta){
    float j_cf_theta = j_cf(theta);
    float temp = j_cf_theta == 0 ? 0 : jy_cf(theta)/j_cf_theta;
    return (-temp) * (c + (sigma_x_cf(theta)*tanf(phi))) * (1 - (exp(-j_cf_theta/K)));
  }
  static float tau_y_cc(float theta){
    float j_cc_theta = j_cc(theta);
    float temp = j_cc_theta == 0 ? 0 : jy_cc(theta)/j_cc_theta;
    return (-temp) * (c + (sigma_x_cc(theta)*tanf(phi))) * (1 - (exp(-j_cc_theta/K)));
  }
  static float tau_y_cr(float theta){
    float j_cr_theta = j_cr(theta);
    float temp = j_cr_theta == 0 ? 0 : jy_cr(theta)/j_cr_theta;
    return (-temp) * (c + (sigma_x_cr(theta)*tanf(phi))) * (1 - (exp(-j_cr_theta/K)));
  }


  static float Fy_eqn3(float theta){return tau_y_cc(theta)*std::pow(1.0f/cosf(theta), 2); }
  
  static float Fz_eqn1(float theta){return (sigma_x_cf(theta)*cosf(theta)) + (tau_x_cf(theta)*sinf(theta)); }
  static float Fz_eqn2(float theta){return (sigma_x_cr(theta)*cosf(theta)) + (tau_x_cr(theta)*sinf(theta)); }
  
  static float Fx_eqn1(float theta){return (tau_x_cf(theta)*cosf(theta)) - (sigma_x_cf(theta)*sinf(theta)); } 
  static float Fx_eqn2(float theta){return (tau_x_cr(theta)*cosf(theta)) - (sigma_x_cr(theta)*sinf(theta)); }
  static float Fx_eqn3(float theta){return tau_x_cc(theta)*(std::pow(1.0f/cosf(theta), 2)); }
  
  static float Ty_eqn1(float theta){return tau_x_cc(theta)*(std::pow(1.0f/cosf(theta), 2)); }

  static float integrate(float (*func)(float), float upper_b, float lower_b){
    float dtheta = (upper_b - lower_b) / num_steps;
    float eps = dtheta*.1; //adaptive machine epsilon
    
    //trapezoidal rule.
    float sum = 0;
    for(float theta = lower_b; theta < (upper_b - eps - dtheta); theta += dtheta){
      sum += .5*dtheta*(func(theta + dtheta) + func(theta));
    }

    //last iteration is different to ensure no floating point error occurs.
    //This ensures integration goes exactly to the upper bound and does not exceed it in the slightest.
    sum += .5*dtheta*(func(upper_b) + func(upper_b - dtheta));
    
    return sum;
  }
};





float BekkerModel::zr;
float BekkerModel::slip_ratio;
float BekkerModel::slip_angle;
  
float BekkerModel::kc;
float BekkerModel::kphi;
float BekkerModel::n0;
float BekkerModel::n1;
float BekkerModel::phi;

float BekkerModel::n;
float BekkerModel::pgc;
float BekkerModel::R;
float BekkerModel::b;
float BekkerModel::ze;
float BekkerModel::zu;
float BekkerModel::ku;
  
float BekkerModel::theta_f;
float BekkerModel::theta_r;
float BekkerModel::theta_c;






PIDController::PIDController(float P, float I, float D, float step){
  P_gain = P;
  I_gain = I;
  D_gain = D;

  timestep = step;
  
  reset();
}

PIDController::~PIDController(){}

void PIDController::reset(){
  err_sum = 0;
  prev_err = 0;
}

float PIDController::step(float err){
  float d_err = (err - prev_err)/timestep;
  float tau = (P_gain*err) - (D_gain*d_err);
  prev_err = err;

  return tau;
}



//Function is not actually used.
SpatialVector get_body_vel(SpatialVector world_vel, float *X){
    Quaternion quat(X[3], X[4], X[5], X[10]);
    Vector3d r(X[0], X[1], X[2]);
    SpatialTransform X_base(quat.toMatrix(), r);
    return X_base.apply(world_vel); //transform from world to body coordinates.    
}

//Function is not actually used.
SpatialVector get_world_force(SpatialVector f_ext_body, float *X){
    Quaternion quat(X[3], X[4], X[5], X[10]);
    Vector3d r(X[0], X[1], X[2]);
    SpatialTransform X_base(quat.toMatrix(), r);
    return X_base.applyTranspose(f_ext_body);  //transform from body to world coordinates.
}




//static stuff


std::ofstream JackalDynamicSolver::log_file;
std::ofstream JackalDynamicSolver::feature_log;
std::ofstream JackalDynamicSolver::temp_log;


Vector3d JackalDynamicSolver::base_size;
int JackalDynamicSolver::debug_level;
Model* JackalDynamicSolver::model = 0;
float JackalDynamicSolver::stepsize;
float JackalDynamicSolver::tire_radius;
float JackalDynamicSolver::tire_thickness;

const TerrainMap* JackalDynamicSolver::terrain_map_;

Eigen::Matrix<float,JackalDynamicSolver::num_hidden_nodes,JackalDynamicSolver::num_in_features> JackalDynamicSolver::weight0;
Eigen::Matrix<float,JackalDynamicSolver::num_hidden_nodes,1> JackalDynamicSolver::bias0;
Eigen::Matrix<float,JackalDynamicSolver::num_hidden_nodes,JackalDynamicSolver::num_hidden_nodes> JackalDynamicSolver::weight2;
Eigen::Matrix<float,JackalDynamicSolver::num_hidden_nodes,1> JackalDynamicSolver::bias2;
//Eigen::Matrix<float,JackalDynamicSolver::num_hidden_nodes,JackalDynamicSolver::num_hidden_nodes> weight4;
//Eigen::Matrix<float,JackalDynamicSolver::num_hidden_nodes,1> bias4;
Eigen::Matrix<float,JackalDynamicSolver::num_out_features,JackalDynamicSolver::num_hidden_nodes> JackalDynamicSolver::weight4;
Eigen::Matrix<float,JackalDynamicSolver::num_out_features,1> JackalDynamicSolver::bias4;

Eigen::Matrix<float,JackalDynamicSolver::num_out_features,1> JackalDynamicSolver::out_mean;
Eigen::Matrix<float,JackalDynamicSolver::num_out_features,1> JackalDynamicSolver::out_std;

Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> JackalDynamicSolver::in_mean;
Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> JackalDynamicSolver::in_std;

float JackalDynamicSolver::force_scalars[4];


//actual class members

JackalDynamicSolver::JackalDynamicSolver(){
  stepsize = GlobalParams::get_timestep();
  tau = VectorNd::Zero(model->qdot_size);
  
  
  internal_controller[0] = PIDController(GlobalParams::get_p_gain(),0,0, stepsize);
  internal_controller[1] = PIDController(GlobalParams::get_p_gain(),0,0, stepsize);
  
  
  for(int i = 0; i < model->mBodies.size(); i++){
    f_ext.push_back(SpatialVector::Zero());
  }
  
  timestep = 0;
}

void JackalDynamicSolver::init_model(int debug){
  debug_level = debug;//;GlobalParams::get_debug_level();
  if(debug_level == 2 && !log_file.is_open()){
      ROS_INFO("DEBUG LEVEL 2");
      log_file.open("/home/justin/xout_file.csv", std::ofstream::out);
      log_file << "qw,qx,qy,qz,x,y,z,wx,wy,wz,vx,vy,vz,q1,q2,q3,q4,qd1,qd2,qd3,qd4\n";
      
      //temp_log.open("/home/justin/feature_file.csv");
      //temp_log << "x,y,z,rx,ry,rz\n";
  }

  if(log_file.bad()){
    ROS_INFO("Log FILE BAD BRO 1");
  }
  
  feature_log.open("/home/justin/features.csv", std::ofstream::out);
  feature_log << "x,y,z,roll,pitch,vx,vy,az,qd1,qd2,qd3,qd4,vl,vr,vxn,vyn,azn,dvx,dvy,dvz,dwx,dwy,dwz\n";
  
  Vector3d initial_pos(0,0,0);
  
  unsigned int base_id;
  unsigned int tire_id[4];
  
  Body tire[4];
  Body floating_base;
  
  Joint tire_joints[4];
  Joint floating_joint;
  
  if(model){
    return;
  }
  
  model = new Model();
  model->gravity = Vector3d(0., 0., -9.8); //gravity off. lmao
  
  float base_mass = 13; //kg
  float tire_mass = 1;
  
  tire_thickness = .05;
  tire_radius = .1;
  
  base_size = Vector3d(.428, .323, .180);
  Vector3d wheel_offset(base_size[0]*.4, (base_size[1] + tire_thickness)/2, .06);
  Vector3d tire_trans[4];

  
  tire_trans[0] = Vector3d(wheel_offset[0], -wheel_offset[1], -wheel_offset[2]);  //front left
  tire_trans[1] = Vector3d(-wheel_offset[0], -wheel_offset[1], -wheel_offset[2]); //back left
  tire_trans[2] = Vector3d(wheel_offset[0], wheel_offset[1], -wheel_offset[2]);   //front right
  tire_trans[3] = Vector3d(-wheel_offset[0], wheel_offset[1], -wheel_offset[2]);  //back right
  
  floating_joint = Joint(JointTypeFloatingBase);
  floating_base = Body(base_mass, Vector3d(0,0,0), get_box_inertia(base_mass, base_size));
  base_id = model->AddBody(0, Xtrans(initial_pos), floating_joint, floating_base, "Floating Base");
  
  const char *names[] = {"Front Right", "Back Right","Front Left","Back Left"};
  for(int i = 0; i < 4; i++){
    tire[i] = Body(tire_mass, Vector3d(0,0,0), get_tire_inertia(tire_mass, tire_radius, tire_thickness));
    tire_joints[i] = Joint(JointTypeRevolute, Vector3d(0.,1,0.));
    model->AddBody(base_id, Xtrans(tire_trans[i]), tire_joints[i], tire[i], names[i]);
  }
  
  
  //float ts = .001;
  //solver = new Solver(model, ts, debug);
  load_nn_gc_model_new();

  if(log_file.bad()){
    ROS_INFO("Log FILE BAD BRO 2");
  }
  
  force_scalars[0] = 1;
  force_scalars[1] = 1;
  force_scalars[2] = 1;
  force_scalars[3] = 1;
}

void JackalDynamicSolver::set_terrain_map(const TerrainMap *terrain_map){
    terrain_map_ = terrain_map;
}


JackalDynamicSolver::~JackalDynamicSolver(){
}

void JackalDynamicSolver::del_model(){
  //This is probably not a really smart thing to be doing.
  
  if(model){
    ROS_INFO("Deleteing Model");
    ROS_INFO("Deleteing Model");
    ROS_INFO("Deleteing Model");
    delete model;
    model = 0;
  }
  else{
    return;
  }
  if(debug_level == 2 && log_file.is_open()){
    log_file.close();
  }

  feature_log.close();
}


Eigen::Matrix<float,JackalDynamicSolver::JackalDynamicSolver::num_in_features,1> JackalDynamicSolver::scale_input(Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> features){
    //Do a transform to standard normal.
    Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> scaled_features;
    for(int i = 0; i < JackalDynamicSolver::num_in_features; i++){
        scaled_features[i] = (features[i] - in_mean[i])/in_std[i];
    }
    return scaled_features;
}

Eigen::Matrix<float,JackalDynamicSolver::JackalDynamicSolver::num_out_features,1> JackalDynamicSolver::scale_output(Eigen::Matrix<float,JackalDynamicSolver::num_out_features,1> labels){
    //Do an inverse transform from standard normal.
    Eigen::Matrix<float,JackalDynamicSolver::num_out_features,1> scaled_labels;
    for(int i = 0; i < JackalDynamicSolver::num_out_features; i++){
        scaled_labels[i] = (labels[i]*out_std[i]) + out_mean[i];
    }
    return scaled_labels;
}


/*
 * This function checks for contact with the soil, and gets the maximum tire sinkage.
 * It will have to discretize the tire into a bunch of points and check all of them
 * For intersection with the soil. It's going to have to return the highest sinkage
 * Found. It's not going to consider multiple contacts with the soil. Thats not
 * What the bekker soil model is good at.
 * cpt_X[i] is the spatial transform from the world frame to the contact point frame.
 * tire_X[i] is the spatial transform from the world frame to the center of the tire frame, with a rotation aligned with the contact point frame.
 * This makes sense, because later I apply the transpose of the transform on the tire wrench.
 * This transforms the tire wrench into the world frame. According to the spatial v2 transform cheat sheet.
 */
void JackalDynamicSolver::get_tire_sinkages_and_cpts_simple(float *X, float *tire_sinkages, SpatialTransform *cpt_X, SpatialTransform *tire_X){
    Quaternion quat(X[3], X[4], X[5], X[10]);
    Vector3d r(X[0], X[1], X[2]);

    //quat.toMatrix() is the rotation that transforms from world
    //to vehicle coordinates.
    //vehicle_rot is the rotation that transforms a vector
    //expressed in the vehicle frame to a vector in world frame
    //coordinates.
    Matrix3d vehicle_rot = quat.toMatrix().transpose();
    Matrix3d test_rot;
    
    Vector3d center_of_tire;
    Vector3d radius_vec(0,0,-tire_radius);
    Vector3d cpt; //contact_point
    
    double theta_limit = -M_PI*.5;
    int max_checks = 100;
    float test_sinkage;
    
    for(int i = 0; i < 4; i++){
      //3 is the front right tire.
      //r is vehicle position
      //Transforms the position of the joint frame into the world frame
      center_of_tire = r + (vehicle_rot*model->GetJointFrame(3+i).r);
      
      cpt_X[i].r = center_of_tire + vehicle_rot*radius_vec;
      tire_sinkages[i] = terrain_map_->getAltitude(cpt_X[i].r[0], cpt_X[i].r[1], cpt_X[i].r[2]) - cpt_X[i].r[2];
      //vehicle_rot.transpose() is quat.toMatrix(),
      //the transform from world to vehicle coordinates.
      cpt_X[i].E = vehicle_rot.transpose();
      
      tire_X[i].r = center_of_tire;
      tire_X[i].E = vehicle_rot.transpose();
      
    }

    /*
    feature_log << terrain_map_->getAltitude(cpt_X[0].r[0], cpt_X[0].r[1], cpt_X[0].r[2]) << ',' <<
      terrain_map_->getAltitude(cpt_X[1].r[0], cpt_X[1].r[1], cpt_X[1].r[2]) << ',' <<
      terrain_map_->getAltitude(cpt_X[2].r[0], cpt_X[2].r[1], cpt_X[2].r[2]) << ',' <<
      terrain_map_->getAltitude(cpt_X[3].r[0], cpt_X[3].r[1], cpt_X[3].r[2]) << '\n';
    */
}


void JackalDynamicSolver::get_tire_sinkages_and_cpts(float *X, float *tire_sinkages, SpatialTransform *cpt_X, SpatialTransform *tire_X){
    Quaternion quat(X[3], X[4], X[5], X[10]);
    Vector3d r(X[0], X[1], X[2]);

    //quat.toMatrix() is the rotation that transforms from world
    //to vehicle coordinates.
    //vehicle_rot is the rotation that transforms a vector
    //expressed in the vehicle frame to a vector in world frame
    //coordinates.
    Matrix3d vehicle_rot = quat.toMatrix().transpose();
    Matrix3d test_rot;
    
    Vector3d center_of_tire;
    Vector3d radius_vec(0,0,-tire_radius);
    Vector3d cpt; //contact_point
    
    double theta_limit = -M_PI*.25;
    int max_checks = 100;
    float test_sinkage;
    
    for(int i = 0; i < 4; i++){
      //3 is the front right tire.
      //r is vehicle position
      //Transforms the position of the joint frame into the world frame
      center_of_tire = r + (vehicle_rot*model->GetJointFrame(3+i).r);
      
      tire_sinkages[i] = -1;
      //cpt_X[i].r = Vector3d(0,0,0);
      cpt_X[i].r = center_of_tire + vehicle_rot*radius_vec;
      //vehicle_rot.transpose() is quat.toMatrix(),
      //the transform from world to vehicle coordinates.
      cpt_X[i].E = vehicle_rot.transpose();
      
      tire_X[i].r = center_of_tire;
      tire_X[i].E = vehicle_rot.transpose();
      
      for(int j = 0; j < max_checks; j++){
        test_rot = roty(theta_limit - (2*theta_limit*j/((float)(max_checks - 1))));
        
        Matrix3d temp_rot = vehicle_rot*test_rot;     //Rot vector from cpt frame to world.
        cpt = center_of_tire + (temp_rot*radius_vec); //Translate vector from cpt frame to world
        test_sinkage = terrain_map_->getAltitude(cpt[0], cpt[1], cpt[2]) - cpt[2];
        
        if(test_sinkage > tire_sinkages[i]){
          tire_sinkages[i] = test_sinkage;
          //temp_rot is the orientation of the contact point frame. Transform from cpt frame to world.
          //temp_rot.transpose() will rotate vectors from world to cpt and tire reaction force frame          
          tire_X[i].E = temp_rot.transpose();
          cpt_X[i].E = temp_rot.transpose();
          cpt_X[i].r = cpt;
        }
      }
    }
    
    //ROS_INFO("Best angles %f %f %f %f", best_angle[0], best_angle[1], best_angle[2], best_angle[3]);
    
    // //ROS_INFO(" ");
    // Vector3d z_vec(0,0,1);
    // //rotate normal vector from contact point frame to world for debugging.
    // Vector3d z_rot = cpt_X[0].E.transpose() * z_vec;
    // Vector3d v_rot = vehicle_rot * z_vec;    
    // if(tire_sinkages[0] > 0){
    //   feature_log << cpt_X[0].r[0] << ',' << cpt_X[0].r[1] << ',' << cpt_X[0].r[2] << ',' << z_rot[0] << ',' << z_rot[1] << ',' << z_rot[2] << ',' << v_rot[0] << ',' << v_rot[1] << ',' << v_rot[2] << '\n';
    // }
    
    /*
    feature_log << cpt_X[0].r[0] << ',' << cpt_X[0].r[1] << ',' << cpt_X[0].r[2]
                << ',' << cpt_X[1].r[0] << ',' << cpt_X[1].r[1] << ',' << cpt_X[1].r[2]
                << ',' << cpt_X[2].r[0] << ',' << cpt_X[2].r[1] << ',' << cpt_X[2].r[2]
                << ',' << cpt_X[3].r[0] << ',' << cpt_X[3].r[1] << ',' << cpt_X[3].r[2] << '\n';
    */
    //ROS_INFO("best tire cpt  r <%f %f %f>  E*z <%f %f %f>  VehicleRot*z <%f %f %f>   rotation %f",  cpt_X[0].r[0], cpt_X[0].r[1], cpt_X[0].r[2],    z_rot[0], z_rot[1], z_rot[2],    v_rot[0], v_rot[1], v_rot[2]);
}



//Computes linear velocity at the point cpt_X caused by the velocity of the vehicle.
void JackalDynamicSolver::get_tire_vels(float *X, Vector3d *tire_vels, SpatialTransform *cpt_X){
  Quaternion quat(X[3], X[4], X[5], X[10]);
  Vector3d r(X[0], X[1], X[2]);

  //body 1 is an intermediate body connected to world by 3 translational DOF
  //orientation and translation of body 1 relative to world
  //This transform doesn't actually do anything to a pure linear spatial velocity
  SpatialTransform X_base1(Quaternion(0,0,0,1).toMatrix(), r);
  //body 2 is an intermediate body connected to body 1 by 3 rotational DOF
  //quat.toMatrix() is the rotation that transforms from world to vehicle frame
  //r is the translation from world to vehicle frame.
  //(These are the correct arguments)
  //you can verify with SpatialOperators.h contructor and ::apply()
  SpatialTransform X_base2(quat.toMatrix(), r);
  
  SpatialVector body_vel_lin(0,0,0,X[11],X[12],X[13]);
  SpatialVector body_vel_ang(X[14],X[15],X[16],0,0,0);

  //X_base1 and X_base2 transform from world to body frames.
  //inverse of these transforms from body to world frames.
  SpatialVector sp_vel_world = X_base2.inverse().apply(body_vel_ang) + X_base1.inverse().apply(body_vel_lin);
  
  for(int i = 0; i < 4; i++){
    //cpt_X is transform from world to cpt frame.
    SpatialVector sp_vel_cp = cpt_X[i].apply(sp_vel_world);
    Vector3d lin_vel(sp_vel_cp[3], sp_vel_cp[4], sp_vel_cp[5]);
    tire_vels[i] = lin_vel;
  }

  //Vector3d cpt_lin_world = quat.toMatrix().transpose()*tire_vels[0]; //tire_vel but rotated into the world frame so plotting makes sense.
  //feature_log << r[0] << ',' << r[1] << ',' << X[11] << ',' << X[12] << ',' << cpt_lin_world[0] << ',' << cpt_lin_world[1] << '\n';
}



SpatialVector JackalDynamicSolver::tire_model_bekker(const Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> &features){
  SpatialVector tire_wrench;
  float Fx = 0;
  float Fy = 0;
  float Fz = 0;
  float Ty = 0;

  BekkerModel bekker_model;
  bekker_model.R = tire_radius;
  bekker_model.b = tire_thickness;
  
  bekker_model.zr = std::min(features[0], tire_radius);
  bekker_model.slip_ratio = features[1];
  bekker_model.slip_angle = features[2];
  
  bekker_model.kc = features[3];
  bekker_model.kphi = features[4];
  bekker_model.n0 = features[5];
  bekker_model.n1 = features[6];
  bekker_model.phi = features[7];
  
  bekker_model.n = bekker_model.n0 + (bekker_model.n1 * fabs(bekker_model.slip_ratio));
  bekker_model.pgc = ((bekker_model.kc/bekker_model.b) + bekker_model.kphi) * std::pow(bekker_model.zr, bekker_model.n);
  
  bekker_model.ze;
  bekker_model.zu;
  bekker_model.ku;
  
  
  if(bekker_model.pgc < bekker_model.pg){
    bekker_model.ze = bekker_model.zr;
  }
  else{
    bekker_model.ze = std::pow(bekker_model.pg/((bekker_model.kc/bekker_model.b) + bekker_model.kphi), 1.0f/bekker_model.n);
  }
  
  bekker_model.ku = bekker_model.k0 + (bekker_model.Au*bekker_model.ze);
  bekker_model.zu = std::pow(((bekker_model.kc/bekker_model.b) + bekker_model.kphi) / bekker_model.ku, 1.0f/bekker_model.n) * bekker_model.ze;
  
  float theta_f = bekker_model.theta_f =  acosf(1 - (bekker_model.zr/bekker_model.R));
  float theta_r = bekker_model.theta_r = -acosf(1 - ((bekker_model.zu + bekker_model.zr - bekker_model.ze)/bekker_model.R));
  float theta_c = bekker_model.theta_c =  acosf(1 - ((bekker_model.zr - bekker_model.ze)/bekker_model.R));

  //seems legit
  //ROS_INFO("f %f     r %f     c %f", theta_f*180.0f/M_PI, theta_r*180.0f/M_PI, theta_c*180.0f/M_PI);
  
  float bt_R = bekker_model.R * bekker_model.b;
  float bt_RR = bekker_model.R * bekker_model.R * bekker_model.b;
  
  Fx = bt_R*bekker_model.integrate(&bekker_model.Fx_eqn1, theta_f, theta_c) +
       bt_R*cosf(theta_c)*bekker_model.integrate(&bekker_model.Fx_eqn3, theta_c, -theta_c) +
       bt_R*bekker_model.integrate(&bekker_model.Fx_eqn2, -theta_c, theta_r);
  
  Fy = bt_R*bekker_model.integrate(&bekker_model.tau_y_cf, theta_f, theta_c) +
       bt_R*cosf(theta_c)*bekker_model.integrate(&bekker_model.Fy_eqn3, theta_c, -theta_c) + //original matlab code has a bug here. In the matlab code I integrated from -theta_c to theta_c which gives a negative result.
       bt_R*bekker_model.integrate(&bekker_model.tau_y_cr, -theta_c, theta_r);
  
  Fz = bt_R*bekker_model.integrate(&bekker_model.Fz_eqn1, theta_f, theta_c) +
       bt_R*bekker_model.integrate(&bekker_model.Fz_eqn2, -theta_c, theta_r) +
     2*bt_R*sinf(theta_c)*bekker_model.pg;
  
  Ty = -bt_RR*bekker_model.integrate(&bekker_model.tau_x_cf, theta_f, theta_c) + 
       -bt_RR*bekker_model.integrate(&bekker_model.tau_x_cr, -theta_c, theta_r) +
       -bt_RR*cosf(theta_c)*cosf(theta_c)*bekker_model.integrate(&bekker_model.Ty_eqn1, theta_c, -theta_c);
  
  tire_wrench = 1000*SpatialVector(0,Ty,0,Fx,Fy,Fz);
  
  return tire_wrench;
}




//return body wrench from features
SpatialVector JackalDynamicSolver::tire_model_nn(const Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> &features){
  Eigen::Matrix<float,JackalDynamicSolver::num_hidden_nodes,1> layer0_out;
  Eigen::Matrix<float,JackalDynamicSolver::num_hidden_nodes,1> layer2_out;
  Eigen::Matrix<float,JackalDynamicSolver::num_out_features,1> layer4_out;
  Eigen::Matrix<float,JackalDynamicSolver::num_out_features,1> labels;
  Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> scaled_features;
  
  scaled_features = scale_input(features);
  layer0_out = (weight0*scaled_features) + bias0;
  layer0_out = layer0_out.unaryExpr(&tanhf);
  layer2_out = (weight2*layer0_out) + bias2;
  layer2_out = layer2_out.unaryExpr(&tanhf);
  layer4_out = (weight4*layer2_out) + bias4;        
  labels = scale_output(layer4_out);
  
  SpatialVector tire_wrench = SpatialVector(0,labels[3],0,labels[0],labels[1],labels[2]);
  
  return tire_wrench;
}

void JackalDynamicSolver::get_tire_f_ext(float *X){
    SpatialVector tire_wrench;
    
    Quaternion quat(X[3], X[4], X[5], X[10]);
    Vector3d r(X[0], X[1], X[2]);
    
    Eigen::Matrix<float,JackalDynamicSolver::num_in_features,1> features;

    SpatialTransform tire_X[4];
    SpatialTransform cpt_X[4];
    float sinkages[4];
    get_tire_sinkages_and_cpts(X, sinkages, cpt_X, tire_X);
    
    Vector3d tire_vels[4];
    get_tire_vels(X, tire_vels, cpt_X);
    
    BekkerData ts_data;
    
    for(int i = 0; i < 4; i++){    
        features[0] = sinkages[i];
        
        if(sinkages[i] <= 0){ //Tire is not in contact with the ground.
            //tire_X is the transform from world to tire.
            //the following 0-wrench is in the tire frame.
            f_ext[3+i] = tire_X[i].applyTranspose(SpatialVector(0,0,0,0,0,0));
            continue;
        }
        
        if(X[17+i] == 0){
            if(tire_vels[i][0] == 0){
              features(1) = 0; //otherwise would be 1 - 0/0 = 1. Which would be wrong.
            }
            else{
              features(1) = 1 - (tire_vels[i][0]/1.0e-3f);
            }
        }
        else{
            if(tire_vels[i][0] == 0){
              features(1) = 1 - (1.0e-3f/(tire_radius*X[17+i]));
            }
            else{
              features(1) = 1 - (tire_vels[i][0]/(tire_radius*X[17+i]));
            }
        }
        
        features(1) = std::min(1.0f, std::max(-1.0f, features(1)));
        
        if(tire_vels[i][0] == 0){ //prevent dividing by zero
          features(2) = atanf(tire_vels[i][1]/1.0e-3f); //this returns +- PI/2 depending on sign of tire_vels[i][1]
        }
        else{
          features(2) = atanf(tire_vels[i][1]/fabs(tire_vels[i][0]));
        }
       
        //features(2) = atan2f(tire_vels[i][1], fabs(tire_vels[i][0]));
        
        ts_data = terrain_map_->getSoilDataAt(cpt_X[i].r[0], cpt_X[i].r[1]);
        
        features(3) = ts_data.kc;
        features(4) = ts_data.kphi;
        features(5) = ts_data.n0;
        features(6) = ts_data.n1;
        features(7) = ts_data.phi;
        
        tire_wrench = tire_model_nn(features);
        
        //Sign corrections.
        if(X[17+i] > 0){
          tire_wrench[3] = tire_wrench[3]*1;
          tire_wrench[1] = tire_wrench[1]*1;
        }
        else{
          tire_wrench[3] = tire_wrench[3]*-1;
          tire_wrench[1] = tire_wrench[1]*-1;
        }

        if(tire_vels[i][1] > 0){          
          tire_wrench[4] = -fabs(tire_wrench[4]);
        }
        else{
          tire_wrench[4] = fabs(tire_wrench[4]);
        }
        
        if(tire_vels[i][2] > 0){ //prevent bouncing. If tire is moving upwards, no Fz. 
          tire_wrench[5] *= .1;//fmin(tire_wrench[5], 10); //add code to artificially increase other forces when high sinkage is detected
        }
        if(tire_wrench[5] < 0){ //Fz should never point down. But the neural net might not know that.
          tire_wrench[5] = 0;
        }

        
        tire_wrench[1] = force_scalars[0]*tire_wrench[1];
        tire_wrench[3] = force_scalars[1]*tire_wrench[3];
        tire_wrench[4] = force_scalars[2]*tire_wrench[4];
        tire_wrench[5] = force_scalars[3]*tire_wrench[5];
        
        
        //tire_X is the transform from world to tire.
        //the following 0-wrench is in the tire frame.
        f_ext[3+i] = tire_X[i].applyTranspose(tire_wrench);
    }
}

//This doesn't actually do anything because the PID controllers are actually PD controllers
//and don't have anything to reset.
void JackalDynamicSolver::reset(){
  internal_controller[0].reset();
  internal_controller[1].reset();
}

float JackalDynamicSolver::get_timestep(){
  return timestep*stepsize;
}

//this equation has no effect on dynamics
void JackalDynamicSolver::log_xout(float *Xout){
  //Log a single row from Xout;
  /*
//qw,qx,qy,qz,x,y,z,wx,wy,wz,vx,vy,vz,q1,q2,q3,q4,qd1,qd2,qd3,qd4
    x_init = [1 0 0 0  0 0 zoff   0 0 0  0 0 0]';
              |______| |_______| |_____| |____|
               |        |        |       |->Linear Velocity in F_1
               |        |        |          Coordinates
               |        |        |
               |        |        |->Angular Velocity in F_1 coordinates
               |        |        
               |        |->Position relative to F_0
               |
               |->Orientation Quaternion

q_init = [0 .4 .8 1.2];
qd_init = [0 0 0 0];
*/

  
  float temp[21];
  temp[0] = Xout[10];
  temp[1] = Xout[3];
  temp[2] = Xout[4];
  temp[3] = Xout[5];
  temp[4] = Xout[0];
  temp[5] = Xout[1];
  temp[6] = Xout[2];
  
  Quaternion quat(Xout[3], Xout[4], Xout[5], Xout[10]);
  Vector3d r(Xout[0], Xout[1], Xout[2]);
  
  SpatialTransform X_base1(Quaternion(0,0,0,1).toMatrix(), r);
  SpatialTransform X_base2(quat.toMatrix(), r);
  
  SpatialVector body_vel_lin(0,0,0,Xout[11],Xout[12],Xout[13]);
  SpatialVector body_vel_ang(Xout[14],Xout[15],Xout[16],0,0,0);
  
  SpatialVector base_vel = X_base2.inverse().apply(body_vel_ang) + X_base1.inverse().apply(body_vel_lin);
  
  temp[7] = base_vel[0]; //Wx
  temp[8] = base_vel[1]; //Wy
  temp[9] = base_vel[2]; //Wz
  temp[10] = base_vel[3]; //Vx
  temp[11] = base_vel[4]; //Vy
  temp[12] = base_vel[5]; //Vz
    

  /*
  temp[17] = Xout[6]; //q1
  temp[18] = Xout[7];
  temp[19] = Xout[8];
  temp[20] = Xout[9];
  
  temp[13] = Xout[17]; //qd1
  temp[14] = Xout[18];
  temp[15] = Xout[19];
  temp[16] = Xout[20];
  */
  
  temp[13] = Xout[6]; //q1
  temp[14] = Xout[7];
  temp[15] = Xout[8];
  temp[16] = Xout[9];

  temp[17] = Xout[17]; //qd1
  temp[18] = Xout[18];
  temp[19] = Xout[19];
  temp[20] = Xout[20];

  if(log_file.bad()){
    ROS_INFO("Log FILE BAD BRO");
  }
  
  for(int i = 0; i < 20; i++){
      log_file << temp[i] << ',';
  }
  log_file << temp[20] << "\n";
}



void JackalDynamicSolver::log_features(float *Xout, float *Xout_next, float *Xd, float vl, float vr){
  //if(timestep%10 == 0){
  for(int i = 0; i < 3; i++){
      feature_log << Xout[i] << ','; //position
  }

  double roll, pitch, yaw;
  tf::Quaternion quat(Xout[3],Xout[4],Xout[5],Xout[10]);
  tf::Matrix3x3 rot(quat);
  rot.getRPY(roll,pitch,yaw);

  feature_log << roll << ',' << pitch << ',';


  Quaternion rbdl_quat(Xout[3],Xout[4],Xout[5],Xout[10]);
  Vector3d body_lin_vel = rbdl_quat.toMatrix() * Vector3d(Xout[11],Xout[12],Xout[13]);
  feature_log << body_lin_vel[0] << ',' << body_lin_vel[1] << ',' << Xout[16] << ',';
  
  for(int i = 17; i < 21; i++){
    feature_log << Xout[i] << ',';
  }
  
  feature_log << vl << ',';
  feature_log << vr << ',';
  
  Quaternion rbdl_quat2(Xout_next[3],Xout_next[4],Xout_next[5],Xout_next[10]);
  body_lin_vel = rbdl_quat2.toMatrix() * Vector3d(Xout_next[11],Xout_next[12],Xout_next[13]);
  feature_log << body_lin_vel[0] << ',' << body_lin_vel[1] << ',' << Xout[16] << ',';

  feature_log << Xd[11] << ',';
  feature_log << Xd[12] << ',';
  feature_log << Xd[13] << ',';
  feature_log << Xd[14] << ',';
  feature_log << Xd[15] << ',';
  feature_log << Xd[16] << '\n';
}


void JackalDynamicSolver::solve(float *x_init, float *x_end, float vl, float vr, float sim_time){
  int sim_steps = floorf(sim_time/stepsize);
  float Xout[21];
  float Xout_next[21];
  
  reset();
  
  // 0 1 2   3  4  5    6  7  8  9    10   11 12 13   14 15 16   17  18  19  20
  // x,y,z,  qx,qy,qz,  q1,q2,q3,q4,  qw,  vx,vy,vz,  ax,ay,az,  qd1,qd2,qd3,qd4
  for(int i = 0; i < 21; i++){
    Xout[i] = x_init[i];
  }

  unsigned max_steps = sim_steps + timestep;
  for(; timestep < max_steps; timestep++){
    step(Xout, Xout_next, vl, vr);
    
    if(debug_level == 2){
      log_xout(Xout);
      //log_features(Xout, 0);

    }
    
    for(int i = 0; i < 21; i++){
      Xout[i] = Xout_next[i];
    } 
  }
  
  for(int i = 0; i < 21; i++){
    x_end[i] = Xout[i];
  }
}

void JackalDynamicSolver::step(float *X_now, float *X_next, float vl, float vr){
  float right_err = vr - (X_now[17] + X_now[18])/2;
  float left_err = vl - (X_now[19] + X_now[20])/2;
  
  //X_now[17] = X_now[18] = vr;
  //X_now[19] = X_now[20] = vl;
  
  vel_left_ = vl;
  vel_right_ = vr;
  
  tau[6] = tau[7] = std::min(std::max(internal_controller[0].step(right_err), -20.0f), 20.0f);
  tau[8] = tau[9] = std::min(std::max(internal_controller[1].step(left_err), -20.0f), 20.0f);
  
  //ROS_INFO("%f   %f   %f   %f", tau[6],tau[7],tau[8],tau[9]);
  
  //euler_method_unit(X_now, X_next);
  runge_kutta_method(X_now, X_next);
}




//solve with constant tire rotational speed
void JackalDynamicSolver::solve(float *x_init, float *x_end, float sim_time){
  int sim_steps = floorf(sim_time/stepsize);
  float Xout[21];
  float Xout_next[21];
  
  reset();
  
  for(int i = 0; i < 21; i++){
    Xout[i] = x_init[i];
  }
  
  unsigned max_steps = sim_steps + timestep;
  for(; timestep < max_steps; timestep++){
    if(debug_level == 2){
      log_xout(Xout);
      //log_features(Xout, 0);
    }
        
    runge_kutta_method(Xout, Xout_next);
    //euler_method_unit(Xout, Xout_next);
        
    for(int i = 0; i < 17; i++){ //dont assign tire velocities, hold them constant
      Xout[i] = Xout_next[i];
    } 
  }
  
  for(int i = 0; i < 21; i++){
    x_end[i] = Xout[i];
  }
}


void JackalDynamicSolver::apply_force(SpatialVector wrench, int body){
  f_ext[body] = wrench;
}

//kinematic
//     0, 1, 2,     3,4,5,  6,7,8,9,  10,11, 12,13, 14,15,16, 17,  18,  19,  20
//X = [x, y, theta, -,-,-,  -,-,-,-,  -, vx, vy,-,  -, -, Wz, qd1, qd2, qd3, qd4  ]
void JackalDynamicSolver::ode_kinematic(float *X, float *Xd){
  for(int i = 0; i < model->q_size; i++){
    Xd[i] = 0;
  }
  for(int i = 0; i < model->qdot_size; i++){
    Xd[i+model->q_size] = 0;
  }

  
  float vr = tire_radius*(X[17]+X[18])*.5;
  float vl = tire_radius*(X[19]+X[20])*.5;
  float vf = (vl+vr)*.5;
  
  Xd[0] = vf*cosf(X[2]);
  Xd[1] = vf*sinf(X[2]);
  Xd[2] = (vr-vl)/.323;
  
  
}


void JackalDynamicSolver::ode_stabilize(float *X, float *Xd){
  VectorNd Q = VectorNd::Zero(model->q_size);
  VectorNd QDot = VectorNd::Zero(model->qdot_size);
  VectorNd QDDot = VectorNd::Zero(model->qdot_size);

  SpatialTransform temp;
  SpatialVector tire_wrench;
  
  for(int i = 0; i < model->q_size; i++){
    Q[i] = X[i];
  }
  for(int i = 0; i < model->qdot_size; i++){
    QDot[i] = X[i+model->q_size];
  }
  
  get_tire_f_ext(X);
  
  ForwardDynamics(*model, Q, QDot, tau, QDDot, &f_ext);
  
  Quaternion quat(Q[3],Q[4],Q[5],Q[10]);
  Vector3d omega_b(QDot[3], QDot[4], QDot[5]);
  //quat.toMatrix().transpose() is the rotation from vehicle to world
  Vector3d omega_w = quat.toMatrix().transpose()*omega_b;
  omega_w[2] = 0; //Do not yaw in world frame.
  omega_b = quat.toMatrix()*omega_w;
  
  QDot[3] = omega_b[0];
  QDot[4] = omega_b[1];
  QDot[5] = omega_b[2];
  
  Eigen::Vector4d qnd = quat.omegaToQDot(omega_b); //get quaternion derivative.
  
  for(int i = 0; i < model->qdot_size; i++){
    Xd[i] = QDot[i];
  }
  
  Xd[3] = qnd[0];
  Xd[4] = qnd[1];
  Xd[5] = qnd[2];
  Xd[10] = qnd[3];
  
  for(int i = 0; i < model->qdot_size; i++){
    Xd[i+model->q_size] = QDDot[i];
  }  
}



void JackalDynamicSolver::ode(float *X, float *Xd){
  VectorNd Q = VectorNd::Zero(model->q_size);
  VectorNd QDot = VectorNd::Zero(model->qdot_size);
  VectorNd QDDot = VectorNd::Zero(model->qdot_size);
  
  SpatialTransform temp;
  SpatialVector tire_wrench;
  
  for(int i = 0; i < model->q_size; i++){
    Q[i] = X[i];
  }
  for(int i = 0; i < model->qdot_size; i++){
    QDot[i] = X[i+model->q_size];
  }
  
  
  get_tire_f_ext(X);
  
  ForwardDynamics(*model, Q, QDot, tau, QDDot, &f_ext);
  
  //2D simplification. Ignore Fz. Assume constant sinkage.
  //QDDot[2] = 0; // Corresponds to Z-acceleration
  //QDDot[3] = 0; // roll
  //QDDot[4] = 0; // pitch
  
  Quaternion quat(Q[3],Q[4],Q[5],Q[10]);
  Vector3d omega(QDot[3], QDot[4], QDot[5]);
  Eigen::Vector4d qnd = quat.omegaToQDot(omega); //get quaternion derivative.
  
  //check main.cpp for the order of X and Xd
  //Its because ForwardDynamics gives me rotational acceleration, but not quaternion derivative
  for(int i = 0; i < model->qdot_size; i++){
    Xd[i] = QDot[i];
  }
  
  Xd[3] = qnd[0]; //quaternion derivative.
  Xd[4] = qnd[1];
  Xd[5] = qnd[2];
  Xd[10] = qnd[3]; //qw is at the end for literally no reason. thanks rbdl.
  
  for(int i = 0; i < model->qdot_size; i++){
    Xd[i+model->q_size] = QDDot[i];
  }
  
  Xd[17] = 0;
  Xd[18] = 0;
  Xd[19] = 0;
  Xd[20] = 0;
}

//achieve a vehicle orientation that is steady state
//only allow vehicle to translate up and down, roll and pitch
//DO not yaw or move in x and y
void JackalDynamicSolver::stabilize_sinkage(float *X, float *Xt1){
  int len = model->q_size + model->qdot_size;
  float ts = stepsize*.1;
  float vel_mag;
  float Xn1[21];
  float Xn[21];
  float Xd[21];
  
  for(int i = 0; i < len; i++){
    Xn[i] = X[i];
  }
  
  //start vehicle above ground.
  Xn[2] = .16 + terrain_map_->getAltitude(X[0], X[1], X[2]);
  
  //do{
  for(int ii = 0; ii < 10000; ii++){
    log_xout(Xn);
    ode_stabilize(Xn, Xd);
    
    for(int i = 0; i < len; i++){
      Xn1[i] = Xn[i] + Xd[i]*ts; 
    }
    
    //ROS_INFO("Quat: %f %f %f %f", Xd[3], Xd[4], Xd[5], Xd[10]);
    //ROS_INFO("Xd ang acc: %f %f %f", Xd[14], Xd[15], Xd[16]);
    //ROS_INFO("Xn %f %f %f      Xn1 %f %f %f", Xn[0], Xn[1], Xn[2],   Xn1[14], Xn1[15], Xn1[16]);
    
    float temp[4] = {Xn1[3], Xn1[4], Xn1[5], Xn1[10]};
    normalize_quat(temp); //this is needed because the quaternion is updated under a small angle assumption using what is essentially a hack for integratin quaternions and it doesnt preserve the quaternion unit norm property
    
    Xn1[3] = temp[0];
    Xn1[4] = temp[1];
    Xn1[5] = temp[2];
    Xn1[10] = temp[3];
    
    Xn[2] = Xn1[2]; //z
    
    //vel_mag = sqrtf((Xn1[13]*Xn1[13]) + (Xn1[14]*Xn1[14]) + (Xn1[15]*Xn1[15]));

    Xn[11] = Xn1[11]; //Vx
    Xn[12] = Xn1[12]; //Vy
    Xn[13] = Xn1[13]; //Vz
    Xn[14] = Xn1[14]; //Wx
    Xn[15] = Xn1[15]; //Wy
    
    Xn[3] = Xn1[3];
    Xn[4] = Xn1[4];
    Xn[5] = Xn1[5];
    Xn[10] = Xn1[10];
    
    for(int i = 17; i < 21; i++){
      Xn[i] = Xn1[i];
      //Xn[i-11] = Xn1[i-11];
    }
  } //while(vel_mag > .01);

  for(int i = 0; i < 21; i++){
    Xt1[i] = 0;
  }
  
  Xt1[0] = Xn[0];
  Xt1[1] = Xn[1];
  Xt1[2] = Xn[2];

  Xt1[3] = Xn[3];
  Xt1[4] = Xn[4];
  Xt1[5] = Xn[5];
  Xt1[10] = Xn[10];
  
}


void JackalDynamicSolver::euler_method(float *X, float *Xt1){
  int len = model->q_size + model->qdot_size;
  float ts = stepsize;
  float Xd[len];
  
  ode(X, Xd);
  
  for(int i = 0; i < len; i++){
    Xt1[i] = X[i] + Xd[i]*ts; 
  }
  
  float temp[4] = {Xt1[3], Xt1[4], Xt1[5], Xt1[10]};
  //this is needed because the quaternion is updated under a small angle assumption using what is essentially a hack for integratin quaternions and it doesnt preserve the quaternion unit norm property
  normalize_quat(temp); 
  Xt1[3] = temp[0];
  Xt1[4] = temp[1];
  Xt1[5] = temp[2];
  Xt1[10]= temp[3];
}

//maintain unit quaternion with smarter integration
void JackalDynamicSolver::euler_method_unit(float *X, float *Xt1){
  int len = model->q_size + model->qdot_size;
  float ts = stepsize;
  float Xd[len];
  
  ode(X, Xd);
  
  for(int i = 0; i < len; i++){
    Xt1[i] = X[i] + Xd[i]*ts; 
  }

  //prevent divide by zero.
  if(X[14] == 0 && X[15] == 0 && X[16] == 0){
    Xt1[3] = X[3];
    Xt1[4] = X[4];
    Xt1[5] = X[5];
    Xt1[10] = X[10];
    return;
  }
  
  Quaternion quat(X[3], X[4], X[5], X[10]);
  Vector3d omega_b(X[14], X[15], X[16]);
  Vector3d omega_w = quat.toMatrix().transpose()*omega_b;
  Quaternion quat_n = quat.timeStep(omega_w, ts);
  
  Xt1[3] = quat_n[0];
  Xt1[4] = quat_n[1];
  Xt1[5] = quat_n[2];
  Xt1[10] = quat_n[3];
}


void JackalDynamicSolver::runge_kutta_method(float *X, float *Xt1){
  int len = model->q_size + model->qdot_size;
  float ts = stepsize;
  
  //make sure quaternion is normalized after every step or error will be terrible.
  auto normalize_helper = [](float *X_temp){
                            float l2_norm = sqrtf((X_temp[3]*X_temp[3]) + (X_temp[4]*X_temp[4]) + (X_temp[5]*X_temp[5]) + (X_temp[10]*X_temp[10]));
                            X_temp[3] /= l2_norm;
                            X_temp[4] /= l2_norm;
                            X_temp[5] /= l2_norm;
                            X_temp[10] /= l2_norm;
                          };
  
  float temp[len];
  float k1[len];
  ode(X, k1); //Calculate derivative.
  for(int i = 0; i < len; i++) temp[i] = X[i]+.5*ts*k1[i];
  normalize_helper(temp);
  
  float k2[len];
  ode(temp, k2);
  for(int i = 0; i < len; i++) temp[i] = X[i]+.5*ts*k2[i];
  normalize_helper(temp);
  
  float k3[len];
  ode(temp, k3);
  for(int i = 0; i < len; i++) temp[i] = X[i]+ts*k3[i];
  normalize_helper(temp);
  
  float k4[len];
  ode(temp, k4);
  for(int i = 0; i < len; i++){
    Xt1[i] = X[i] + (ts/6)*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
  }
  normalize_helper(Xt1);
}

void JackalDynamicSolver::normalize_quat(float *q){
  float norm = sqrtf((q[0]*q[0]) + (q[1]*q[1]) + (q[2]*q[2]) + (q[3]*q[3]));
  for(int i = 0; i < 4; i++){
    q[i] /= norm;
  }
}




//#include "solver.h"
//Auto Generated by pretrain.py

void JackalDynamicSolver::load_nn_gc_model_new(){
    weight0 << -7.0340e-02, -1.3777e+00,  9.2441e-03, -1.7163e-03,  2.0788e-02,
        -6.5836e-02, -5.4273e-02, -2.2705e-02,  4.6693e-02, -2.0944e-01,
        -7.4550e-01, -1.9709e-02, -3.8534e-02,  1.8317e-01, -3.0310e-02,
        1.8648e-01,  2.7617e-01,  2.9756e-01,  2.7833e-01,  2.4891e-02,
        5.6775e-02, -5.1937e-01, -1.0698e-01, -4.4940e-02, -1.4342e-01,
        -7.1330e-01,  6.5361e-01,  3.8525e-02,  7.3327e-02, -3.5951e-01,
        3.1534e-02, -1.1789e-01, -1.8221e+00, -3.9410e-02,  2.3617e-02,
        1.5971e-02,  3.2743e-02, -9.4661e-02, -2.6995e-03,  1.1001e-01,
        -4.3236e-01,  7.7764e-02, -1.0650e-01, -8.3629e-03, -1.1048e-02,
        -3.1443e-02,  1.4808e-02,  1.1603e-01, -2.9052e-01,  6.6207e-01,
        -2.4430e-01, -6.1835e-02, -1.0357e-01,  3.3118e-01,  5.2785e-02,
        1.4259e-01,  7.5585e-02,  6.1426e-01, -1.6899e-02, -6.5739e-02,
        -1.3602e-01,  1.8971e-01, -2.0040e-01,  2.7124e-02,  2.4390e-01,
        7.2640e-01,  2.6235e-01, -9.3236e-02, -1.6883e-01,  7.4714e-01,
        4.7909e-02, -3.8195e-02, -1.5118e-01, -9.0689e-01,  1.0439e+00,
        -2.0902e-02, -4.2463e-02,  1.8473e-01, -1.3392e-02, -1.0732e-01,
        -1.8233e-01,  2.0588e-01, -1.0471e-01, -2.4849e-02, -6.4332e-02,
        2.2000e-01, -4.6985e-02, -3.1092e-01,  2.1942e-02, -3.8950e-02,
        8.8361e-01,  2.7245e-02,  5.6515e-02, -1.4182e-01,  2.2345e-03,
        -1.9046e-01, -5.1976e-02, -6.9437e-01, -3.3204e-02, -4.7367e-02,
        -8.9564e-02,  1.5075e-01, -4.4362e-01,  2.1843e-02,  3.6864e-01,
        -2.5539e-01,  1.0064e+00,  1.0120e-02,  2.9172e-02, -6.9503e-02,
        4.2828e-03,  6.1831e-02, -9.6779e-01,  2.3174e-01,  1.3941e-02,
        -3.2607e-01, -6.1971e-01,  1.5685e+00, -1.5550e-02,  7.7931e-03,
        3.8460e-01, -1.9626e-01,  7.4334e-01,  2.9892e-02,  5.7317e-02,
        -1.5233e-01, -2.1422e-03,  1.5339e-01,  2.8305e-01, -3.2595e-01,
        -1.1001e-01,  2.2723e-01,  4.2590e-01, -1.1131e+00,  7.2330e-03,
        4.9166e-02, -9.2643e-02,  3.9087e-01,  2.2222e-02,  5.1695e-02,
        9.6915e-02,  2.1895e-01, -4.3362e-03,  5.2372e-02, -7.5266e-02,
        -7.9956e-02,  4.5829e-01, -2.2301e-02, -4.8310e-02,  3.2761e-01,
        5.1552e-04,  2.3895e-01,  2.4211e-01,  7.4231e-01, -1.8944e+00,
        -7.3728e-03, -2.7242e-03,  8.6473e-03,  3.3436e-02,  1.7271e-01,
        -8.7093e-02, -7.3527e-01, -2.0603e+00, -8.8246e-03, -7.4441e-04,
        8.0686e-02,  9.1644e-03, -1.1765e-01, -2.9140e-01, -3.7120e-01,
        1.1184e-01, -9.5647e-02, -1.5332e-01,  5.8286e-01,  3.4750e-01,
        -3.1053e-02,  1.1354e+00, -6.7248e-02, -2.0055e-02, -5.0945e-02,
        -8.8254e-02, -8.1927e-01,  1.6615e-02, -2.5247e-02,  3.3555e-01,
        1.8358e-01, -8.5899e-02,  1.9126e-02,  3.4695e-02, -9.9606e-02,
        -7.3102e-03,  1.8530e-01, -3.6791e-01, -3.6774e-02, -5.0938e-02,
        5.3178e-02,  9.8901e-02, -4.5716e-01,  1.4086e-02, -9.5289e-02,
        5.6299e-02,  5.7048e-01, -2.3632e-02,  4.7377e-02,  9.4888e-02,
        -1.7535e-01, -1.3386e-01,  3.8088e-02,  4.6375e-01, -2.1648e-01,
        -4.7512e-01,  9.9960e-02,  1.8881e-01, -4.7040e-01, -1.3445e-02,
        6.0112e-02,  8.2626e-02,  1.5673e-01,  2.9168e-02, -5.5691e-01,
        -1.0639e+00,  4.8495e-01, -1.6344e-02,  2.3664e-02, -2.8039e-01,
        -2.0418e-01,  7.2995e-01, -4.1323e-03, -1.1207e-02,  6.5584e-02,
        1.0729e-02,  2.9808e-01, -8.0489e-02,  7.8434e-02, -6.1693e+00,
        -7.7285e-03,  1.6149e-03,  3.5902e-02, -3.8272e-03,  1.5479e-02,
        6.4393e-01,  3.2122e-01,  8.0331e-02,  3.6701e-02,  6.2860e-02,
        -2.8004e-01, -1.1950e-02,  1.8044e-02, -7.0171e-02, -1.1324e+00,
        -1.2108e+00,  7.9270e-03,  1.5751e-02, -1.1707e-01,  2.4348e-02,
        -8.0677e-02, -2.4695e-01,  4.3320e-01,  9.8883e-03, -9.7938e-02,
        -1.8934e-01,  7.0550e-01, -1.4812e-01, -4.8580e-02,  1.2918e-01,
        -1.1589e+00, -1.8172e+00,  1.8160e-02,  4.6420e-02, -1.3362e-01,
        1.2045e-02,  8.6222e-02, -9.6760e-02, -1.6702e+00,  2.1706e+00,
        2.6485e-02,  5.2936e-02, -1.5183e-01,  3.3976e-04, -1.9722e-03;
    bias0 <<  2.4444,  0.3685, -0.7155,  0.8065, -3.2940,  0.4666, -0.5354, -2.0618,
        0.0760,  0.7083,  2.2906,  1.7222, -1.2814, -0.9732, -1.3169, -0.0287,
        0.5439, -0.1801,  0.6052,  0.4868, -1.8635,  0.6370,  1.6670, -0.7754,
        -0.6588,  1.9269,  0.7599, -2.4936, -0.8000, -0.0328,  0.3675,  0.6599,
        0.6075, -0.7946, -0.2237;
    weight2 <<  1.6449e-01,  2.2080e-01,  2.3684e-01, -7.9852e-02, -9.1551e-01,
        1.7968e-02,  2.9793e-01,  9.5824e-01, -1.6311e-01, -5.5738e-02,
        -6.7849e-01, -5.4230e-01, -2.9498e-01,  1.2400e-01, -3.1552e-01,
        3.8692e-01,  2.2962e-01, -5.7225e-02, -4.4849e-01, -1.6527e-02,
        -1.8463e-01, -1.3284e-01,  1.6879e-01, -4.4415e-02, -8.5719e-01,
        -2.6941e-01, -7.0800e-02, -4.4699e-01,  2.5367e-01,  2.1903e-01,
        2.4571e-01,  8.2921e-01, -3.2071e-01,  1.7651e-01,  5.1862e-01,
        2.2124e-01,  3.5687e-01,  3.0960e-01, -1.0440e-01, -7.5322e-01,
        9.1099e-02,  1.1793e-01,  4.7136e-01,  1.6895e-01,  1.5801e-01,
        -7.6572e-01,  4.4740e-02, -3.0793e-01,  4.7122e-01, -3.7234e-01,
        -1.3919e-02,  2.0802e-01, -4.5322e-01,  1.2043e-01, -4.7836e-01,
        -5.8348e-02, -2.0468e-01,  3.9237e-01,  4.0865e-01, -1.0531e-01,
        -1.9608e-01,  5.0867e-01, -5.7118e-01, -6.7870e-02, -1.8949e-01,
        -7.1137e-02, -5.8757e-02, -6.2262e-01, -4.0648e-01, -6.8729e-02,
        -3.9391e-02, -1.3616e-01,  1.4992e-01, -7.3486e-03, -1.3498e+00,
        -1.3930e-01, -1.5345e-01,  3.8021e-01,  7.3478e-02, -1.2824e-01,
        -8.4359e-01,  2.4505e-01, -1.4045e-01,  1.8645e-01, -2.8025e-01,
        4.0750e-02,  3.4534e-01, -2.9969e-01, -2.6898e-01,  9.1850e-02,
        -2.3609e-02, -5.3971e-02, -1.0978e-01,  3.1918e-01, -6.9547e-01,
        -4.3215e-01,  6.0281e-03, -5.4044e-01, -2.4049e-01, -4.9218e-02,
        2.7119e-01,  7.1576e-02, -1.6770e-01,  9.0916e-02,  7.3597e-02,
        -1.0149e-01, -1.6428e-01,  2.1386e-01, -2.0629e-01,  7.8741e-01,
        -5.1052e-01, -9.3051e-01, -4.7320e-01, -2.3886e-01, -2.4233e-01,
        9.7357e-01,  2.4369e-02,  7.5543e-03, -5.7427e-02,  2.5665e-01,
        -7.5815e-01,  2.3018e-01, -4.8435e-01, -2.9512e-01,  1.1179e-01,
        3.1096e-01,  1.6743e-01,  1.0240e-02, -3.1013e-01, -5.5090e-01,
        1.1422e+00, -7.6622e-01,  1.4698e-01,  2.2269e-01,  2.5664e-02,
        -7.6496e-01, -2.1084e-01, -6.8488e-02,  6.0437e-01,  4.8646e-01,
        5.8086e-01,  2.9018e-01, -1.8746e-01, -3.4241e-02, -3.9994e-01,
        9.0737e-02,  1.8660e-01,  8.1049e-01, -4.4343e-01, -1.3802e-01,
        -1.3496e+00, -5.0674e-01, -3.3386e-01, -4.0589e-01, -4.7747e-01,
        2.8361e-01,  3.2743e-01, -3.7912e-01, -5.3924e-01,  5.3359e-01,
        -3.4897e-01, -2.5918e-01,  2.7075e-01,  6.8634e-02, -9.0838e-01,
        -7.8908e-01, -1.0373e+00, -6.8172e-01, -6.6682e-01,  5.1928e-01,
        6.3022e-01,  9.0841e-01, -2.4363e-01,  3.5669e-01,  4.1984e-01,
        2.4104e-01,  1.6118e-01,  2.2240e-01, -4.4022e-02,  2.8779e-01,
        -5.9524e-01,  4.7097e-01, -5.5882e-01,  5.7729e-02, -5.0785e-01,
        1.2488e+00,  4.7346e-02,  2.3073e-01,  2.4335e-01,  5.0111e-01,
        -1.1307e-01, -1.4431e-01,  4.3640e-01, -4.5511e-01, -2.3307e-01,
        1.9474e-01,  3.6845e-01, -4.3227e-01, -6.6592e-01,  5.0662e-01,
        8.4723e-01,  5.6983e-01,  3.5284e-01, -1.4117e-01,  1.0688e+00,
        -4.7030e-01, -3.8551e-02,  2.9681e-01, -5.0635e-02,  9.4503e-02,
        -7.5778e-02,  1.8829e-01, -3.0379e-01, -2.0343e-01, -6.5320e-01,
        -1.3202e+00, -4.5640e-02, -7.2086e-01,  3.8831e-01, -9.5982e-02,
        5.7830e-01, -8.4707e-02,  2.0041e-01, -8.9187e-03,  3.4916e-01,
        2.8183e-02, -1.4468e-01,  3.2100e-02,  2.2229e-01,  7.4581e-02,
        3.6962e-02,  1.0359e-01, -9.1277e-02, -2.8647e-02, -5.3648e-01,
        9.1357e-01,  2.3564e-01,  8.0686e-01,  1.5542e-01, -3.8936e-02,
        2.9242e-01, -3.5408e-02,  9.7101e-02, -5.4232e-02,  4.5437e-02,
        -3.0103e-01,  5.6157e-01,  2.5107e-01,  8.4846e-01, -5.9651e-01,
        2.7440e-01, -2.5419e-01,  6.7841e-01,  4.4960e-02,  4.3721e-01,
        -3.0615e-01, -1.2062e+00, -4.8617e-02, -4.4453e-01,  5.3177e-02,
        4.5187e-01,  4.0684e-01, -3.0080e-02, -8.7397e-03,  5.8625e-01,
        1.1921e+00, -2.9881e-02,  7.8536e-02, -1.1404e-01, -3.5684e-01,
        -3.1464e-01, -1.5432e-02, -4.4744e-01, -2.3854e-01,  4.5062e-01,
        -7.4909e-02,  1.5093e+00, -2.0528e-01, -6.9638e-02,  3.7344e-01,
        -8.1869e-03,  2.5659e-02,  3.4147e-01,  4.8350e-02, -6.7471e-01,
        8.9769e-01, -3.8110e-01,  5.5660e-01,  1.4680e-01,  1.6168e-01,
        -3.5116e-01,  1.1158e-01, -1.4302e-01, -3.3229e-02, -2.0068e-01,
        5.7529e-01,  3.7258e-01,  2.5619e-01,  2.4951e-01,  3.8064e-01,
        4.4245e-01, -2.1411e-01,  4.1675e-01,  5.1566e-01, -6.5729e-01,
        -9.5103e-02, -2.5880e-01, -4.8837e-01, -3.0663e-01,  1.1735e-01,
        -5.1931e-01,  4.5215e-01, -3.1355e-01,  3.4188e-03,  4.2505e-01,
        4.8257e-01, -6.9320e-01,  4.8242e-01, -1.3117e-01,  9.2758e-01,
        -1.2691e-01, -7.8455e-02, -5.1750e-01, -4.0494e-01, -7.4016e-01,
        1.9133e-01,  7.0321e-01,  9.5235e-02, -3.9764e-01,  2.1406e-01,
        -6.9553e-01,  4.7177e-02,  3.6545e-01,  1.4195e-01, -3.1956e-01,
        3.1714e-01,  9.5993e-02, -9.4442e-02,  1.6189e-01, -6.4165e-01,
        -5.8677e-02, -3.5131e-01, -1.8541e-01,  7.0906e-01, -1.5018e-01,
        -2.6171e-01,  6.3877e-02, -3.4558e-01,  1.8645e-01, -2.3121e-01,
        -7.6265e-02, -3.9080e-01, -2.0780e-01,  3.8550e-01, -1.6413e-01,
        -8.9493e-01,  1.9678e-01, -1.0470e+00, -2.2085e-01, -3.1875e-03,
        5.2212e-01, -7.0129e-01,  8.3950e-02, -1.2285e-01,  6.7661e-01,
        -3.4647e-01, -5.6896e-01,  3.2410e-02,  3.7217e-01,  2.3740e-01,
        1.0831e-01,  5.0811e-04, -2.2765e-01, -9.3160e-01,  6.1379e-01,
        1.0702e+00,  4.2025e-01, -8.8172e-02,  7.0395e-01,  8.6511e-02,
        1.8360e-03,  9.4176e-02,  3.5769e-01,  6.5399e-01,  5.9204e-01,
        3.8046e-01, -2.9113e-01,  1.0069e+00,  2.0596e-01, -1.6070e+00,
        -1.4386e-03, -5.7539e-01,  1.0700e+00,  2.6225e-01, -1.7485e-01,
        -8.5222e-01, -3.6203e-01, -1.3778e+00, -2.3244e-01, -1.5953e+00,
        5.1410e-01,  4.6593e-01, -1.2239e+00,  7.6833e-02,  2.3335e-01,
        2.9420e-02, -1.4032e+00,  1.4122e+00,  6.0487e-01, -1.0800e+00,
        -1.7084e+00,  2.7995e-01, -2.2158e+00, -1.2913e-01,  9.6849e-02,
        -2.8648e-01,  9.9384e-02, -1.4398e+00,  1.8988e-02, -8.3461e-02,
        2.1892e+00,  6.6076e-02, -3.0655e-01,  3.3079e-01,  3.6023e-01,
        1.9027e-01, -2.4314e-01, -7.6840e-01,  1.3585e-01, -7.3921e-01,
        4.5626e-01,  4.3280e-02,  6.7079e-02, -8.9157e-01,  2.9056e-01,
        -5.1909e-01,  2.1993e-01, -5.2945e-01, -2.3090e-01,  5.7068e-01,
        4.5549e-01, -4.6779e-02, -3.3570e-01, -2.8264e-01, -1.6103e-01,
        1.0364e-01,  4.1529e-01,  4.9105e-02, -1.5017e-01, -8.5504e-01,
        2.3176e-01, -1.0457e+00,  9.4382e-02, -2.5041e-02, -2.2460e+00,
        2.3648e-01, -3.6042e-01, -1.7434e-01,  2.1595e-01,  1.6561e-01,
        -6.0030e-01,  2.0674e-01, -6.8525e-01,  3.3000e-01, -8.3924e-02,
        1.0050e+00, -6.8602e-01,  4.7776e-02, -2.2192e-01,  4.0267e-01,
        -2.2845e-02,  8.3664e-01,  7.0806e-01, -8.3382e-02, -6.9541e-02,
        6.2361e-02, -6.2480e-02, -7.5555e-01, -3.5870e-01,  3.1783e-01,
        3.9697e-01, -1.0395e-01,  6.5588e-02,  5.4351e-01,  8.5790e-02,
        -6.0903e-01, -2.2894e-02,  1.7367e-01, -2.1035e-01, -1.0319e-01,
        -9.8915e-01,  3.3951e-01,  5.0835e-02,  9.0754e-01, -7.6678e-01,
        1.1276e+00, -3.1257e-01,  6.6093e-01, -4.9330e-02,  6.4910e-01,
        -2.4113e-01, -7.2811e-01, -1.0975e-01,  2.2660e-01,  9.9435e-03,
        3.6843e-02,  4.0797e-02,  4.1304e-01, -2.0000e-01,  3.6960e-01,
        9.8803e-01, -4.6092e-02, -7.0761e-04,  6.4410e-01, -2.6288e-01,
        6.9584e-02,  2.3690e-01, -4.0300e-01,  2.3700e-01, -1.9553e-01,
        7.0026e-01,  5.5844e-01,  6.2825e-02,  3.3134e-01,  3.4055e-01,
        6.6851e-02, -4.7550e-01, -2.5697e-01,  8.1527e-03,  1.4599e-01,
        5.0232e-01, -2.2460e-01, -8.5581e-01, -6.5602e-02, -9.0994e-02,
        7.2315e-01, -2.1894e-01,  6.0144e-01, -3.4937e-01,  6.9322e-01,
        2.2948e-01, -2.1714e-01,  7.3983e-01, -9.2778e-02,  3.9690e-01,
        2.5248e-01,  4.2681e-01, -6.7069e-01,  1.0188e+00,  1.0891e-01,
        4.4897e-01, -2.5854e-01,  8.4171e-01,  1.4171e-02,  2.8170e-01,
        3.2720e-01,  2.3100e-01,  5.3144e-01,  2.3508e-01,  1.3885e-01,
        -1.1136e-01,  1.3285e-01,  2.7599e-01, -7.6592e-01, -9.7438e-01,
        -5.3950e-01, -1.1468e-01, -3.5562e-01, -1.1499e-01,  1.7101e-01,
        -3.3215e-01,  1.7073e-01,  6.0425e-02,  5.9316e-01, -2.2559e-01,
        2.5785e-02, -1.3285e-01, -2.1327e-02,  8.9789e-02, -2.7364e-01,
        -1.7247e-01,  3.4391e-02,  6.6213e-02,  7.5373e-02,  2.1117e-01,
        -7.0510e-01,  6.2246e-01,  9.6226e-02, -2.3675e-01, -3.5080e-01,
        1.9361e-01, -5.1938e-01, -1.6093e-01, -7.2983e-01, -1.2103e-01,
        5.6225e-01,  1.6527e-01, -6.8502e-01,  1.2255e+00,  3.0792e-01,
        5.1897e-02, -1.0234e+00,  2.0775e-01,  1.1056e-01, -7.0854e-01,
        -1.9383e-01, -2.8871e-01,  2.8617e-03, -1.1421e+00,  9.4126e-02,
        4.1046e-01,  1.9762e-01, -3.0034e-02, -2.2666e-01,  8.9127e-01,
        1.0630e+00, -2.2671e-01, -1.2372e-01, -1.9280e-01, -4.4449e-02,
        5.6831e-01, -1.3010e+00, -3.0496e-01,  5.0967e-01,  6.1976e-01,
        4.6871e-01,  5.8575e-01,  3.1418e-01,  1.2499e+00,  4.6815e-01,
        3.4621e-01,  1.3710e-01,  2.6213e-01,  5.3669e-02, -1.0390e+00,
        1.0555e-01,  2.9921e-02,  8.0447e-01, -1.2129e-01, -1.0454e-01,
        -7.2466e-01,  3.8285e-03, -3.3412e-01, -1.9884e-01, -1.0656e-01,
        4.8582e-01,  3.0180e-01,  2.0600e-01, -1.8858e-02,  1.4219e-01,
        -1.6578e-01, -3.2513e-01,  4.4642e-01, -3.1896e-01,  3.4108e-01,
        -8.4024e-01,  5.8424e-02, -8.2996e-01, -2.1418e-01, -3.4915e-02,
        -3.6948e-02, -8.2369e-02, -6.7659e-01, -6.4420e-02, -3.1375e-02,
        6.8094e-01, -8.8645e-01,  1.8597e-01, -6.6118e-01,  3.5645e-01,
        -2.9454e-01, -4.3811e-01, -5.4988e-01,  9.2145e-02, -9.2561e-01,
        -3.3007e-01,  7.0858e-01,  4.5325e-02, -1.3523e+00,  3.3630e-01,
        5.6257e-01,  2.5980e-01, -5.0692e-01,  6.0048e-01, -5.3352e-01,
        -4.2426e-01, -7.1951e-02,  3.3710e-02, -1.1575e+00, -4.1365e-01,
        3.6617e-01, -1.2757e-01, -6.2422e-02,  7.8650e-01,  3.8520e-01,
        -9.5611e-01, -7.3084e-01, -5.6212e-01, -3.3327e-01,  1.5215e-01,
        5.6160e-01,  3.3914e-01,  4.6195e-01, -3.4419e-01, -9.2718e-01,
        1.9360e-02,  2.5455e-03,  6.1603e-01,  1.3480e-01,  3.4320e-01,
        -4.0152e-01, -1.6539e-01, -2.9257e-01, -5.9113e-01, -8.5610e-01,
        1.5341e-01,  8.6365e-02, -1.2988e-01,  3.9709e-01, -5.7864e-02,
        2.2912e-01, -2.3904e-01,  6.2221e-01,  7.0673e-01,  9.7763e-02,
        -3.4556e-01, -3.3575e-01, -5.9741e-01,  6.5057e-03,  5.2661e-01,
        2.8815e-02,  1.4568e-01, -7.3221e-01,  8.5461e-02,  2.6660e-02,
        -4.4620e-01,  1.5912e-01,  2.8994e-01, -4.3757e-01, -1.3878e+00,
        -5.6219e-01, -1.0447e-01,  7.8082e-01, -4.6428e-02,  4.9843e-01,
        -8.3902e-01, -3.8791e-01, -5.5716e-02,  2.4348e-01,  1.6681e-01,
        6.2453e-01,  4.6100e-01, -3.2979e-01, -1.5132e-01,  1.6708e-01,
        -7.8048e-01, -1.1771e-01,  2.2318e-03, -4.7377e-01, -6.5502e-01,
        -5.0597e-01, -9.6113e-02, -6.8268e-01,  1.3110e-01, -2.1437e-01,
        -4.8285e-01,  4.1987e-01, -8.4851e-01, -4.7893e-01,  6.0092e-01,
        2.6677e-01, -1.4863e-01,  3.4160e-01,  1.8781e-02, -1.0453e+00,
        -3.1119e-01, -2.0322e-01,  3.6729e-01,  7.9912e-04, -1.3115e-01,
        -5.2544e-02,  1.1462e-01, -2.7177e-01, -2.5987e-01, -1.9652e-01,
        1.8787e-01, -1.4953e-01, -9.8640e-02, -3.5946e-02, -1.1334e-03,
        3.7273e-03, -3.1378e-01,  2.8012e-01,  2.8203e-01, -3.3547e-01,
        -3.7152e-01,  4.1959e-01, -4.3732e-01,  7.3200e-02,  2.2786e-02,
        4.6925e-02, -3.8293e-02, -8.1039e-01, -6.9046e-02, -1.1057e-01,
        1.0487e-01,  4.7887e-01, -5.1313e-01,  5.2708e-01, -2.1423e-01,
        -2.3327e-01,  1.4109e-01,  6.7770e-02, -6.5712e-02,  6.1695e-01,
        1.2686e-01, -6.1948e-01, -1.4960e-02, -7.6245e-01,  1.1665e-01,
        -4.8976e-02, -2.3129e-01,  1.6251e-02, -4.4750e-01,  2.2854e-01,
        8.7491e-01,  2.2577e-02, -2.3348e-01,  6.3265e-01,  6.0521e-01,
        1.2776e-01,  9.6722e-02,  2.1228e-01, -3.8639e-01,  4.2142e-01,
        1.2821e+00, -4.1292e-01,  1.3769e-01,  2.5240e-01, -4.5286e-01,
        -1.5814e-01, -1.4966e-01, -2.5318e-02, -9.5732e-02, -5.1496e-02,
        4.4562e-01,  1.8568e-01, -2.2494e-01,  1.1262e-01, -1.2601e-01,
        3.1221e-01,  1.0752e+00,  7.5216e-02, -1.0976e+00,  1.4101e-02,
        6.7161e-01, -1.3853e-01, -1.7118e-01,  3.6181e-01,  1.4476e-01,
        2.6507e-01,  3.4079e-02, -1.2187e-01,  5.4267e-01,  1.5435e-01,
        -3.5425e-01,  3.6057e-01,  7.9481e-02, -4.5799e-01, -4.0751e-01,
        2.2742e-01, -7.7486e-01,  2.4327e-01, -7.5265e-02, -3.9361e-01,
        -5.5481e-01,  4.5260e-01, -1.1734e+00, -1.2067e+00,  5.1001e-01,
        -9.4489e-01,  2.0036e-01, -7.0500e-01,  2.9383e-01, -1.6895e+00,
        1.8246e+00,  1.2588e+00,  1.2073e-01, -7.0800e-01,  2.4916e-01,
        1.0311e-01, -8.4445e-02, -3.6198e-01, -1.0885e+00, -6.5886e-01,
        4.4940e-01, -2.7716e-01, -3.3805e-01, -1.4636e-01,  5.0019e-01,
        8.8397e-01, -6.9386e-01,  3.5189e-01,  4.7318e-01,  6.6588e-01,
        -4.4960e-02, -3.8862e-01, -6.0726e-02,  4.1090e-01, -2.7024e-01,
        4.4049e-01, -4.5307e-02,  6.1496e-01,  7.2913e-02, -8.5326e-01,
        4.4814e-01, -5.9663e-01,  1.1029e+00,  1.8972e-01, -7.9642e-02,
        -1.0126e+00, -1.7830e-01, -1.0244e+00,  1.0228e-01, -1.4029e+00,
        1.6746e-01,  3.5756e-02, -5.9462e-01,  1.6100e-01,  5.9572e-02,
        1.5229e-02, -9.8989e-01,  1.0167e+00,  1.7727e-01, -3.8433e-01,
        -1.2960e+00,  3.7435e-01, -1.9910e+00, -8.0066e-02,  9.4181e-03,
        -7.7069e-02,  4.1967e-02, -9.2880e-01,  1.9713e-02,  1.0931e-02,
        -1.5680e-01,  2.4526e-01, -6.1519e-02,  4.1716e-01, -1.0707e+00,
        2.8834e-01,  2.3017e-01,  2.3812e-02, -1.8734e-01, -2.8957e-01,
        -1.1241e-01,  1.1323e+00,  1.0681e-01,  5.2306e-01,  2.4099e-01,
        -6.3081e-02,  1.3166e-01, -3.3914e-01,  9.1580e-01,  2.2475e-01,
        -2.3722e-01, -6.2021e-02, -2.3132e-02, -2.4783e-01, -1.8574e-01,
        -2.9329e-03, -4.4206e-01, -1.3530e-01, -7.4441e-02,  2.9810e-02,
        -3.3499e-01,  3.4059e-01, -1.5357e-01,  5.8026e-01,  8.4610e-01,
        -1.4386e-01,  7.0043e-01, -4.7883e-02,  2.2089e-01, -1.9543e-01,
        5.9831e-01, -2.9561e-02,  4.4163e-01,  2.4822e-02,  9.6381e-02,
        -1.2689e+00, -1.8607e-01, -2.6633e-01, -9.8341e-03, -4.9547e-01,
        -1.7242e-02,  2.2412e-01, -5.6516e-01,  2.4961e-01,  2.9575e-01,
        -1.0470e-01, -2.4447e-01,  5.4045e-01,  2.8208e-01, -1.7524e-01,
        -6.4056e-01, -8.4139e-01, -3.9013e-01, -3.6680e-01,  5.5297e-01,
        2.2261e-01,  1.0945e-01, -7.9964e-02, -3.7347e-02, -8.4769e-03,
        -1.5353e-01,  8.5592e-01,  1.0039e-01, -1.2687e-01, -6.0720e-01,
        2.6220e-01,  2.9648e-01,  3.6308e-01,  4.1747e-01,  2.5036e-01,
        -6.5873e-01,  5.4360e-02, -2.5879e-01,  8.4992e-02, -1.0453e-01,
        9.7640e-01,  2.0398e-01,  1.1594e-02,  4.2106e-02,  1.8190e-01,
        -1.3919e-01, -1.9911e-01,  1.9688e-01,  7.9231e-01, -2.0468e-03,
        -1.8954e-01, -4.3741e-01, -1.1745e-02, -8.2276e-01, -8.7594e-02,
        2.8780e-01,  2.1189e-01, -5.9453e-02, -7.0469e-01, -9.9669e-02,
        3.9213e-01, -1.1530e-01,  3.1924e-01,  6.0637e-01, -9.9167e-01,
        -3.4022e-01,  1.3316e-01,  6.3793e-01,  7.5888e-03, -9.2365e-03,
        -8.4695e-01, -6.3179e-01, -5.0712e-01, -7.4666e-02, -4.7694e-01,
        1.8139e-01,  9.5771e-02,  6.8797e-03, -2.2461e-01, -3.5122e-01,
        2.3453e-01, -6.7112e-01,  4.7122e-01, -7.2443e-01, -5.4266e-01,
        -9.9436e-02,  1.1297e-01, -9.4975e-01,  1.9686e-01,  8.3097e-02,
        1.7468e-01,  4.5495e-01, -8.8142e-01,  4.6428e-01,  9.1053e-02,
        1.1373e-01,  1.5106e-01,  2.2050e-01, -3.4928e-01,  5.3114e-02,
        -6.9953e-01, -4.1704e-01,  8.7352e-01,  1.4113e-03,  1.3896e-02,
        6.2099e-02, -1.3816e-01, -6.3594e-01, -7.2258e-01, -1.3783e+00,
        -3.7534e-02, -3.7449e-03, -3.2914e-01,  2.7357e-01,  1.4655e-01,
        2.0722e-01, -5.8523e-01,  8.6511e-01, -9.8916e-01, -2.5035e-01,
        -6.2712e-01, -9.5320e-01, -8.2434e-01,  8.1096e-02,  5.2128e-01,
        -2.1008e-01, -1.0685e-02, -8.0097e-01,  9.0528e-02, -1.2567e-02,
        -4.1359e-01,  1.7280e-01,  3.5903e-02,  2.4225e-01,  9.7685e-01,
        7.4352e-02, -2.7586e-01, -7.1037e-01, -4.8540e-01,  2.7596e-01,
        8.7280e-01, -3.4506e-02,  3.7486e-01,  4.7916e-02,  3.6737e-01,
        -8.1051e-01, -7.3971e-01,  5.9155e-01, -3.3270e-01, -9.9396e-02,
        5.3453e-02,  4.1585e-01, -4.9388e-01, -8.4243e-02,  3.5460e-01,
        9.9658e-01, -3.2417e-01,  7.5998e-01,  5.4708e-01, -1.3867e-02,
        1.3055e-01,  4.9821e-02,  2.4095e-01,  2.4052e-01, -7.8922e-02,
        -3.1245e-01,  1.0820e-01,  3.6264e-01,  6.2867e-01, -6.8237e-01,
        4.1217e-01, -2.6653e-01,  1.1288e+00, -3.0342e-01,  8.8072e-01,
        -1.6846e+00, -2.4449e-02, -1.7928e-01,  3.8415e-01, -1.3663e-01,
        -2.3126e-01,  4.8624e-01, -7.8132e-02, -6.4855e-01, -2.2591e-01,
        -9.2147e-01,  4.3805e-02,  6.0768e-01,  4.9971e-01, -1.2038e+00,
        -1.1372e+00, -4.2074e-01, -6.8520e-01,  2.6489e-02, -3.4665e-01,
        7.1645e-01,  7.6983e-01, -2.9879e-01, -4.6609e-02,  7.2216e-02,
        -3.6850e-01, -5.9807e-02, -1.0969e+00, -3.0798e-01,  1.6420e+00,
        -1.4701e-01,  4.6605e-01, -1.0889e+00, -2.3580e-01,  5.5496e-02,
        1.1546e+00, -1.1429e-01,  1.4304e+00, -2.8388e-01,  1.5480e+00,
        -1.5792e-01, -3.9228e-01,  1.1507e+00,  2.0208e-01, -8.7291e-02,
        -4.8717e-03,  1.3991e+00, -1.4783e+00, -5.3358e-01,  1.0522e+00,
        1.6671e+00, -6.5517e-01,  2.2308e+00, -1.0890e-01,  1.8058e-01,
        1.9813e-01, -8.4085e-03,  1.3988e+00,  1.2289e-01,  5.6204e-02;
    bias2 << -0.3107,  0.1317, -0.9490,  1.1658, -1.4644,  1.4196,  1.0762, -0.7573,
        -0.2004,  0.3568,  1.0685, -0.2663,  0.3834,  1.3697, -0.5341,  0.6816,
        -0.1637, -0.2753, -0.3535,  0.1152, -0.1880, -0.3566, -0.1795, -0.2791,
        -0.0446,  1.4193, -0.5676,  0.0403, -1.3635, -0.8971, -0.5339, -0.7329,
        0.8001, -1.4294,  0.3646;
    weight4 << -1.5888e-01, -8.5999e-02,  6.4393e-01, -1.0959e+00, -4.3857e-01,
        -3.6698e-03,  1.3612e-01, -7.6085e-01,  7.3661e-01,  4.9741e-01,
        -1.0724e+00, -3.7233e-01,  8.8011e-01,  1.0901e+00, -6.1351e-01,
        1.7229e-01,  4.3010e-01, -4.3096e-01,  8.8046e-01, -6.6597e-01,
        -1.1715e-01, -3.6362e-01, -5.8057e-01,  6.6985e-01, -1.1758e+00,
        6.1249e-01,  8.4842e-01,  8.7052e-01,  2.7611e-01,  6.0196e-04,
        -1.8411e-01, -1.4727e-01, -1.7658e-01, -3.4017e-01,  2.4207e-01,
        2.7771e-02, -9.0728e-01,  1.2098e-01, -1.2561e-02, -2.5137e-01,
        -1.0995e+00, -5.9352e-03, -4.4799e-02,  3.8899e-02, -1.3031e-02,
        1.2957e-01,  1.4258e+00,  5.1375e-02,  3.6055e-02, -1.2223e-02,
        -8.9858e-01, -3.4949e-02, -1.1596e-02, -7.0568e-02, -4.3861e-02,
        6.7121e-01, -6.5101e-02,  1.4147e-01,  3.0597e-02, -4.2248e-02,
        -1.6033e-01,  5.3689e-02, -6.2959e-02, -1.0653e+00, -4.4656e-02,
        -1.1018e-01, -6.9813e-01,  1.4351e-02, -1.9473e-02,  1.4403e+00,
        4.7841e-03,  1.4823e-01,  6.4216e-01,  2.9696e-02,  5.8048e-03,
        2.8374e-03,  7.3030e-01, -6.3179e-03,  6.2396e-02, -4.9180e-02,
        4.7028e-02, -8.5298e-02,  1.3268e-02,  1.2936e-02,  2.5581e-03,
        6.5898e-02,  5.0503e-02,  1.8335e-04, -7.7136e-02, -8.1141e-03,
        6.0088e-02, -6.7164e-03,  6.2757e-01, -1.5225e-02, -1.3339e-02,
        4.4225e-02,  4.4246e-01,  9.6403e-03, -2.3488e-02, -2.2245e-03,
        -6.9258e-03, -3.4872e-02,  1.2314e-01, -1.2847e-02,  6.1940e-02,
        9.5351e-01, -1.8617e-01, -4.0019e-02,  1.3935e+00, -2.7912e-01,
        -8.0205e-02,  1.6921e-01,  7.5401e-01, -4.8469e-01, -5.8824e-01,
        -1.0366e+00, -9.4052e-02, -1.0235e+00,  6.5909e-01,  5.9372e-01,
        -2.2173e-01, -5.1704e-01,  4.2185e-01,  1.5376e-01,  7.5608e-01,
        -5.6568e-02,  4.4606e-01, -4.9589e-01, -6.8244e-01,  1.4772e+00,
        -6.2847e-01,  1.8209e-02, -8.6742e-01,  2.5132e-01, -4.1317e-01,
        7.3504e-01,  4.4873e-02,  7.8186e-01, -4.2391e-01,  5.4203e-02;
    bias4 << -0.1873, -0.1453, -1.0477,  0.1538;
    out_mean << -24.245327789699125, -4.972238784710127, 858.4484288696027, -1.6173056382356812;
    out_std << 106.60507509883607, 342.38221052518674, 505.27382432744236, 6.871454392023513;
    in_mean << 0.04755175580815579, -0.10310912487916579, -0.00822193571007142, 59.96615465535339, 1989.5623071836276, 0.8023074986664168, 0.0998446902747603, 0.34080175360478987;
    in_std << 0.028762000257431117, 0.5305530689625024, 0.9490676466883241, 23.21776169763347, 869.4323970393215, 0.29001461209388496, 0.057700841504368966, 0.10050279338464484;
}

//Old
void JackalDynamicSolver::load_nn_gc_model(){
    weight0 <<  4.3048e-02,  6.1408e-02,  2.6195e-02,  2.0294e-02,  3.5062e-02,
        -1.0517e-01,  9.9070e-02,  2.3669e-02, -5.5610e-01, -1.6376e-01,
         4.9504e-02, -2.8987e-02, -5.2506e-02,  2.0303e-01,  5.9441e-02,
         5.4853e-02,  1.4948e-01,  1.3111e+00,  1.9421e-02,  2.0466e-02,
         3.7577e-02, -2.3337e-01, -2.4271e-02, -1.1476e-01,  2.2506e-02,
         9.8571e-01,  1.0287e+00, -9.5947e-03, -1.7813e-02,  6.5850e-03,
        -5.8891e-03, -1.0262e-01, -2.0518e-01,  9.5938e-01, -1.1949e+00,
        -1.3132e-02, -2.6193e-02,  7.2231e-02,  2.8701e-03, -1.1930e-01,
        -6.4811e-01,  8.8913e-02,  2.8725e-02, -5.1134e-02, -9.5663e-02,
         3.2414e-01,  6.7280e-02, -2.1560e-01,  1.0332e+00,  6.6428e-02,
        -8.2122e-03,  2.6753e-01,  4.8884e-01, -1.3558e+00, -3.7150e-01,
         2.8550e-02, -5.2098e-01,  1.0818e-01, -9.2304e-03, -3.2496e-02,
        -5.6165e-02,  2.0715e-01,  7.0873e-02, -3.0430e-01,  5.5839e-01,
        -1.0939e-02, -7.7093e-01,  4.5593e-02,  8.2832e-02, -3.6198e-01,
        -9.6596e-02,  1.2502e-02, -4.6231e-02,  9.3868e-01,  1.2784e+00,
        -1.0473e-02, -1.8526e-02,  8.2865e-02, -3.9377e-03, -2.5855e-02,
         1.3802e-01, -5.3480e-01,  9.3873e-03, -4.6218e-02, -8.8143e-02,
         2.4649e-01,  4.6637e-02,  5.9164e-02,  3.2470e-01, -1.1585e+00,
        -3.7327e-03,  1.9506e-02,  3.6686e-02, -1.8814e-01,  7.2382e-02,
         8.1036e-02, -2.8672e-01, -4.3476e-01, -9.1057e-01, -1.2247e-03,
        -4.4764e-03, -9.9461e-03, -4.0785e-04, -1.0807e-01,  7.4878e-02,
        -7.9288e-01,  1.1463e+00,  8.0244e-03,  1.6543e-02, -4.3123e-02,
        -2.6215e-03,  8.2543e-02, -3.9407e-01, -7.3907e-01, -7.7291e-02,
        -6.6176e-02, -1.2396e-01,  4.8534e-01,  2.9738e-01, -1.3506e-01,
        -5.6242e-03,  9.3973e-01, -2.8240e-02,  5.5313e-03,  1.0184e-02,
         4.7458e-02,  3.8673e-02,  2.9870e-01,  3.7155e-01, -4.5445e-02,
        -6.7512e-01, -2.5973e-02, -4.9610e-02,  1.6495e-01,  4.5633e-02,
         4.5646e-02,  8.9938e-01,  1.8516e-01, -1.3684e-02,  7.5236e-02,
         1.4547e-01, -5.8428e-01, -1.6416e-01, -9.3692e-03, -5.6166e-02,
         6.8716e-02, -1.3012e-02,  9.4412e-02,  1.8056e-01, -6.3107e-01,
        -1.5286e-01, -4.1809e-03, -1.1603e-01, -6.9317e-01,  9.4933e-02,
         2.5496e-02,  4.5947e-02, -1.4945e-01, -1.3693e-02,  3.8084e-01,
         1.3033e+00, -4.6110e-02, -1.0078e-02, -3.6936e-02, -6.9200e-02,
         2.2601e-01,  3.5847e-02, -5.7925e-02,  2.9298e-01,  9.0550e-01,
         3.4901e-02, -1.1863e-02, -2.2328e-02,  1.4480e-01, -3.1564e-02,
        -1.5687e-02, -1.8749e-01,  3.5708e-01, -3.5514e-02, -8.8586e-03,
        -1.9379e-02, -1.2587e-01, -6.1394e-02, -7.9981e-03,  4.7652e-01,
        -3.7520e-02,  7.3920e-01,  3.6854e-02,  6.6222e-02, -3.1729e-01,
        -7.9461e-02, -9.5391e-02, -2.0482e-01,  7.9517e-02,  1.5884e-01,
        -2.8286e-02, -5.5412e-02,  1.7013e-01,  2.1985e-02,  2.2662e-01,
         4.6616e-02, -1.0079e+00,  7.5020e-03,  2.7353e-02,  5.1273e-02,
        -1.0733e-01, -2.4514e-01,  2.0763e-02,  1.9761e-01, -3.4631e-01,
        -1.1619e+00,  7.9694e-03,  1.3356e-02, -4.9095e-02, -1.1567e-02,
         9.1006e-02,  9.0474e-02, -1.0427e+00,  5.7941e-01, -9.8446e-03,
        -2.0325e-02,  1.9407e-05,  2.1082e-02, -1.5570e-01,  1.1998e-01,
        -2.5301e+00,  3.3729e-02,  1.1663e-02,  2.1445e-02, -7.5143e-02,
        -1.1367e-04,  7.8427e-02, -2.9016e-01,  2.1024e+00, -1.4114e-02,
        -9.0342e-03, -1.5961e-02,  6.4431e-02,  2.0813e-02, -5.8033e-02,
         4.1739e-01, -5.2732e-01, -2.6589e-01, -1.0871e-02, -1.8185e-02,
        -3.1650e-02, -6.2898e-03,  6.5243e-02,  1.1566e-01,  2.1791e-03,
        -1.3412e-03, -4.6895e-01, -8.6814e-01,  4.3594e-01,  9.0806e-02,
         2.8354e-02;
bias0 <<  0.9557,  0.3086, -1.1127,  1.1054,  1.5918,  0.1871,  0.7385,  1.6650,
        -0.8195,  0.9687,  0.3662, -0.0641,  0.3998, -0.6469,  1.7500, -1.1990,
         0.7029,  1.1324,  0.4076, -0.8646,  2.6968, -0.5247,  1.4090, -0.5543,
         0.3880,  0.0992, -0.2841,  0.2537,  1.3751, -1.5613, -0.7966, -2.1929;
weight2 <<  1.5630e-02,  1.1955e-01,  2.3215e-01, -2.3676e-01,  3.6274e-01,
        -2.1092e-01,  2.3341e-01, -5.3359e-01, -2.3638e-01, -3.1614e-01,
         3.6277e-01,  7.0663e-02,  4.3984e-02,  1.7500e-02, -3.9786e-01,
         1.0065e-01,  3.4902e-02,  2.1392e-01,  9.3549e-02, -1.7884e-01,
         4.0491e-01,  7.1173e-02, -2.8008e-01, -2.0696e-01,  1.9594e-01,
        -3.3431e-01,  1.5566e-01,  5.6806e-02, -4.0956e-01,  4.5213e-01,
         6.3736e-02, -9.4510e-02, -1.2290e-01,  1.6331e-01,  1.2325e-01,
        -7.7950e-02, -8.9531e-01,  2.5399e-01, -5.9314e-02, -2.3175e-01,
        -1.3020e-01, -1.9232e-01,  8.9008e-02, -1.7885e-01,  1.9054e-01,
         2.1745e-01, -6.7409e-02,  4.5291e-01, -1.9638e-01, -1.9450e-01,
         1.8430e-01, -2.0699e-01, -5.9113e-01,  3.0900e-01, -3.9100e-01,
         5.3869e-01, -2.8032e-01, -2.8697e-01, -5.5900e-02,  3.7842e-02,
        -1.2022e+00,  2.1462e-01, -3.8758e-01,  2.7379e-02, -1.2517e+00,
        -6.7999e-01, -2.3047e-01,  8.8717e-02,  9.1404e-02,  1.2406e-01,
         4.3687e-01, -5.4449e-01,  1.0705e-01,  9.9036e-02,  4.8834e-01,
         5.4830e-01, -1.8153e-01, -1.3510e-01, -5.3639e-01, -2.3088e-01,
        -2.8334e-03,  6.2630e-01,  8.2256e-01, -5.1658e-02,  9.7391e-01,
        -4.8977e-01, -3.0993e-03, -4.7365e-03,  2.7891e-01, -1.2060e+00,
         7.7163e-02,  8.3827e-03,  2.7869e-02, -2.9155e-01, -2.2407e-01,
        -6.8741e-01, -3.9172e-01, -8.7643e-02,  1.0579e-01,  1.6118e-02,
        -1.4589e-01, -3.3856e-02,  1.1396e-01,  4.5558e-02, -5.1742e-01,
        -1.8123e-01, -1.6976e-01,  2.2242e-01, -1.4148e-01,  1.5922e+00,
        -2.0123e-02,  4.4821e-01, -2.1344e-01,  2.4270e-01, -1.9894e-01,
        -3.7092e-01,  1.3091e-01,  2.0584e-01, -2.8787e-01,  9.2689e-03,
         2.6753e-01, -8.3050e-02,  7.2707e-02,  4.5908e-02,  1.9249e-01,
         4.3699e-01,  1.0329e-01, -7.5118e-02,  1.6189e-01, -6.8124e-02,
         1.0368e-01,  9.8203e-01, -9.8975e-01,  2.0889e-01, -1.9208e-01,
         1.0205e-01, -2.6529e-01,  3.0162e-01, -1.3523e-01, -2.6647e-01,
         1.2593e-01,  1.2257e+00,  7.5668e-02,  2.6201e-01, -3.6895e-01,
        -2.7783e-01,  3.1090e-02, -1.7623e-01,  2.6971e-01,  4.1436e-01,
         3.1525e-01,  6.3769e-01,  2.0179e-01, -2.6960e-02, -3.0898e-01,
        -7.2500e-02,  6.2088e-01,  3.4516e-01, -1.7084e-01, -3.9619e-02,
         5.6811e-02,  8.7519e-02,  2.1366e-02,  4.5301e-01, -1.8696e-01,
        -5.3594e-02, -1.2221e-01, -3.4895e-02,  2.6731e-01,  3.5074e-01,
        -6.7369e-02,  2.4864e-01, -1.0647e-01, -1.2073e-01,  3.8345e-01,
        -2.5997e-01,  3.2401e-01, -5.2805e-02, -9.1097e-02,  4.2437e-01,
         2.6282e-01, -2.4339e-01,  1.0708e-01,  2.3173e-01,  6.7860e-01,
         4.3502e-01, -6.0974e-01, -5.6752e-01,  3.9390e-01, -6.3820e-01,
        -8.6190e-02, -3.4193e-04, -1.8358e-01,  2.8607e-01, -3.2489e-01,
        -4.9681e-01, -2.9496e-01, -7.0094e-01, -1.2862e-02, -4.9725e-01,
         2.1108e-01, -2.7397e-01, -2.2998e-01, -7.2751e-02,  2.8128e-02,
        -8.8501e-01, -1.5432e-01,  1.3103e-01,  3.6333e-01,  3.1457e-01,
         2.6749e-02, -3.0488e-01,  5.9011e-01,  2.2723e-01,  6.7974e-01,
        -4.3525e-01,  2.8131e-01, -2.4100e-01, -1.5861e-01, -8.9762e-02,
        -9.0601e-01,  5.9554e-01, -2.8577e-01, -1.8825e-01, -1.6293e-02,
        -9.8100e-02,  1.9165e-01, -4.0648e-02,  2.3478e-01,  4.0741e-01,
        -2.5942e-02,  4.7214e-01, -2.5103e-02, -5.6617e-01, -1.2731e-01,
         4.3158e-01, -5.6489e-02, -5.2512e-01,  4.9397e-01, -4.9567e-02,
         6.3124e-02, -4.1953e-01, -7.1188e-02,  5.1605e-03, -7.3800e-01,
        -9.5311e-02, -5.0965e-01, -1.1644e-02, -5.2841e-01,  3.9155e-01,
         5.8395e-01,  3.5020e-02,  5.5465e-01, -3.9005e-01, -3.6570e-01,
         2.2934e-01, -1.6433e-01,  4.1729e-01,  5.6386e-01,  3.8331e-01,
         1.7245e-02, -2.3305e-01,  2.5411e-01, -4.6206e-01, -3.3157e-03,
         2.0019e-02,  5.4735e-01,  2.2207e-01,  9.4781e-02,  2.6532e-01,
        -1.8294e-01,  3.2643e-01,  3.7807e-02,  6.3549e-02,  2.5012e-01,
        -1.2254e-01,  3.9185e-01,  4.0489e-01, -3.1748e-01, -3.9005e-02,
         4.3830e-01, -2.3555e-01, -3.4853e-01,  1.7998e-01, -3.4430e-01,
        -7.5980e-02,  1.4866e-01, -2.1381e-01,  5.8293e-01, -2.9049e-01,
        -1.5164e-01, -7.6351e-01, -6.7271e-03,  2.6752e-02, -3.1506e-01,
         5.9897e-01, -1.3900e-01,  8.6865e-02, -5.7849e-01, -8.6922e-02,
         1.8821e-01, -4.5480e-01,  4.5328e-01, -9.6660e-02, -2.6191e-01,
        -2.3718e-01, -3.7950e-01,  2.1618e-01, -6.6799e-01,  1.1259e-01,
         1.4218e-02, -1.8417e-02,  1.4503e-01,  5.2771e-01, -9.7454e-03,
         7.4913e-02,  2.4599e-01,  2.0139e-01, -2.7879e-01,  2.4819e-01,
         2.0446e-02, -4.4619e-02, -3.5411e-01, -2.1986e-01, -5.2728e-01,
         1.2155e-01,  2.3174e-01,  2.3624e-01,  3.3660e-01, -2.4966e-01,
         2.0722e-01,  1.2135e-01,  1.8004e-01, -3.5652e-02, -9.6479e-02,
        -3.5541e-01,  7.6302e-03,  7.7918e-02,  1.3050e-01,  9.2409e-02,
         9.0286e-02, -5.4855e-01, -7.0962e-03,  2.3402e-01,  1.1729e-01,
        -1.4024e-01, -5.0314e-01, -9.0805e-02,  1.8015e-01,  5.3654e-01,
         2.2408e-01, -3.0237e-01,  5.5823e-01,  8.2316e-02, -3.5320e-01,
        -3.0892e-01,  6.3811e-01, -3.9739e-01, -3.5080e-01, -1.3455e-01,
        -1.5608e-01, -5.0700e-01, -4.9253e-01, -1.3437e-01,  3.6363e-02,
        -3.0883e-01,  3.1121e-01,  6.7161e-01, -7.3347e-01, -2.0299e-02,
        -6.3434e-01, -2.3876e-01, -6.1278e-01,  7.5658e-02,  1.0132e-01,
        -3.9689e-01, -9.8121e-02,  5.5632e-01, -4.1137e-01, -2.5009e-01,
         8.3138e-02, -2.5928e-02, -2.3363e-01,  4.5513e-01,  5.7035e-01,
         2.9446e-01, -4.5573e-01,  2.7192e-01, -1.5349e+00,  1.9962e-01,
        -2.0121e-01,  6.7749e-01, -4.8721e-01,  1.9609e-01,  8.0050e-02,
         4.5117e-01,  2.8283e-01, -4.3161e-01,  4.7784e-01, -8.3142e-01,
        -3.5723e-01, -2.5098e-01,  1.5618e-01,  1.3679e-02, -2.3879e-01,
        -6.4057e-01,  7.4034e-01, -1.2229e-01, -6.7171e-02,  9.7098e-01,
         7.3389e-01,  4.3805e-01,  1.0656e+00, -2.0745e+00, -4.1491e-02,
         6.4240e-02,  2.8388e-01, -3.5985e-01,  3.1553e-01, -1.2547e-01,
         5.5464e-01,  3.0921e-01, -1.7903e-01,  2.2022e-01, -2.7028e-02,
        -4.3668e-01, -3.6612e-01, -1.3134e-02,  1.6065e-01, -5.7675e-02,
         1.4663e-01,  2.4585e-01,  8.8730e-03, -1.4942e-02, -1.0271e-01,
        -2.4191e-01, -5.0192e-01,  3.2426e-02,  1.5068e-01, -2.1499e-01,
        -2.5573e-01,  1.9843e-01,  5.4216e-01,  1.3277e-01,  7.2467e-02,
         4.1453e-01, -1.6642e-01,  1.8939e-01, -7.5029e-01,  1.6236e-01,
        -2.3711e-01,  3.7631e-02,  2.1417e-01,  4.6700e-02,  2.3652e-01,
        -7.5489e-02,  4.0714e-01,  1.3645e-01,  7.8816e-01,  3.9902e-01,
        -3.2186e-01, -1.6084e-01, -1.4081e-01,  7.1937e-01, -7.5822e-02,
         9.4264e-02,  7.4305e-01, -3.5980e-01,  5.7636e-01, -1.7121e-01,
        -5.7798e-01,  1.0820e-01, -2.8135e-01, -2.3933e-01, -3.6011e-01,
        -4.3574e-01,  2.7703e-01, -1.0295e-01, -2.7811e-02, -2.8247e-01,
        -9.3404e-02, -1.1352e-01, -4.0040e-01, -5.5175e-01, -1.0372e-01,
        -1.1253e-02,  8.6166e-02, -4.7228e-01,  3.7906e-01, -1.7935e-01,
         6.7284e-02,  1.5428e-02, -1.8391e-01, -3.0369e-01, -1.3999e-01,
        -2.3855e-01,  8.3093e-02,  3.3003e-01,  2.2045e-01,  1.9432e-03,
         6.1237e-01, -1.1379e-01,  5.5622e-02, -6.7393e-02,  4.4679e-01,
        -2.7219e-01,  1.7827e-01,  1.1598e-01,  2.8464e-01, -1.3612e-01,
        -1.4619e-01, -2.2750e-01,  6.7718e-01, -8.7913e-01,  7.3834e-01,
        -4.2138e-01,  1.0244e+00, -4.8204e-02, -1.3318e-01, -5.1281e-02,
         6.5781e-01, -1.2956e+00, -3.4576e-01, -9.4177e-01,  1.0130e+00,
         3.8926e-01,  7.8860e-02,  7.6350e-01, -3.0951e-01,  4.1892e-01,
        -2.4318e-01, -5.1777e-01, -1.5684e-02,  9.5078e-01,  1.0614e+00,
        -9.5549e-01,  5.8776e-01, -3.5192e-01,  6.5726e-01, -1.5130e-01,
        -1.9157e-01,  2.0985e-01, -5.9012e-02,  4.4116e-02, -7.2913e-01,
         8.7791e-02, -1.9375e-01, -3.6614e-01, -9.2353e-02,  1.0098e-01,
         4.3963e-01, -3.3074e-01,  3.5177e-01,  3.8898e-01,  2.2936e-01,
         3.5858e-01, -4.9083e-01, -4.0876e-01, -9.0678e-02, -7.0470e-01,
         2.7033e-01,  1.8989e-01,  6.3333e-01, -1.5572e-01,  6.5873e-01,
        -7.5316e-01, -3.0902e-01,  6.6033e-02,  1.7893e-01, -6.7201e-01,
        -8.8734e-02, -1.1826e-01, -4.3199e-01, -6.4221e-01,  4.1153e-02,
        -5.2451e-01, -1.0088e-01, -4.0775e-02,  3.9120e-01, -6.5376e-01,
         3.2221e-02,  3.6288e-01, -8.9282e-02, -1.1490e-02,  7.4862e-01,
        -7.4064e-01,  1.7449e-01, -7.5405e-01,  6.4146e-01,  1.0291e+00,
         2.8847e-01,  5.2673e-01, -4.9626e-01, -1.9954e-01,  1.3216e-01,
         1.8825e-01, -4.7657e-01,  4.2282e-01, -4.0568e-01,  2.3722e-02,
         5.4920e-01, -1.4587e-01, -3.5770e-01, -1.9939e-01, -1.1731e+00,
         5.3839e-01,  3.3669e-01,  1.6355e-01,  3.5891e-01, -1.2282e-01,
        -2.3226e-01, -6.8171e-01, -8.4709e-01,  3.1565e-01,  1.1788e-01,
         2.7053e-01,  2.9246e-01, -8.2972e-01,  4.2982e-01, -1.3717e-01,
         1.0891e+00,  9.6568e-01,  1.6599e-01, -2.5408e-01, -2.4714e-01,
        -1.3623e-01,  3.1174e-02,  1.8920e-01, -3.6764e-01,  3.1379e-02,
         6.7164e-01,  3.9830e-01, -6.0101e-02, -4.3713e-02, -3.1704e-01,
         3.3430e-01,  3.3288e-01, -4.7438e-01,  1.7393e-01,  1.0775e-02,
        -2.7040e-01, -2.2331e-01, -4.2336e-03, -1.6390e-01,  3.4489e-01,
        -2.1620e-01,  2.5920e-01, -7.0687e-01,  1.1055e-01, -6.6482e-01,
        -4.5127e-02, -2.8191e-02,  2.7641e-01, -7.9412e-01, -1.8351e-01,
         2.4231e-01, -4.4055e-02,  2.8127e-01, -1.0536e-01, -3.0405e-01,
         2.9168e-01,  2.9783e-01, -3.3719e-01, -4.6399e-01,  4.1250e-01,
        -2.2121e-01,  1.8398e-02,  2.4042e-02,  1.2196e+00,  6.2152e-01,
        -2.9397e-01, -1.3755e-01,  9.6192e-01,  1.2938e-02,  3.9612e-01,
        -3.4881e-01, -4.0311e-01,  7.7816e-02, -5.7262e-01,  7.3690e-01,
        -2.3015e-01, -2.3768e-02, -3.0926e-01, -5.7969e-01,  1.7674e-01,
        -8.6507e-02,  4.7539e-01,  5.6689e-02, -2.1775e-02, -5.1129e-01,
        -6.7515e-01,  1.3208e-01, -7.8015e-01,  3.8143e-01,  2.0680e-01,
        -1.7086e-01, -4.4747e-02,  1.0118e+00, -1.0206e-01,  1.8513e-01,
        -7.3474e-03,  3.0256e-01,  2.4061e-01,  1.0566e+00,  1.1465e-01,
         3.6376e-02, -4.1803e-01,  7.6873e-02,  5.0526e-01, -9.6579e-02,
        -2.6844e-01, -2.6945e-01, -4.2376e-01,  2.6483e-01, -3.4249e-01,
        -1.5059e-01, -3.8035e-01, -2.3503e-01,  7.3696e-01,  2.2822e-01,
         1.2733e-01, -1.7033e-02, -3.7245e-01, -2.9838e-01, -6.3784e-01,
         3.3366e-01, -3.4834e-01, -2.3252e-01,  3.6221e-02,  5.2631e-01,
         2.2934e-01, -1.8480e-01, -3.9684e-01, -3.2412e-01, -4.3245e-01,
         5.5092e-01,  3.3251e-01, -3.0288e-01, -6.0032e-01,  8.7526e-01,
         2.3209e-01, -3.4428e-01, -2.4560e-01,  1.6642e-01, -1.8395e-01,
        -2.1557e-01, -4.2033e-01, -1.3947e-01,  2.8193e-01,  2.5779e-01,
         2.1791e-01,  3.7339e-01, -3.6503e-01, -1.3434e-01, -4.6796e-01,
        -1.6872e-03, -5.0013e-01, -2.2037e-01, -1.4587e-01, -1.7420e-01,
         2.0227e-01,  3.4999e-01, -3.2873e-01, -2.3834e-01, -3.0599e-01,
         4.5744e-01, -2.5530e-01,  2.8697e-01,  7.4247e-02,  3.3911e-01,
        -4.2706e-01, -3.1845e-01,  3.9458e-02,  2.2810e-02,  3.9337e-01,
        -5.5479e-01,  2.5660e-01, -4.7753e-01,  8.4380e-02, -1.6748e-01,
        -5.7502e-01, -1.3180e+00, -1.2038e-01,  1.4395e-01, -2.6719e-02,
         1.3802e-01, -1.8562e-01,  2.4056e-01,  6.9039e-02, -1.4829e-01,
        -1.1776e-02, -2.0422e-01,  5.3102e-02, -6.4926e-02,  3.0017e-01,
         2.9637e-01,  3.9143e-01,  8.2873e-02,  3.0298e-01, -1.1868e-01,
         5.3025e-02, -4.6890e-02, -4.6998e-01, -2.8341e-01, -7.4492e-01,
         1.4738e-01, -1.9054e-01,  8.0130e-01, -1.4795e-01,  5.2046e-01,
        -4.2863e-01, -5.0698e-03, -1.4039e-02,  2.1895e-01, -2.7498e-02,
        -3.8656e-01,  7.7652e-02,  8.3478e-02, -4.2660e-01,  2.3145e-01,
        -7.5428e-01, -5.9487e-01, -1.5017e-01, -3.5440e-02,  6.2617e-02,
         3.4681e-01, -2.3807e-02, -3.8301e-02, -1.0956e-01,  1.9523e-01,
        -3.9468e-01,  1.1397e-01,  5.6755e-01, -1.9457e-01, -8.8296e-01,
         1.3403e+00,  1.0307e+00, -1.6420e-01, -3.7464e-01, -2.2938e-01,
        -5.0393e-01,  8.4092e-01, -4.1213e-01,  3.9970e-01, -5.1796e-01,
        -1.2779e+00,  1.7706e-01, -6.1882e-01,  2.8533e-01,  4.8424e-02,
         1.4162e-01,  1.9445e-01, -4.3570e-01, -3.6234e-01,  3.2883e-01,
        -5.4550e-01, -8.2578e-02,  3.1610e-01,  5.8394e-02, -3.0672e-02,
         1.7845e+00, -7.4955e-01, -4.0991e-01,  1.7082e-01, -2.5463e-01,
         6.1270e-01, -6.4119e-01, -5.7561e-01,  3.0672e-01,  4.5866e-01,
        -1.7229e-02,  9.0807e-02,  7.5503e-01, -1.3342e-01,  2.6080e-01,
         7.5584e-01, -3.0868e-01, -1.5768e+00,  8.6768e-02, -4.4540e-01,
        -3.5387e-03, -1.6549e-01,  1.0163e-01,  2.8272e-01, -4.2269e-01,
        -7.1709e-01, -3.4202e-01, -4.9366e-01, -8.7904e-01,  3.6445e-01,
         3.5460e-01, -3.8504e-01,  1.2271e-01, -7.7151e-01,  4.6544e-01,
        -1.8396e-03, -9.2312e-03, -9.1516e-01, -4.2211e-02, -1.7639e-01,
        -7.0991e-02, -2.8632e-01, -3.2595e-01,  1.3589e-01, -1.3778e-01,
         1.3151e-01,  3.0055e-01, -1.8168e-01, -1.9541e-02,  3.7024e-03,
         1.9515e-01, -3.1181e-02,  3.8267e-02,  1.2688e-01, -1.2018e-02,
        -1.0305e-01,  2.2535e-01, -1.0444e-01,  3.1130e-02, -8.0197e-02,
         6.7364e-02,  1.0957e-01, -8.8075e-02, -2.9841e-02, -2.9269e-02,
         8.5656e-02,  9.9645e-02,  2.0161e-03, -1.9431e-01, -1.0320e-01,
         3.4250e-02,  3.4574e-01,  2.2657e-01, -4.6352e-01,  3.1559e-01,
        -3.5699e-01, -3.2080e-01,  7.3675e-02, -2.0672e-01,  2.5707e-01,
         3.3393e-02,  1.2496e-01, -2.7032e-01,  6.0769e-01,  1.1883e-01,
         3.4694e-01, -1.4502e-01, -3.8020e-01,  5.9709e-01,  9.7147e-02,
         3.0965e-01, -2.6264e-01,  2.1767e-01, -1.7506e-01,  1.2270e-01,
         1.4878e-02,  2.3382e-01,  2.3965e-01, -1.8823e-01, -1.1409e-01,
         5.7178e-01, -3.5686e-01, -3.8646e-01,  1.0922e-01, -3.5658e-02,
         2.3627e-01, -5.1865e-01,  7.0912e-01, -2.1188e-01, -3.9505e-01,
        -6.2161e-01, -4.4976e-01,  3.5379e-01, -1.4883e-01,  5.2099e-01,
         6.6077e-01, -5.6015e-01, -2.5688e-01, -6.7990e-01, -7.2501e-02,
        -8.8815e-01,  5.3693e-01, -3.4321e-01,  2.4931e-02, -2.5817e-02,
         7.4951e-01, -2.6671e-01, -2.7413e-01, -4.5844e-02,  2.3636e-01,
         1.0319e-02,  8.1633e-01,  3.3539e-01,  1.9528e-01, -5.7168e-01,
        -7.7557e-01,  3.3907e-01,  1.2387e-01, -1.9255e-01,  6.1984e-01,
        -1.1981e-01, -3.8751e-02,  1.9073e-01,  5.3039e-01,  3.8239e-01,
         1.3108e-01,  4.7853e-01, -6.5451e-01, -1.5465e-01, -1.4710e-01,
         1.5585e-01, -4.2158e-02, -2.7893e-01, -7.4151e-01,  3.8108e-01,
        -1.8793e-01, -1.7573e-01,  8.9161e-01, -2.5160e-02,  2.7951e-01,
         7.7533e-01, -1.8024e+00,  8.5375e-02,  1.9248e-02;
bias2 << -0.1806, -0.0023, -0.5271, -0.4584,  0.5705,  0.4739,  0.8786, -0.5049,
        -0.3605,  0.2106, -0.3656,  0.6040,  0.3869,  0.0619, -1.0421, -0.2997,
         1.1641, -0.8961,  0.1825,  0.5571, -0.6393,  1.0069, -0.5345,  0.3645,
         0.1715, -0.1309,  0.0577, -0.8600,  0.2297,  0.4126,  0.3760,  0.4481;
weight4 <<  3.0199e-01, -2.8148e-01, -8.1423e-01,  7.0867e-01,  4.4414e-01,
         2.2833e-01, -1.0776e+00,  1.0075e+00,  1.3883e-01,  2.3721e-01,
        -1.1558e+00,  1.0374e-01, -2.1664e-01, -2.1244e-02,  8.3336e-01,
         7.6976e-01,  6.4475e-01,  3.6960e-01, -2.7935e-01, -1.0228e+00,
         7.5650e-01, -9.3597e-01, -1.7841e-01,  6.1545e-01, -1.1475e-01,
        -2.5076e-01,  1.2943e+00, -7.7956e-01, -1.7868e-01, -8.9795e-01,
        -3.3827e-01,  1.0137e-01,  1.2036e+00,  1.4599e+00,  9.9072e-03,
         7.7966e-03, -6.8626e-01,  1.2845e+00, -3.4560e-01, -3.2169e-01,
        -1.0039e+00,  3.8250e-01, -8.9372e-02,  2.0852e-01, -1.2248e+00,
        -1.1118e+00,  2.4272e-01, -1.0056e+00,  6.7093e-01, -3.9125e-02,
        -5.4679e-01,  1.0092e-01,  5.9122e-01, -7.5186e-02,  5.5802e-02,
        -4.4863e-01, -8.1113e-01, -3.2056e-01, -2.1569e-01,  6.5527e-01,
         1.1537e-02, -2.5968e-02, -2.0943e-01,  1.4675e+00,  7.1848e-02,
         3.3348e-02,  1.1090e+00,  2.0411e-02,  6.1565e-03, -2.6860e-03,
        -1.1590e-02,  7.7723e-02, -3.0767e-02, -1.8143e-02, -5.0464e-02,
        -1.0159e-01,  1.1277e-03, -2.4642e-02,  6.5563e-03, -1.9430e-02,
        -2.6249e-02, -7.6370e-02,  8.1061e-02, -1.0337e-02, -4.1041e-03,
        -8.7849e-02, -6.0009e-02, -1.4930e-01,  3.5052e-02,  5.2599e-03,
         6.6747e-02, -2.0396e-02,  8.9915e-01,  9.6542e-02,  3.3653e-02,
        -1.8402e-02,  4.2016e-01,  1.6035e-01, -2.4149e-01, -6.2586e-01,
        -4.2187e-01, -6.4637e-02,  8.6995e-01, -4.2317e-01,  4.6097e-01,
         1.4702e+00, -9.4706e-01, -8.6498e-01,  2.2454e-01, -1.4527e-01,
         3.0911e-01, -4.8967e-01, -5.7136e-01,  9.4645e-01,  3.1346e-01,
         6.9174e-01, -6.1886e-01,  4.0283e-01, -1.2344e+00, -9.7919e-01,
        -3.3807e-02,  1.3566e+00, -1.4428e+00,  5.3357e-01,  1.1280e-01,
         1.2749e+00,  1.1695e+00, -5.1690e-02;
bias4 <<  0.1712,  0.1674, -0.7571,  0.8526;
out_mean << 25.614948891346508, -0.06273709346857953, 602.3649405521444, -3.639064063675181;
out_std << 163.9878472541439, 261.22318171724066, 329.62565507072793, 10.900411434590092;
in_mean << 0.050013480030594526, -0.0001507066126386099, 0.0007388024854837143, 60.028427422600004, 1999.9303124040005, 0.8001799666020003, 0.09995742891702171, 0.3489849735030001;
in_std << 0.02887507434905162, 1.1551599929798697, 0.9066816123489984, 23.097802996273668, 865.6642938090355, 0.2887791583966398, 0.05772827666609045, 0.10074196292554011;
}
