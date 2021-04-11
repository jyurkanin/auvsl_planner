#include <ros/ros.h>
#include <rbdl/rbdl.h>
#include "JackalStatePropagator.h"
#include <stdio.h>

JackalStatePropagator::JackalStatePropagator(ompl::control::SpaceInformationPtr si) : StatePropagator(si){
  JackalDynamicSolver::init_model(0);
}

JackalStatePropagator::~JackalStatePropagator(){
  JackalDynamicSolver::del_model();
}



void JackalStatePropagator::convert_to_vehicle_space(double *vehicle_space, const float *model_space){
  vehicle_space[0] = model_space[0];
  vehicle_space[1] = model_space[1];
  vehicle_space[2] = fmod((2*M_PI) + (2*atan2f(model_space[5], model_space[10])), 2*M_PI);
  vehicle_space[3] = model_space[11];
  vehicle_space[4] = model_space[12];
  vehicle_space[5] = model_space[16];
}

void JackalStatePropagator::convert_to_model_space(const double *vehicle_space, float *model_space){
  model_space[0] = (float) vehicle_space[0];            //x
  model_space[1] = (float) vehicle_space[1];            //y
  model_space[5] = sinf((float)vehicle_space[2]/2.0f);  //qz
  model_space[10] = cosf((float)vehicle_space[2]/2.0f); //qw
  model_space[11] = (float) vehicle_space[3];
  model_space[12] = (float) vehicle_space[4];
  model_space[16] = (float) vehicle_space[5];
}

Vector3d JackalStatePropagator::get_base_velocity(float *Xout){
  Quaternion quat(Xout[3], Xout[4], Xout[5], Xout[10]);
  Vector3d r(Xout[0], Xout[1], Xout[2]);
  
  SpatialTransform X_base1(Quaternion(0,0,0,1).toMatrix(), r);
  SpatialTransform X_base2(quat.toMatrix(), r);
  
  SpatialVector body_vel_lin(0,0,0,Xout[11],Xout[12],Xout[13]);
  SpatialVector body_vel_ang(Xout[14],Xout[15],Xout[16],0,0,0);
  
  SpatialVector base_vel = X_base2.inverse().apply(body_vel_ang) + X_base1.inverse().apply(body_vel_lin);

  Vector3d temp = (VectorCrossMatrix(Vector3d(base_vel[0], base_vel[1], base_vel[2])) * Vector3d(Xout[0], Xout[1], Xout[2]));  
  return Vector3d(base_vel[3], base_vel[4], base_vel[5]) + temp;
}






void JackalStatePropagator::propagate(const ompl::base::State *state, const ompl::control::Control *control, double duration, ompl::base::State *result) const{
  const double* val = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  double* result_val = result->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  
  const double *control_vector = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
  
  JackalDynamicSolver solver;

  //ROS_INFO("Control %.2f %.2f", control_vector[0], control_vector[1]);
                           
  unsigned vehicle_state_len = 21;
  float x_start[21];
  float x_end[21];
  
  for(unsigned i = 0; i < vehicle_state_len; i++){
    x_start[i] = 0;
    x_end[i] = 0;
  }

  convert_to_model_space(val, x_start);
  
  
  float Vf = control_vector[1];//GlobalParams::get_fuzzy_constant_speed();
  float base_width = solver.base_size[0];
  float vl = (Vf - control_vector[0]*(base_width/2.0))/solver.tire_radius;
  float vr = (Vf + control_vector[0]*(base_width/2.0))/solver.tire_radius;
  
  solver.solve(x_start, x_end, vl, vr, (float) duration);
  
  convert_to_vehicle_space(result_val, x_end);
  
  //ROS_INFO("Vl Vr %f %f", vl, vr);

  /*
  float dt = .01;
  result_val[0] = val[0];
  result_val[1] = val[1];
  result_val[2] = val[2];
  for(unsigned i = 0; (i*dt) < duration; i++){
    //Xd
    result_val[3] = Vf*cosf(result_val[2]);
    result_val[4] = Vf*sinf(result_val[2]);
    result_val[5] = control_vector[0];
    //X
    result_val[0] += dt*result_val[3];
    result_val[1] += dt*result_val[4];
    result_val[2] += dt*result_val[5];
  }
  */
  
  
}


void JackalStatePropagator::getWaypoints(std::vector<ompl::control::Control*> &controls, std::vector<double> &durations, std::vector<ompl::base::State*> states, std::vector<float> &waypoints, unsigned &num_waypoints){
  const double* val = states[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  JackalDynamicSolver::init_model(2);
  JackalDynamicSolver solver;
                           
  unsigned vehicle_state_len = 21;
  float x_start[21];
  float x_end[21];
  
  for(unsigned i = 0; i < vehicle_state_len; i++){
    x_start[i] = 0;
    x_end[i] = 0;
  }
  
  convert_to_model_space(val, x_start);
  
  waypoints.push_back(x_start[0]);
  waypoints.push_back(x_start[1]);
  
  
  float Vf;
  float W;
  float base_width = solver.base_size[0];
  float vl;
  float vr;
  float timestep = GlobalParams::get_propagation_step_size();
  double result_val[6];

  unsigned idx = 1;
  float eps = 1e-5;
  for(unsigned i = 0; i < controls.size(); i++){
    const double *control_vector = controls[i]->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double* val = states[i+1]->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    
    Vf = control_vector[1];
    W = control_vector[0];
    vl = (Vf - W*(base_width/2.0))/solver.tire_radius;
    vr = (Vf + W*(base_width/2.0))/solver.tire_radius;

    for(int k = 0; k*timestep < durations[i]; k++){
      solver.solve(x_start, x_end, vl, vr, timestep); //durations[i]);
    
      for(int j = 0; j < vehicle_state_len; j++){
        x_start[j] = x_end[j];
      }

      waypoints.push_back(x_end[0]);
      waypoints.push_back(x_end[1]);      
    }
    
    convert_to_vehicle_space(result_val, x_end);
        
    for(int j = 0; j < 6; j++){
      if(fabs(val[j] - result_val[j]) > 1e-5){
        break;
      }
    }
  }

  num_waypoints = (waypoints.size())/2;
}



bool JackalStatePropagator::steer(const ompl::base::State *from, const ompl::base::State *to, ompl::control::Control *result, double &duration) const{
  printf("Steer Function Not Implemented\n"); //Should never happen.
  return false;
}

bool JackalStatePropagator::canPropagateBackward() const{
  return false;
}

bool JackalStatePropagator::canSteer() const{ //Steering means, can we drive exactly from one state to the next and compute the control necessary to do it. I think.
  return false;
}




