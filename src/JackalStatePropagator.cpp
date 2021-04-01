#include <ros/ros.h>
#include "JackalStatePropagator.h"
#include <stdio.h>

JackalStatePropagator::JackalStatePropagator(ompl::control::SpaceInformationPtr si) : StatePropagator(si){
  JackalDynamicSolver::init_model(0);
}

JackalStatePropagator::~JackalStatePropagator(){
  JackalDynamicSolver::del_model();
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
  
  x_start[0] = (float) val[0];            //x
  x_start[1] = (float) val[1];            //y
  x_start[5] = sinf((float)val[2]/2.0f);  //qz
  x_start[10] = cosf((float)val[2]/2.0f); //qw
  x_start[11] = (float) val[3];
  x_start[12] = (float) val[4];
  x_start[16] = (float) val[5];
  
  float Vf = control_vector[1];//GlobalParams::get_fuzzy_constant_speed();
  float base_width = solver.base_size[0];
  float vl = (Vf - control_vector[0]*(base_width/2.0))/solver.tire_radius;
  float vr = (Vf + control_vector[0]*(base_width/2.0))/solver.tire_radius;
  
  ROS_INFO("Vl Vr %f %f", vl, vr);

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
  
  
  solver.solve(x_start, x_end, vl, vr, (float) duration);
    
  result_val[0] = x_end[0];
  result_val[1] = x_end[1];
  result_val[2] = fmod((2*M_PI) + (2*atan2f(x_end[5], x_end[10])), 2*M_PI);
  result_val[3] = x_end[11];
  result_val[4] = x_end[12];
  result_val[5] = x_end[16];
  

  //ROS_INFO("After State %f %f %f %f %f %f", result_val[0], result_val[1], result_val[2], result_val[3], result_val[4], result_val[5]);
  
}


void JackalStatePropagator::getWaypoints(std::vector<ompl::control::Control*> &controls, std::vector<double> &durations, std::vector<ompl::base::State*> states, float *waypoints){
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
  
  x_start[0] = (float) val[0];       //x
  x_start[1] = (float) val[1];       //y
  x_start[5] = sinf((float)val[2]/2.0);  //qz
  x_start[10] = cosf((float)val[2]/2.0); //qw
  x_start[11] = (float) val[3];
  x_start[12] = (float) val[4];
  x_start[16] = (float) val[5];

  waypoints[(2*0) + 0] = val[0];
  waypoints[(2*0) + 1] = val[1];
  
  float Vf;
  float W;
  float base_width = solver.base_size[0];
  float vl;
  float vr;
  
  float timestep = GlobalParams::get_propagation_step_size();

  float result_val[6];

  unsigned idx = 1;
  float eps = 1e-5;
  for(unsigned i = 0; i < controls.size(); i++){
    const double *control_vector = controls[i]->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double* val = states[i+1]->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    
    Vf = control_vector[1];
    W = control_vector[0];
    vl = (Vf - W*(base_width/2.0))/solver.tire_radius;
    vr = (Vf + W*(base_width/2.0))/solver.tire_radius;
    
    solver.solve(x_start, x_end, vl, vr, durations[i]);//timestep);
    
    for(int j = 0; j < vehicle_state_len; j++){
      ROS_INFO("%d %f", j, x_end[j]);
      x_start[j] = x_end[j];
    }
    
    waypoints[(2*idx) + 0] = x_end[0];
    waypoints[(2*idx) + 1] = x_end[1];
    
    result_val[0] = x_end[0];
    result_val[1] = x_end[1];
    result_val[2] = fmod((2*M_PI) + (2*atan2f(x_end[5], x_end[10])), 2*M_PI);
    result_val[3] = x_end[11];
    result_val[4] = x_end[12];
    result_val[5] = x_end[16];
    
    idx++;
    
    for(int j = 0; j < 6; j++){
      if(fabs(val[j] - result_val[j]) > 1e-5){
        ROS_INFO("Index %d    Controls Vf: %f  w: %f     Duration %f    Vl Vr %f %f", i, Vf, control_vector[0], durations[i], vl, vr);
        ROS_INFO("Planned %.2f %.2f %.2f %.2f %.2f %.2f", val[0], val[1], val[2], val[3], val[4], val[5]);
        ROS_INFO("Actual  %.2f %.2f %.2f %.2f %.2f %.2f\n", result_val[0], result_val[1], result_val[2], result_val[3], result_val[4], result_val[5]);
        break;
      }
    }
  }
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




