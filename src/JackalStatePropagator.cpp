#include "JackalStatePropagator.h"
#include <stdio.h>

JackalStatePropagator::JackalStatePropagator(ompl::control::SpaceInformationPtr si) : StatePropagator(si){
  JackalDynamicSolver::init_model(0);
}

void JackalStatePropagator::propagate(const ompl::base::State *state, const ompl::control::Control *control, double duration, ompl::base::State *result) const{
  const double* val = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  double* result_val = result->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  
  const double *control_vector = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
  
  JackalDynamicSolver solver;
  
                           
  unsigned vehicle_state_len = 21;
  float x_start[21];
  float x_end[21];
  
  for(unsigned i = 0; i < vehicle_state_len; i++){
    x_start[0] = 0;
    x_end[0] = 0;
  }
  
  x_start[0] = (float) val[0];       //x
  x_start[1] = (float) val[1];       //y
  x_start[5] = sinf((float)val[2]);  //qz
  x_start[10] = cosf((float)val[2]); //qw
  x_start[11] = (float) val[3];
  x_start[12] = (float) val[4];
  x_start[16] = (float) val[5];
  
  
  float Vf = GlobalParams::get_fuzzy_constant_speed();
  float base_width = solver.base_size[0];
  float vl = Vf - control_vector[0]*(base_width/2.0);
  float vr = Vf + control_vector[0]*(base_width/2.0);

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
  result_val[2] = M_PI + atan2f(x_end[5], x_end[10]);
  result_val[3] = x_end[11];
  result_val[4] = x_end[12];
  result_val[5] = x_end[16];
  
  
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




