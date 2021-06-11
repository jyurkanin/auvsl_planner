#include <ros/ros.h>
#include "JackalStatePropagator.h"
#include "auvsl_planner_node.h"
#include <stdio.h>



JackalStatePropagator::JackalStatePropagator(ompl::control::SpaceInformationPtr si) : StatePropagator(si){
      JackalDynamicSolver::init_model(0);
}

JackalStatePropagator::~JackalStatePropagator(){
    JackalDynamicSolver::del_model();
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


void JackalStatePropagator::convert_to_model_space(const double *planner_state, float *model_state){
  // 0 1 2   3  4  5    6  7  8  9    10   11 12 13   14 15 16   17  18  19  20
  // x,y,z,  qx,qy,qz,  q1,q2,q3,q4,  qw,  vx,vy,vz,  ax,ay,az,  qd1,qd2,qd3,qd4
  model_state[0] = planner_state[0];
  model_state[1] = planner_state[1];
  model_state[2] = planner_state[2];

  model_state[3] = planner_state[3];  //qx
  model_state[4] = planner_state[4];  //qy
  model_state[5] = planner_state[5];  //qz
  model_state[10] = planner_state[6]; //qw

  model_state[6] = 0;
  model_state[7] = 0;
  model_state[8] = 0;
  model_state[9] = 0;

  model_state[11] = planner_state[7]; //vx
  model_state[12] = planner_state[8]; //vy
  model_state[13] = planner_state[9]; //vz

  model_state[14] = planner_state[10]; //wx
  model_state[15] = planner_state[11]; //wy
  model_state[16] = planner_state[12]; //wz

  model_state[17] = planner_state[13]; //qd1
  model_state[18] = planner_state[14]; //qd2
  model_state[19] = planner_state[15]; //qd3
  model_state[20] = planner_state[16]; //qd4
}

void JackalStatePropagator::convert_to_planner_space(double *planner_state, const float *model_state){
  planner_state[0] = model_state[0]; //x
  planner_state[1] = model_state[1]; //y
  planner_state[2] = model_state[2]; //z

  planner_state[3] = model_state[3];  //qx
  planner_state[4] = model_state[4];  //qy
  planner_state[5] = model_state[5];  //qz
  planner_state[6] = model_state[10]; //qw

  planner_state[7] = model_state[11]; //vx
  planner_state[8] = model_state[12]; //vy
  planner_state[9] = model_state[13]; //vz

  planner_state[10] = model_state[14]; //wx
  planner_state[11] = model_state[15]; //wy
  planner_state[12] = model_state[16]; //wz

  planner_state[13] = model_state[17]; //qd1
  planner_state[14] = model_state[18]; //qd2
  planner_state[15] = model_state[19]; //qd3
  planner_state[16] = model_state[20]; //qd4
}



void JackalStatePropagator::propagate(const ompl::base::State *state, const ompl::control::Control *control, double duration, ompl::base::State *result) const{
  const double* val = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  double* result_val = result->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  const double *control_vector = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
  
  float x_start[21];
  float x_end[21];
  
  JackalDynamicSolver solver;
  convert_to_model_space(val, x_start);
  
  float Vf = control_vector[1];//GlobalParams::get_fuzzy_constant_speed();
  float base_width = solver.base_size[0];
  float vl = (Vf - control_vector[0]*(base_width/2.0))/solver.tire_radius;
  float vr = (Vf + control_vector[0]*(base_width/2.0))/solver.tire_radius;
  
  solver.solve(x_start, x_end, vl, vr, (float) duration);
  
  convert_to_planner_space(result_val, x_end);
}


void JackalStatePropagator::getWaypoints(std::vector<ompl::control::Control*> &controls, std::vector<double> &durations, std::vector<ompl::base::State*> states, std::vector<Vector2d> &waypoints, unsigned &num_waypoints){
  waypoints.clear();

  JackalDynamicSolver::init_model(2);
  JackalDynamicSolver solver;

  unsigned vehicle_state_len = 21;
  
  ompl::base::State *start_state = si_->allocState();
  ompl::base::State *result_state = si_->allocState();

  si_->copyState(start_state, states[0]);
  const double* start_val = start_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  waypoints.push_back(Vector2d(start_val[0], start_val[1]));
  
  for(unsigned i = 0; i < controls.size(); i++){
    const double *control_vector = controls[i]->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    
    propagate(start_state, controls[i], durations[i], result_state);
    si_->copyState(start_state, result_state);

    const double* result_val = result_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    waypoints.push_back(Vector2d(result_val[0], result_val[1]));
  }
  
  num_waypoints = waypoints.size();
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
