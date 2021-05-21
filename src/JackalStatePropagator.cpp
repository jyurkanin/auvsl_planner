#include <ros/ros.h>
#include "JackalStatePropagator.h"
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

  JackalDynamicSolver solver;

  //ROS_INFO("Control %.2f %.2f", control_vector[0], control_vector[1]);

  unsigned vehicle_state_len = 21;
  float x_start[21];
  float x_end[21];

  for(unsigned i = 0; i < vehicle_state_len; i++){
    x_start[i] = 0;
    x_end[i] = 0;
  }

  ROS_INFO("Before 13 %f", val[13]);
  convert_to_model_space(val, x_start);

  float Vf = control_vector[1];//GlobalParams::get_fuzzy_constant_speed();
  float base_width = solver.base_size[0];
  float vl = (Vf - control_vector[0]*(base_width/2.0))/solver.tire_radius;
  float vr = (Vf + control_vector[0]*(base_width/2.0))/solver.tire_radius;

  solver.solve(x_start, x_end, vl, vr, (float) duration);

  convert_to_planner_space(result_val, x_end);
  ROS_INFO("Before 13 %f", val[13]);

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


void JackalStatePropagator::getWaypoints(std::vector<ompl::control::Control*> &controls, std::vector<double> &durations, std::vector<ompl::base::State*> states, std::vector<Vector2d> &waypoints, unsigned &num_waypoints){
  const double* val = states[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  waypoints.clear();

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

  waypoints.push_back(Vector2d(x_start[0], x_start[1]));


  float Vf;
  float W;
  float base_width = solver.base_size[0];
  float vl;
  float vr;
  float timestep = GlobalParams::get_propagation_step_size();
  double result_val[17];

  unsigned idx = 1;
  float eps = 1e-5;
  for(unsigned i = 0; i < controls.size(); i++){
    const double *control_vector = controls[i]->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double* val = states[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    const double* expected_val = states[i+1]->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    convert_to_model_space(val, x_start);

    Vf = control_vector[1];
    W = control_vector[0];
    vl = (Vf - W*(base_width/2.0))/solver.tire_radius;
    vr = (Vf + W*(base_width/2.0))/solver.tire_radius;


    //ROS_INFO("vl vr %f %f", vl, vr);

    ROS_INFO("Control Vf Wz   %f %f", Vf, W);

    for(int k = 0; k*timestep < durations[i]; k++){
      solver.solve(x_start, x_end, vl, vr, timestep); //durations[i]);

      for(int j = 0; j < vehicle_state_len; j++){
        x_start[j] = x_end[j];
      }

      waypoints.push_back(Vector2d(x_end[0], x_end[1]));
    }

    convert_to_planner_space(result_val, x_end);

    /*
    for(int j = 0; j < 6; j++){
      if(fabs(expected_val[j] - result_val[j]) > 1e-5){
        ROS_INFO("Divergence after Control %d", i);
        ROS_INFO("expected val\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f", expected_val[0], expected_val[1], expected_val[2], expected_val[3], expected_val[4], expected_val[5], expected_val[6], expected_val[7], expected_val[8], expected_val[9]);
        ROS_INFO("actual   val\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f", result_val[0], result_val[1], result_val[2], result_val[3], result_val[4], result_val[5], result_val[6], result_val[7], result_val[8], result_val[9]);
        ROS_INFO(" ");
        break;
      }
    }
    */
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
