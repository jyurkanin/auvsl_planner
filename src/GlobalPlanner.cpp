#include <iostream>
#include <thread>
#include <stdlib.h>
#include <memory>

#include "GlobalPlanner.h"
#include "JackalStatePropagator.h"
#include "GlobalParams.h"
#include "PlannerVisualizer.h"
#include "VehicleControlSampler.h"
#include "DirectedVehicleControlSampler.h"

#include <ompl/util/RandomNumbers.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>



//launch-prefix="gdb -ex run --args


/*
 * State of dynamic vehicle is actually:
 * [x,y,z,qx,qy,qz,q1,q2,q3,q4,qw,vx,vy,vz,wx,wy,wz,qd1,qd2,qd3,qd4]
 *
 * Reduced state for planner is going to be:
 * [x, y, theta, vx, vy, wz]
 *
 * Vx,Vy,Wz are important, but x and y are going to be most important
 * for determining distance between states
 *
 */

//statics
const TerrainMap *GlobalPlanner::global_map_;


ompl::control::DirectedControlSamplerPtr allocCustomDirectedControlSampler(const ompl::control::SpaceInformation *si){
  return std::make_shared<DirectedVehicleControlSampler>(si, GlobalParams::get_num_control_samples());
}


ompl::control::ControlSamplerPtr allocCustomControlSampler(const ompl::control::ControlSpace *cspace){
  return std::make_shared<VehicleControlSampler>(cspace);
}




GlobalPlanner::GlobalPlanner(const TerrainMap* terrain_map){
  global_map_ = terrain_map;

  ompl::base::VehicleStateSpace *space = new ompl::base::VehicleStateSpace(17);
  ompl::base::RealVectorBounds bounds(17);

  bounds.setLow(0, -100); bounds.setHigh(0, 100); //x
  bounds.setLow(1, -100); bounds.setHigh(1, 100); //y
  bounds.setLow(2, -100); bounds.setHigh(2, 100); //z

  bounds.setLow(3, -1); bounds.setHigh(3, 1); //quaterion has to stay on the 4-ball, so its components max is 1, and min is -1
  bounds.setLow(4, -1); bounds.setHigh(4, 1);
  bounds.setLow(5, -1); bounds.setHigh(5, 1);
  bounds.setLow(6, -1); bounds.setHigh(6, 1);

  bounds.setLow(7, -100); bounds.setHigh(7, 100); //vx
  bounds.setLow(8, -100); bounds.setHigh(8, 100);
  bounds.setLow(9, -100); bounds.setHigh(9, 100);

  bounds.setLow(10, -100); bounds.setHigh(10, 100); //wx
  bounds.setLow(11, -100); bounds.setHigh(11, 100);
  bounds.setLow(12, -100); bounds.setHigh(12, 100);

  bounds.setLow(13, -100); bounds.setHigh(13, 100); //qd1
  bounds.setLow(14, -100); bounds.setHigh(14, 100);
  bounds.setLow(15, -100); bounds.setHigh(15, 100);
  bounds.setLow(16, -100); bounds.setHigh(16, 100);

  space->setBounds(bounds);

  space_ptr_.reset(space); //no me gusta shared ptrs
  
  ompl::control::RealVectorControlSpace *cspace = new ompl::control::RealVectorControlSpace(space_ptr_, 2);

  //cspace.setControlSamplerAllocator(allocCustomControlSampler);

  ompl::base::RealVectorBounds cbounds(2);

  cbounds.setLow(0, -GlobalParams::get_max_angular_vel());
  cbounds.setHigh(0, GlobalParams::get_max_angular_vel());

  cbounds.setLow(1, 0);
  cbounds.setHigh(1, GlobalParams::get_fuzzy_constant_speed());

  cspace->setBounds(cbounds);

  si_ = ompl::control::SpaceInformationPtr(new ompl::control::SpaceInformation(space_ptr_, ompl::control::ControlSpacePtr(cspace)));

  ompl::control::StatePropagatorPtr dynamic_model_ptr(new JackalStatePropagator(si_));
  si_->setStatePropagator(dynamic_model_ptr);
  si_->setPropagationStepSize(GlobalParams::get_propagation_step_size());
  si_->setMinMaxControlDuration(5, 10);

  si_->setDirectedControlSamplerAllocator(allocCustomDirectedControlSampler);

  si_->setStateValidityChecker(GlobalPlanner::isStateValid);
  si_->setStateValidityCheckingResolution(GlobalParams::get_state_checker_resolution());    //this is for checking motions
  si_->setup();

  pdef_ = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(si_));

  planner_ = new ompl::control::VehicleRRT(si_);
  planner_->setGoalBias(GlobalParams::get_goal_bias()); //.05 was recommended.
  planner_->setIntermediateStates(GlobalParams::get_add_intermediate_states());

  //G_TOLERANCE_ = GlobalParams::get_goal_tolerance();
}

bool GlobalPlanner::isStateValid(const ompl::base::State *state){
  const ompl::base::VehicleStateSpace::StateType& state_vector = *state->as<ompl::base::VehicleStateSpace::StateType>();

  //test for roll over
  RigidBodyDynamics::Math::Quaternion quat(state_vector[3], state_vector[4], state_vector[5], state_vector[6]);
  RigidBodyDynamics::Math::Vector3d vec = quat.rotate(RigidBodyDynamics::Math::Vector3d(0,0,1));
  if(vec[2] < 0){ //you could put a number slightly greater than zero here. But I'll leave it as zero for now.
    return false; //if the vehicle has rotated so the z axis of the body frame is pointing down in the world frame, then it fucked up
  }


  return global_map_->isStateValid(state_vector[0], state_vector[1]);
}


GlobalPlanner::~GlobalPlanner(){
    delete planner_;
}




int GlobalPlanner::plan(std::vector<RigidBodyDynamics::Math::Vector2d> &waypoints, float *vehicle_start_state, RigidBodyDynamics::Math::Vector2d goal_pos, float goal_tol){
  // construct the state space we are planning in

  ompl::base::ScopedState<> start(space_ptr_);
  for(int i = 0; i < 17; i++){
    start[i] = vehicle_start_state[i];
  }

  ompl::base::GoalSpace goal(si_);
  ompl::base::VehicleStateSpace gspace(17);
  ompl::base::RealVectorBounds gbounds(17);
  gbounds.setLow(0, goal_pos[0] - goal_tol);
  gbounds.setHigh(0, goal_pos[0] + goal_tol);

  gbounds.setLow(1, goal_pos[1] - goal_tol);
  gbounds.setHigh(1, goal_pos[1] + goal_tol);

  for(int i = 2; i < 17; i++){ //Goal region is only in x and y. Unbounded in other state variables.
    gbounds.setLow(i, -1000); gbounds.setHigh(i, 1000);
  }
  gspace.setBounds(gbounds);
  goal.setSpace(ompl::base::StateSpacePtr(&gspace));

  pdef_->addStartState(start);
  pdef_->setGoal((ompl::base::GoalPtr) &goal);

  planner_->setProblemDefinition(pdef_);

  
  PlannerVisualizer planner_visualizer(si_, (ompl::base::PlannerPtr) planner_, .5);
  planner_visualizer.setObstacles(global_map_->getObstacles());
  if(GlobalParams::get_visualize_planner()){
    planner_visualizer.startMonitor();
  }
  

  //float max_runtime = 600; //seconds
  float max_runtime = GlobalParams::get_max_gp_runtime();
  ompl::base::PlannerTerminationCondition ptc = ompl::base::plannerOrTerminationCondition(ompl::base::timedPlannerTerminationCondition(max_runtime), ompl::base::exactSolnPlannerTerminationCondition(pdef_));

  ompl::base::PlannerStatus solved = planner_->solve(ptc);

  if(solved){
    ompl::base::PlannerSolution soln = pdef_->getSolutions()[0];
    ompl::control::PathControl *path = soln.path_->as<ompl::control::PathControl>();

    std::vector<ompl::base::State*> states = path->getStates();
    std::vector<ompl::control::Control*> controls = path->getControls();
    std::vector<double> durations = path->getControlDurations();

    JackalStatePropagator dynamic_model(si_);

    unsigned num_waypoints;
    dynamic_model.getWaypoints(controls, durations, states, waypoints, num_waypoints);
    return EXIT_SUCCESS;
  }
  else{
    return EXIT_FAILURE;
  }

}
