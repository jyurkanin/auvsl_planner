#include <iostream>
#include <thread>
#include <stdlib.h>
#include <memory>
#include <math.h>

#include "GlobalPlanner.h"
#include "JackalStatePropagator.h"
#include "GlobalParams.h"
#include "PlannerVisualizer.h"
#include "VehicleControlSampler.h"
#include "DirectedVehicleControlSampler.h"

#include <ompl/util/RandomNumbers.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <assert.h>


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
ompl::base::StateSpacePtr GlobalPlanner::space_ptr_;

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

  bounds.setLow(3, -1.01); bounds.setHigh(3, 1.01); //quaterion has to stay on the unit 4-ball, so its components max is 1, and min is -1
  bounds.setLow(4, -1.01); bounds.setHigh(4, 1.01); //The .01 is like an epsilon.
  bounds.setLow(5, -1.01); bounds.setHigh(5, 1.01);
  bounds.setLow(6, -1.01); bounds.setHigh(6, 1.01);

  for(unsigned i = 7; i < 17; i++){
      bounds.setLow(i, -2000);
      bounds.setHigh(i, 2000); //vx
  }

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

  dynamic_model_ptr_ = ompl::control::StatePropagatorPtr(new JackalStatePropagator(si_));

  si_->setStatePropagator(dynamic_model_ptr_);
  si_->setPropagationStepSize(GlobalParams::get_propagation_step_size());
  si_->setMinMaxControlDuration(5, 10);

  si_->setDirectedControlSamplerAllocator(allocCustomDirectedControlSampler);

  si_->setStateValidityChecker(GlobalPlanner::isStateValid);
  si_->setStateValidityCheckingResolution(GlobalParams::get_state_checker_resolution());    //this is for checking motions
  si_->setup();

  pdef_ = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(si_));

  ompl::control::VehicleRRT *rrt_planner = new ompl::control::VehicleRRT(si_);
  rrt_planner->setGoalBias(GlobalParams::get_goal_bias()); //.05 was recommended.
  rrt_planner->setIntermediateStates(GlobalParams::get_add_intermediate_states());

  planner_ = ompl::base::PlannerPtr(rrt_planner);
  //G_TOLERANCE_ = GlobalParams::get_goal_tolerance();
}

bool GlobalPlanner::isStateValid(const ompl::base::State *state){
  const ompl::base::VehicleStateSpace::StateType& state_vector = *state->as<ompl::base::VehicleStateSpace::StateType>();

  //test for roll over
  RigidBodyDynamics::Math::Quaternion quat(state_vector[3], state_vector[4], state_vector[5], state_vector[6]);
  RigidBodyDynamics::Math::Vector3d vec = quat.rotate(RigidBodyDynamics::Math::Vector3d(0,0,1));
  if(vec[2] < 0){ //you could put a number slightly greater than zero here. But I'll leave it as zero for now.
    ROS_INFO("INVALID STATE: ROllOVER");
    return false; //if the vehicle has rotated so the z axis of the body frame is pointing down in the world frame, then it fucked up
  }

  if(!space_ptr_->satisfiesBounds(state)){
      ROS_INFO("INVALID STATE: OOB");
      return false;
  }


  return global_map_->isStateValid(state_vector[0], state_vector[1]);
}


GlobalPlanner::~GlobalPlanner(){
  //delete planner_; //PlannerVisualizer actually deletes this because of shared ptr bs.

}

int GlobalPlanner::plan(std::vector<RigidBodyDynamics::Math::Vector2d> &waypoints, float *vehicle_start_state, RigidBodyDynamics::Math::Vector2d goal_pos, float goal_tol){
  // construct the state space we are planning in
  ompl::base::ScopedState<> start(space_ptr_);
  for(int i = 0; i < 17; i++){
    start[i] = vehicle_start_state[i];
  }

  ompl::base::GoalSpace *goal = new ompl::base::GoalSpace(si_);
  ompl::base::VehicleStateSpace *gspace = new ompl::base::VehicleStateSpace(17);
  ompl::base::RealVectorBounds gbounds(17);

  gbounds.setLow(0, goal_pos[0] - goal_tol);
  gbounds.setHigh(0, goal_pos[0] + goal_tol);

  gbounds.setLow(1, goal_pos[1] - goal_tol);
  gbounds.setHigh(1, goal_pos[1] + goal_tol);

  for(int i = 2; i < 17; i++){ //Goal region is only in x and y. Unbounded in other state variables.
    gbounds.setLow(i, -1000); gbounds.setHigh(i, 1000);
  }
  gspace->setBounds(gbounds);


  ompl::base::StateSpacePtr gspace_ptr(gspace);
  goal->setSpace(gspace_ptr);

  pdef_->addStartState(start);

  ompl::base::GoalPtr goal_ptr(goal);
  pdef_->setGoal(goal_ptr);

  planner_->setProblemDefinition(pdef_);


  PlannerVisualizer planner_visualizer(si_, planner_, .5);
  planner_visualizer.setObstacles(global_map_->getObstacles());
  planner_visualizer.setGoal(goal_pos);
  if(GlobalParams::get_visualize_planner()){
    planner_visualizer.startMonitor();
  }


  //float max_runtime = 600; //seconds
  float max_runtime = GlobalParams::get_max_gp_runtime();
  ompl::base::PlannerTerminationCondition ptc = ompl::base::plannerOrTerminationCondition(ompl::base::timedPlannerTerminationCondition(max_runtime), ompl::base::exactSolnPlannerTerminationCondition(pdef_));

  ompl::base::PlannerStatus solved = planner_->solve(ptc);

  if(solved){
    ROS_INFO("\n\nHAS SOLUTION\n\n");
    ompl::base::PlannerSolution soln = pdef_->getSolutions()[0];
    ompl::control::PathControl *path = soln.path_->as<ompl::control::PathControl>();

    std::vector<ompl::base::State*> states = path->getStates();
    std::vector<ompl::control::Control*> controls = path->getControls();
    std::vector<double> durations = path->getControlDurations();

    unsigned num_waypoints;
    getWaypoints(controls, durations, states, waypoints, num_waypoints);
    planner_visualizer.setSolution(waypoints, num_waypoints);

    ROS_INFO("Press Enter\n");
    char getchar;
    std::cin >> getchar;

    planner_visualizer.stopMonitor();

    return EXIT_SUCCESS;
  }
  else{


    ROS_INFO("Press Enter\n");
    char getchar;
    std::cin >> getchar;

    return EXIT_FAILURE;
  }


}


void GlobalPlanner::getWaypoints(std::vector<ompl::control::Control*> &controls, std::vector<double> &durations, std::vector<ompl::base::State*> states, std::vector<RigidBodyDynamics::Math::Vector2d> &waypoints, unsigned &num_waypoints){
  waypoints.clear();

  JackalDynamicSolver::init_model(2);

  unsigned vehicle_state_len = 21;

  ompl::base::State *start_state = si_->allocState();
  ompl::base::State *result_state = si_->allocState();


  //initialize with start state
  si_->copyState(start_state, states[0]);
  const double* start_val = start_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  waypoints.push_back(Vector2d(start_val[0], start_val[1]));


  ROS_INFO("Num controls %ld     num durations %ld    num states %ld", controls.size(), durations.size(), states.size()); //Sanity check.


  //Iterate through the controls.
  for(unsigned i = 0; i < controls.size(); i++){
    const double *control_vector = controls[i]->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

    si_->propagateWhileValid(start_state, controls[i], round(durations[i]/si_->getPropagationStepSize()), result_state);
    si_->copyState(start_state, result_state);

    const double* result_val = result_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    ROS_INFO("Waypoint %d  %f %f", i, result_val[0], result_val[1]);
    waypoints.push_back(Vector2d(result_val[0], result_val[1]));
  }

  num_waypoints = waypoints.size();
}







void GlobalPlanner::dynamicWindowPlan(std::vector<RigidBodyDynamics::Math::Vector2d> &waypoints){
  ompl::base::State *start_state = si_->allocState();
  ompl::base::State *result_state = si_->allocState();
  ompl::control::Control *temp_control = si_->allocControl();

  double *control_vector = temp_control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
  double *result_vector = result_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  double duration = 1; //seconds
  double error;
  double best_error = 100;
  unsigned num_tries = 10;

  for(unsigned i = 0; i < waypoints.size()-1; i++){
    for(unsigned j = 0; j < num_tries; j++){
      control_vector[0] = 0; //W
      control_vector[1] = 0; //Vf

      si_->propagateWhileValid(start_state, temp_control, duration, result_state);

      float dx = result_vector[0] - waypoints[i+1][0];
      float dy = result_vector[1] - waypoints[i+1][1];

      error = sqrtf(dx*dx + dy*dy);

    }
  }


}
